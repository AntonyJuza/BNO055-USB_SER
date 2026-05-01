#!/usr/bin/env python3
"""
BNO055 IMU ROS2 Driver Node

Reads BNO055 sensor data from Arduino Uno via USB serial and publishes
to ROS2 topics. Supports calibration monitoring, automatic reconnection,
and **persistent calibration offset storage** so offsets survive reboots.

Calibration flow:
  1. On startup, if a calibration file exists, the saved offsets are sent to
     the Arduino via a LOAD_CAL command before the sensor enters NDOF mode.
  2. When the Arduino reports CAL_COMPLETE it also sends a CAL_OFFSETS line
     containing all 11 hardware offset values.
  3. The node saves those offsets to ``cal_file`` (default:
     ~/.ros/bno055_cal.json) so they persist across reboots.

Published Topics:
    /bno055/imu          (sensor_msgs/Imu)    - fused orientation data
    /bno055/calibration  (std_msgs/String)    - JSON calibration status

Parameters:
    serial_port  (str):  Serial port path (default: /dev/ttyUSB0)
    baud_rate    (int):  Serial baud rate  (default: 115200)
    frame_id     (str):  TF frame ID       (default: bno055_link)
    cal_file     (str):  Path to calibration JSON (default: ~/.ros/bno055_cal.json)
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import serial
from serial.serialutil import SerialException


class BNO055Node(Node):
    """ROS2 node for BNO055 IMU via Arduino serial bridge."""

    def __init__(self):
        super().__init__('bno055_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'bno055_link')
        self.declare_parameter(
            'cal_file',
            os.path.expanduser('~/.ros/bno055_cal.json')
        )

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate   = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id    = self.get_parameter('frame_id').get_parameter_value().string_value
        self.cal_file    = self.get_parameter('cal_file').get_parameter_value().string_value

        # ── Publishers ───────────────────────────────────────────────────────
        self.imu_pub = self.create_publisher(Imu,    '/bno055/imu',         10)
        self.cal_pub = self.create_publisher(String, '/bno055/calibration', 10)

        # ── State ────────────────────────────────────────────────────────────
        self.ser       = None
        self.connected = False

        self.calibration     = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        self.fully_calibrated = False

        # ── Connect ──────────────────────────────────────────────────────────
        self.connect_serial()

        # ── Timers ───────────────────────────────────────────────────────────
        self.timer     = self.create_timer(0.01, self.timer_callback)       # 100 Hz
        self.cal_timer = self.create_timer(1.0,  self.publish_calibration)  # 1 Hz

        self.get_logger().info(
            f'BNO055 node started on {self.serial_port} @ {self.baud_rate}  '
            f'| cal_file: {self.cal_file}'
        )

    # ── Calibration persistence ───────────────────────────────────────────────

    def _load_cal_offsets(self):
        """Load saved calibration offsets from disk. Returns dict or None."""
        if not os.path.isfile(self.cal_file):
            return None
        try:
            with open(self.cal_file, 'r') as f:
                data = json.load(f)
            # Validate required keys
            required = [
                'accel_offset_x', 'accel_offset_y', 'accel_offset_z',
                'mag_offset_x',   'mag_offset_y',   'mag_offset_z',
                'gyro_offset_x',  'gyro_offset_y',  'gyro_offset_z',
                'accel_radius',   'mag_radius',
            ]
            if all(k in data for k in required):
                self.get_logger().info(f'Loaded calibration from {self.cal_file}')
                return data
            else:
                self.get_logger().warn('Calibration file is missing fields — ignoring.')
                return None
        except Exception as e:
            self.get_logger().warn(f'Could not read calibration file: {e}')
            return None

    def _save_cal_offsets(self, offsets: dict):
        """Persist calibration offsets to disk."""
        try:
            os.makedirs(os.path.dirname(self.cal_file), exist_ok=True)
            with open(self.cal_file, 'w') as f:
                json.dump(offsets, f, indent=2)
            self.get_logger().info(f'Calibration saved to {self.cal_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')

    def _send_load_cal(self, offsets: dict):
        """Send a LOAD_CAL command to the Arduino with stored offsets."""
        cmd = (
            f"LOAD_CAL,"
            f"{offsets['accel_offset_x']},{offsets['accel_offset_y']},{offsets['accel_offset_z']},"
            f"{offsets['mag_offset_x']},{offsets['mag_offset_y']},{offsets['mag_offset_z']},"
            f"{offsets['gyro_offset_x']},{offsets['gyro_offset_y']},{offsets['gyro_offset_z']},"
            f"{offsets['accel_radius']},{offsets['mag_radius']}\n"
        )
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().info('Sent stored calibration offsets to Arduino.')

    def _parse_cal_offsets_line(self, line: str):
        """
        Parse a CAL_OFFSETS line from Arduino and save to disk.
        Format: CAL_OFFSETS,ax,ay,az,mx,my,mz,gx,gy,gz,ar,mr
        """
        try:
            parts = line.split(',')
            if len(parts) != 12:
                self.get_logger().warn(
                    f'CAL_OFFSETS: expected 12 fields, got {len(parts)}'
                )
                return
            offsets = {
                'accel_offset_x': int(parts[1]),
                'accel_offset_y': int(parts[2]),
                'accel_offset_z': int(parts[3]),
                'mag_offset_x':   int(parts[4]),
                'mag_offset_y':   int(parts[5]),
                'mag_offset_z':   int(parts[6]),
                'gyro_offset_x':  int(parts[7]),
                'gyro_offset_y':  int(parts[8]),
                'gyro_offset_z':  int(parts[9]),
                'accel_radius':   int(parts[10]),
                'mag_radius':     int(parts[11]),
            }
            self._save_cal_offsets(offsets)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse CAL_OFFSETS: {e}')

    # ── Serial connection ─────────────────────────────────────────────────────

    def connect_serial(self):
        """Establish serial connection to Arduino and restore calibration."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )

            # Wait for Arduino to reset after serial connection
            time.sleep(2.0)
            self.ser.reset_input_buffer()

            self.connected = True
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')

            # Wait for BNO_READY and then send stored offsets (if any)
            saved_offsets = self._load_cal_offsets()
            start_time = time.time()
            ready_received = False

            while time.time() - start_time < 5.0:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'Arduino: {line}')
                    if 'BNO_READY' in line:
                        ready_received = True
                        if saved_offsets:
                            # Small delay so Arduino enters its offset-wait window
                            time.sleep(0.1)
                            self._send_load_cal(saved_offsets)
                            self.fully_calibrated = True
                        return True
                    elif 'BNO_ERROR' in line:
                        self.get_logger().error('BNO055 sensor initialization failed! Check wiring.')
                        return False

            if not ready_received:
                self.get_logger().warn('Did not receive BNO_READY signal, proceeding anyway...')
            return True

        except SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.connected = False
            return False

    # ── Main timer ────────────────────────────────────────────────────────────

    def timer_callback(self):
        """Main loop: read serial data and publish."""
        if not self.connected:
            self.get_logger().warn(
                'Serial disconnected. Attempting reconnection...',
                throttle_duration_sec=5.0
            )
            self.connect_serial()
            return

        try:
            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if line.startswith('BNO,'):
                self.parse_and_publish_imu(line)

            elif line.startswith('CAL_STATUS,'):
                self.parse_calibration_status(line)

            elif line == 'CAL_COMPLETE':
                self.get_logger().info('*** BNO055 FULLY CALIBRATED (3,3,3,3) — waiting for offsets ***')
                self.fully_calibrated = True

            elif line.startswith('CAL_OFFSETS,'):
                # Arduino sends this right after CAL_COMPLETE
                self._parse_cal_offsets_line(line)

            elif line == 'CAL_LOADED':
                self.get_logger().info('Arduino confirmed: calibration offsets loaded.')

            elif not line.startswith('BNO'):
                self.get_logger().debug(f'Arduino: {line}')

        except (SerialException, OSError) as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            self.connected = False

    # ── IMU parsing ───────────────────────────────────────────────────────────

    def parse_and_publish_imu(self, line):
        """Parse CSV IMU data and publish as sensor_msgs/Imu."""
        try:
            parts = line.split(',')
            if len(parts) != 15:
                self.get_logger().warn(
                    f'Invalid BNO data (expected 15 fields, got {len(parts)})'
                )
                return

            # index 0 is "BNO"
            qw = float(parts[1]);  qx = float(parts[2])
            qy = float(parts[3]);  qz = float(parts[4])
            gx = float(parts[5]);  gy = float(parts[6]);  gz = float(parts[7])
            ax = float(parts[8]);  ay = float(parts[9]);  az = float(parts[10])
            cal_sys   = int(parts[11])
            cal_gyro  = int(parts[12])
            cal_accel = int(parts[13])
            cal_mag   = int(parts[14])

            self.calibration = {
                'sys': cal_sys, 'gyro': cal_gyro,
                'accel': cal_accel, 'mag': cal_mag
            }

            imu_msg = Imu()
            imu_msg.header.stamp    = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz

            orient_cov = 0.01 if cal_sys >= 2 else 0.1
            imu_msg.orientation_covariance = [
                orient_cov, 0.0, 0.0,
                0.0, orient_cov, 0.0,
                0.0, 0.0, orient_cov
            ]

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            gyro_cov = 0.01 if cal_gyro >= 2 else 0.1
            imu_msg.angular_velocity_covariance = [
                gyro_cov, 0.0, 0.0,
                0.0, gyro_cov, 0.0,
                0.0, 0.0, gyro_cov
            ]

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            accel_cov = 0.01 if cal_accel >= 2 else 0.1
            imu_msg.linear_acceleration_covariance = [
                accel_cov, 0.0, 0.0,
                0.0, accel_cov, 0.0,
                0.0, 0.0, accel_cov
            ]

            self.imu_pub.publish(imu_msg)

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse IMU data: {e}')

    # ── Calibration status ────────────────────────────────────────────────────

    def parse_calibration_status(self, line):
        """Parse calibration status update from Arduino."""
        try:
            parts = line.split(',')
            if len(parts) == 5:
                self.calibration = {
                    'sys':   int(parts[1]),
                    'gyro':  int(parts[2]),
                    'accel': int(parts[3]),
                    'mag':   int(parts[4]),
                }
                self.get_logger().info(
                    f'Calibration: sys={self.calibration["sys"]} '
                    f'gyro={self.calibration["gyro"]} '
                    f'accel={self.calibration["accel"]} '
                    f'mag={self.calibration["mag"]}'
                )
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse calibration: {e}')

    def publish_calibration(self):
        """Publish calibration status as JSON string at 1 Hz."""
        cal_msg      = String()
        cal_msg.data = json.dumps(self.calibration)
        self.cal_pub.publish(cal_msg)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down BNO055 node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
