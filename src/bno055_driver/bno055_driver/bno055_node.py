#!/usr/bin/env python3
"""
BNO055 IMU ROS2 Driver Node

Reads BNO055 sensor data from Arduino Uno via USB serial and publishes
to ROS2 topics. Supports calibration monitoring and automatic reconnection.

Published Topics:
    /bno055/imu (sensor_msgs/Imu) - Orientation, angular velocity, linear acceleration
    /bno055/calibration (std_msgs/String) - JSON calibration status {sys, gyro, accel, mag}

Parameters:
    serial_port (str): Serial port path (default: /dev/ttyUSB0)
    baud_rate (int): Serial baud rate (default: 115200)
    frame_id (str): TF frame ID (default: bno055_link)
"""

import json
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

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'bno055_link')

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/bno055/imu', 10)
        self.cal_pub = self.create_publisher(String, '/bno055/calibration', 10)

        # Serial connection
        self.ser = None
        self.connected = False

        # Calibration state
        self.calibration = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        self.fully_calibrated = False

        # Connect to serial
        self.connect_serial()

        # Main timer — reads serial at ~100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Calibration publish timer — 1 Hz
        self.cal_timer = self.create_timer(1.0, self.publish_calibration)

        self.get_logger().info(f'BNO055 node started on {self.serial_port} @ {self.baud_rate}')

    def connect_serial(self):
        """Establish serial connection to Arduino."""
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

            # Flush any startup messages
            self.ser.reset_input_buffer()

            self.connected = True
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')

            # Read startup messages
            start_time = time.time()
            while time.time() - start_time < 3.0:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'Arduino: {line}')
                    if 'BNO_READY' in line:
                        self.get_logger().info('BNO055 sensor initialized successfully!')
                        return True
                    elif 'BNO_ERROR' in line:
                        self.get_logger().error('BNO055 sensor initialization failed! Check wiring.')
                        return False

            self.get_logger().warn('Did not receive BNO_READY signal, proceeding anyway...')
            return True

        except SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.connected = False
            return False

    def timer_callback(self):
        """Main loop: read serial data and publish."""
        if not self.connected:
            # Try to reconnect every call (timer is 10ms, but connect has 2s delay)
            self.get_logger().warn('Serial disconnected. Attempting reconnection...', throttle_duration_sec=5.0)
            self.connect_serial()
            return

        try:
            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                return

            # Parse BNO data line
            if line.startswith('BNO,'):
                self.parse_and_publish_imu(line)

            # Parse calibration status change
            elif line.startswith('CAL_STATUS,'):
                self.parse_calibration_status(line)

            # Log calibration complete
            elif line == 'CAL_COMPLETE':
                self.get_logger().info('*** BNO055 FULLY CALIBRATED (3,3,3,3) ***')
                self.fully_calibrated = True

            # Log other Arduino messages
            elif not line.startswith('BNO'):
                self.get_logger().debug(f'Arduino: {line}')

        except (SerialException, OSError) as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            self.connected = False

    def parse_and_publish_imu(self, line):
        """Parse CSV IMU data and publish as sensor_msgs/Imu."""
        try:
            parts = line.split(',')
            if len(parts) != 15:
                self.get_logger().warn(f'Invalid BNO data (expected 15 fields, got {len(parts)})')
                return

            # Parse values (index 0 is "BNO")
            qw = float(parts[1])
            qx = float(parts[2])
            qy = float(parts[3])
            qz = float(parts[4])
            gx = float(parts[5])
            gy = float(parts[6])
            gz = float(parts[7])
            ax = float(parts[8])
            ay = float(parts[9])
            az = float(parts[10])
            cal_sys = int(parts[11])
            cal_gyro = int(parts[12])
            cal_accel = int(parts[13])
            cal_mag = int(parts[14])

            # Update calibration state
            self.calibration = {
                'sys': cal_sys,
                'gyro': cal_gyro,
                'accel': cal_accel,
                'mag': cal_mag
            }

            # Build IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Orientation (quaternion)
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz

            # Set orientation covariance based on calibration
            # Lower values = more confident. Scale with calibration level.
            orient_cov = 0.01 if cal_sys >= 2 else 0.1
            imu_msg.orientation_covariance = [
                orient_cov, 0.0, 0.0,
                0.0, orient_cov, 0.0,
                0.0, 0.0, orient_cov
            ]

            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            gyro_cov = 0.01 if cal_gyro >= 2 else 0.1
            imu_msg.angular_velocity_covariance = [
                gyro_cov, 0.0, 0.0,
                0.0, gyro_cov, 0.0,
                0.0, 0.0, gyro_cov
            ]

            # Linear acceleration (m/s^2)
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

    def parse_calibration_status(self, line):
        """Parse calibration status update from Arduino."""
        try:
            parts = line.split(',')
            if len(parts) == 5:
                self.calibration = {
                    'sys': int(parts[1]),
                    'gyro': int(parts[2]),
                    'accel': int(parts[3]),
                    'mag': int(parts[4])
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
        cal_msg = String()
        cal_msg.data = json.dumps(self.calibration)
        self.cal_pub.publish(cal_msg)

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
