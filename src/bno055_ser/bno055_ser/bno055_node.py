#!/usr/bin/env python3
"""
BNO055 IMU ROS2 Driver Node (bno055_ser)

Reads BNO055 sensor data from an Arduino Uno via USB serial and publishes to ROS2 topics.
Feature set ported from bno055_driver (I2C, C++) — adapted for the serial bridge.

Features
--------
* All 12 BNO055 operation modes with mode-aware publishing
  (fusion vs non-fusion, unknown-orientation flag per sensor_msgs spec)
* Magnetometer topic:   /bno055/mag         (sensor_msgs/MagneticField)
* Temperature topic:    /bno055/temperature  (sensor_msgs/Temperature)
* Diagnostics topic:    /bno055/status       (diagnostic_msgs/DiagnosticArray, 1 Hz)
  – mode-aware calibration guidance messages
* Calibration persistence: saves offsets on full calibration, restores on startup
* Configurable update rate (the host-side read timer)
* Axis-remap parameter (P0–P7) forwarded to diagnostics
* Datasheet-based covariance values instead of ad-hoc constants

Serial Protocol (from Arduino)
-------------------------------
  BNO,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,ex,ey,ez,temp,
      cal_sys,cal_gyro,cal_accel,cal_mag   ← 22 fields
  CAL_STATUS,sys,gyro,accel,mag
  CAL_COMPLETE
  CAL_OFFSETS,ax,ay,az,mx,my,mz,gx,gy,gz,accel_radius,mag_radius
  CAL_LOADED

Published Topics
----------------
  /bno055/imu          sensor_msgs/Imu
  /bno055/mag          sensor_msgs/MagneticField  (when mode uses magnetometer)
  /bno055/temperature  sensor_msgs/Temperature
  /bno055/status       diagnostic_msgs/DiagnosticArray
  /bno055/calibration  std_msgs/String  (JSON, 1 Hz — legacy compatibility)

Parameters
----------
  serial_port         str    /dev/ttyUSB0
  baud_rate           int    115200
  frame_id            str    bno055_link
  update_rate         float  50.0   Hz — governs the serial-read timer
  operation_mode      str    NDOF   (ACCONLY | MAGONLY | GYROONLY | ACCMAG |
                                     ACCGYRO | MAGGYRO | AMG | IMU | IMUPLUS |
                                     COMPASS | M4G | NDOF_FMC_OFF | NDOF)
  publish_diagnostics bool   true
  publish_temperature bool   true
  publish_magnetometer bool  true
  placement_axis_remap str   P1  (P0–P7, informational — axis remap applied on Arduino)
  cal_file            str    ~/.ros/bno055_cal.json
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import serial
from serial.serialutil import SerialException

# ─── Mode helpers (mirrored from bno055_driver C++) ──────────────────────────

FUSION_MODES     = {'IMU', 'IMUPLUS', 'COMPASS', 'M4G', 'NDOF_FMC_OFF', 'NDOF'}
MAG_MODES        = {'MAGONLY', 'ACCMAG', 'MAGGYRO', 'AMG',
                    'COMPASS', 'M4G', 'NDOF_FMC_OFF', 'NDOF'}
GYRO_MODES       = {'GYROONLY', 'ACCGYRO', 'MAGGYRO', 'AMG',
                    'IMU', 'IMUPLUS', 'NDOF_FMC_OFF', 'NDOF'}
ACCEL_MODES      = {'ACCONLY', 'ACCMAG', 'ACCGYRO', 'AMG',
                    'IMU', 'IMUPLUS', 'COMPASS', 'M4G', 'NDOF_FMC_OFF', 'NDOF'}
KNOWN_MODES      = {'ACCONLY', 'MAGONLY', 'GYROONLY', 'ACCMAG', 'ACCGYRO',
                    'MAGGYRO', 'AMG', 'IMU', 'IMUPLUS', 'COMPASS', 'M4G',
                    'NDOF_FMC_OFF', 'NDOF'}

# Datasheet-based covariances (diagonal element value)
# See bno055_driver/bno055_node.hpp for derivation comments
ORIENT_COV     = 0.0025    # ~(0.05 rad)²   — ±1° heading accuracy
GYRO_COV       = 0.0001    # ~(0.01 rad/s)²
ACCEL_COV      = 0.0036    # ~(0.06 m/s²)²
MAG_COV        = 0.09e-12  # ~(0.3 µT)² converted to T² (BNO055 outputs µT)


def _diag(v: float) -> list:
    return [v, 0.0, 0.0,  0.0, v, 0.0,  0.0, 0.0, v]


class BNO055Node(Node):
    """ROS2 node for BNO055 IMU via Arduino serial bridge."""

    def __init__(self):
        super().__init__('bno055_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port',          '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',            115200)
        self.declare_parameter('frame_id',             'bno055_link')
        self.declare_parameter('update_rate',          50.0)
        self.declare_parameter('operation_mode',       'NDOF')
        self.declare_parameter('publish_diagnostics',  True)
        self.declare_parameter('publish_temperature',  True)
        self.declare_parameter('publish_magnetometer', True)
        self.declare_parameter('placement_axis_remap', 'P1')
        self.declare_parameter(
            'cal_file', os.path.expanduser('~/.ros/bno055_cal.json'))

        self.serial_port   = self.get_parameter('serial_port').value
        self.baud_rate     = self.get_parameter('baud_rate').value
        self.frame_id      = self.get_parameter('frame_id').value
        self.update_rate   = float(self.get_parameter('update_rate').value)
        self.placement     = self.get_parameter('placement_axis_remap').value
        self.cal_file      = self.get_parameter('cal_file').value
        self.pub_diag      = self.get_parameter('publish_diagnostics').value
        self.pub_temp      = self.get_parameter('publish_temperature').value
        self.pub_mag       = self.get_parameter('publish_magnetometer').value

        # Validate / normalise operation mode
        mode_raw = self.get_parameter('operation_mode').value.upper()
        if mode_raw not in KNOWN_MODES:
            self.get_logger().warn(
                f"Unknown operation_mode '{mode_raw}', defaulting to NDOF. "
                f"Valid: {sorted(KNOWN_MODES)}"
            )
            mode_raw = 'NDOF'
        # Normalise alias
        self.operation_mode = 'IMU' if mode_raw == 'IMUPLUS' else mode_raw

        self.is_fusion = self.operation_mode in FUSION_MODES
        self.uses_mag  = self.operation_mode in MAG_MODES
        self.uses_gyro = self.operation_mode in GYRO_MODES
        self.uses_accel= self.operation_mode in ACCEL_MODES

        self.get_logger().info(
            f"Mode: {self.operation_mode}  fusion={self.is_fusion}  "
            f"mag={self.uses_mag}  gyro={self.uses_gyro}  accel={self.uses_accel}  "
            f"placement={self.placement}"
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.imu_pub  = self.create_publisher(Imu,    '/bno055/imu', 10)
        self.cal_pub  = self.create_publisher(String, '/bno055/calibration', 10)

        self.mag_pub  = None
        if self.pub_mag and self.uses_mag:
            self.mag_pub = self.create_publisher(
                MagneticField, '/bno055/mag', 10)
            self.get_logger().info('Magnetometer publisher enabled on /bno055/mag')

        self.temp_pub = None
        if self.pub_temp:
            self.temp_pub = self.create_publisher(
                Temperature, '/bno055/temperature', 10)

        self.diag_pub = None
        if self.pub_diag:
            self.diag_pub = self.create_publisher(
                DiagnosticArray, '/bno055/status', 10)

        # ── State ────────────────────────────────────────────────────────────
        self.ser       = None
        self.connected = False
        self.calibration     = {'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        self.fully_calibrated = False

        # Latest temperature for diagnostics (updated from data lines)
        self._last_temp = 0.0

        # ── Connect & timers ─────────────────────────────────────────────────
        self.connect_serial()

        period = 1.0 / self.update_rate
        self.timer     = self.create_timer(period, self.timer_callback)
        self.cal_timer = self.create_timer(1.0, self.publish_calibration)
        if self.diag_pub:
            self.diag_timer = self.create_timer(1.0, self.publish_diagnostics_cb)

        self.get_logger().info(
            f'BNO055 node started  port={self.serial_port}  '
            f'baud={self.baud_rate}  rate={self.update_rate} Hz  '
            f'cal_file={self.cal_file}'
        )

    # ── Calibration persistence ───────────────────────────────────────────────

    def _load_cal_offsets(self):
        if not os.path.isfile(self.cal_file):
            return None
        try:
            with open(self.cal_file) as f:
                data = json.load(f)
            required = [
                'accel_offset_x', 'accel_offset_y', 'accel_offset_z',
                'mag_offset_x',   'mag_offset_y',   'mag_offset_z',
                'gyro_offset_x',  'gyro_offset_y',  'gyro_offset_z',
                'accel_radius',   'mag_radius',
            ]
            if all(k in data for k in required):
                self.get_logger().info(f'Loaded calibration from {self.cal_file}')
                return data
            self.get_logger().warn('Calibration file missing fields — ignoring.')
        except Exception as e:
            self.get_logger().warn(f'Could not read calibration file: {e}')
        return None

    def _save_cal_offsets(self, offsets: dict):
        try:
            os.makedirs(os.path.dirname(self.cal_file), exist_ok=True)
            with open(self.cal_file, 'w') as f:
                json.dump(offsets, f, indent=2)
            self.get_logger().info(f'Calibration saved to {self.cal_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')

    def _send_load_cal(self, offsets: dict):
        cmd = (
            f"LOAD_CAL,"
            f"{offsets['accel_offset_x']},{offsets['accel_offset_y']},{offsets['accel_offset_z']},"
            f"{offsets['mag_offset_x']},{offsets['mag_offset_y']},{offsets['mag_offset_z']},"
            f"{offsets['gyro_offset_x']},{offsets['gyro_offset_y']},{offsets['gyro_offset_z']},"
            f"{offsets['accel_radius']},{offsets['mag_radius']}\n"
        )
        self.ser.write(cmd.encode())
        self.get_logger().info('Sent stored calibration offsets to Arduino.')

    def _parse_cal_offsets_line(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 12:
                self.get_logger().warn(
                    f'CAL_OFFSETS: expected 12 fields, got {len(parts)}')
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
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(
                port=self.serial_port, baudrate=self.baud_rate, timeout=1.0)

            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.connected = True
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')

            saved = self._load_cal_offsets()
            start = time.time()
            while time.time() - start < 5.0:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'Arduino: {line}')
                    if 'BNO_READY' in line:
                        if saved:
                            time.sleep(0.1)
                            self._send_load_cal(saved)
                            self.fully_calibrated = True
                        return True
                    elif 'BNO_ERROR' in line:
                        self.get_logger().error('BNO055 init failed! Check wiring.')
                        return False

            self.get_logger().warn('Did not receive BNO_READY, proceeding anyway...')
            return True

        except SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.connected = False
            return False

    # ── Main timer ────────────────────────────────────────────────────────────

    def timer_callback(self):
        if not self.connected:
            self.get_logger().warn(
                'Serial disconnected. Attempting reconnection...',
                throttle_duration_sec=5.0)
            self.connect_serial()
            return

        try:
            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if line.startswith('BNO,'):
                self._parse_and_publish(line)
            elif line.startswith('CAL_STATUS,'):
                self._parse_calibration_status(line)
            elif line == 'CAL_COMPLETE':
                self.get_logger().info(
                    '*** BNO055 FULLY CALIBRATED (3,3,3,3) — offsets incoming ***')
                self.fully_calibrated = True
            elif line.startswith('CAL_OFFSETS,'):
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

    # ── Data parsing ──────────────────────────────────────────────────────────

    def _parse_and_publish(self, line: str):
        """
        Parse the 22-field CSV data line from Arduino and publish all topics.

        Format: BNO,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,ex,ey,ez,temp,
                    cal_sys,cal_gyro,cal_accel,cal_mag
        """
        try:
            parts = line.split(',')
            if len(parts) != 22:
                self.get_logger().warn(
                    f'Invalid BNO data (expected 22 fields, got {len(parts)}): {line}')
                return

            qw, qx, qy, qz = float(parts[1]), float(parts[2]), \
                               float(parts[3]), float(parts[4])
            gx, gy, gz      = float(parts[5]),  float(parts[6]),  float(parts[7])
            ax, ay, az      = float(parts[8]),  float(parts[9]),  float(parts[10])
            mx, my, mz      = float(parts[11]), float(parts[12]), float(parts[13])
            # euler: heading(x), roll(y), pitch(z) in degrees
            ex, ey, ez      = float(parts[14]), float(parts[15]), float(parts[16])
            temp            = float(parts[17])
            cal_sys         = int(parts[18])
            cal_gyro        = int(parts[19])
            cal_accel       = int(parts[20])
            cal_mag         = int(parts[21])

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse IMU data: {e}')
            return

        self.calibration = {
            'sys': cal_sys, 'gyro': cal_gyro,
            'accel': cal_accel, 'mag': cal_mag
        }
        self._last_temp = temp

        stamp = self.get_clock().now().to_msg()

        # ── IMU message ────────────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = self.frame_id

        if self.is_fusion:
            # Fusion modes: valid quaternion from on-chip engine
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation_covariance = _diag(ORIENT_COV)
        else:
            # Non-fusion: orientation unknown — set covariance[0] = -1 per spec
            imu_msg.orientation_covariance[0] = -1.0

        if self.uses_gyro:
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.angular_velocity_covariance = _diag(GYRO_COV)
        else:
            imu_msg.angular_velocity_covariance[0] = -1.0

        if self.is_fusion:
            # Fusion: gravity-compensated linear acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance = _diag(ACCEL_COV)
        elif self.uses_accel:
            # Non-fusion: raw accelerometer (includes gravity)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance = _diag(ACCEL_COV)
        else:
            imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        # ── Magnetometer ───────────────────────────────────────────────────
        if self.mag_pub and self.uses_mag:
            mag_msg = MagneticField()
            mag_msg.header.stamp    = stamp
            mag_msg.header.frame_id = self.frame_id
            # BNO055 outputs µT — sensor_msgs/MagneticField expects Tesla
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6
            mag_msg.magnetic_field_covariance = _diag(MAG_COV)
            self.mag_pub.publish(mag_msg)

        # ── Temperature ────────────────────────────────────────────────────
        if self.temp_pub:
            t_msg = Temperature()
            t_msg.header.stamp    = stamp
            t_msg.header.frame_id = self.frame_id
            t_msg.temperature     = temp
            t_msg.variance        = 1.0   # ±1 °C per BNO055 datasheet
            self.temp_pub.publish(t_msg)

    def _parse_calibration_status(self, line: str):
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

    # ── Periodic publishers ───────────────────────────────────────────────────

    def publish_calibration(self):
        """Publish calibration JSON on legacy /bno055/calibration topic."""
        msg = String()
        msg.data = json.dumps(self.calibration)
        self.cal_pub.publish(msg)

    def publish_diagnostics_cb(self):
        """Publish mode-aware diagnostic status at 1 Hz."""
        if not self.diag_pub:
            return

        cal = self.calibration
        sys_v, gyro_v, accel_v, mag_v = (
            cal['sys'], cal['gyro'], cal['accel'], cal['mag'])

        # Determine fully-calibrated based on mode (mirrors bno055_driver logic)
        if not self.is_fusion:
            ok = True
            if self.uses_accel and accel_v < 3: ok = False
            if self.uses_gyro  and gyro_v  < 3: ok = False
            if self.uses_mag   and mag_v   < 3: ok = False
            full_cal = ok
        elif not self.uses_mag:
            # Fusion without mag (IMU mode): ignore mag calibration
            full_cal = sys_v == 3 and gyro_v == 3 and accel_v == 3
        else:
            full_cal = sys_v == 3 and gyro_v == 3 and accel_v == 3 and mag_v == 3

        status = DiagnosticStatus()
        status.name        = 'BNO055 IMU'
        status.hardware_id = 'bno055_serial'

        if full_cal:
            status.level   = DiagnosticStatus.OK
            status.message = (
                f'Fully calibrated, running in {self.operation_mode} mode')
        else:
            status.level = DiagnosticStatus.WARN
            # Mode-aware guidance (mirrors bno055_node.cpp diagnosticsTimerCallback)
            if not self.uses_mag:
                if self.uses_gyro and gyro_v < 3:
                    status.message = 'Calibration incomplete — hold sensor still for gyroscope'
                elif self.is_fusion and sys_v < 3:
                    status.message = 'Sensors ready — waiting for fusion engine (sys), hold still'
                elif self.uses_accel and accel_v < 3:
                    status.message = 'Calibration incomplete — slowly rotate sensor for accelerometer'
                else:
                    status.message = 'Calibration incomplete'
            else:
                if mag_v < 3:
                    status.message = 'Calibration incomplete — rotate sensor in figure-8 for magnetometer'
                elif self.uses_gyro and gyro_v < 3:
                    status.message = 'Calibration incomplete — hold sensor still for gyroscope'
                elif self.is_fusion and sys_v < 3:
                    status.message = 'Sensors ready — waiting for fusion engine (sys) to converge'
                else:
                    status.message = 'Calibration incomplete'

            # Throttled log
            self.get_logger().info(
                f'[{self.operation_mode}] Calibration: '
                f'sys={sys_v} gyro={gyro_v} accel={accel_v} mag={mag_v} — '
                f'{status.message}',
                throttle_duration_sec=10.0
            )

        def kv(k, v):
            item = KeyValue(); item.key = k; item.value = str(v)
            return item

        status.values = [
            kv('operation_mode',    self.operation_mode),
            kv('fusion_mode',       str(self.is_fusion)),
            kv('placement',         self.placement),
            kv('calib_sys',         f'{sys_v}/3'),
            kv('calib_gyro',        f'{gyro_v}/3'),
            kv('calib_accel',       f'{accel_v}/3'),
            kv('calib_mag',         f'{mag_v}/3'),
            kv('temperature_degC',  f'{self._last_temp:.1f}'),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(status)
        self.diag_pub.publish(arr)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
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
