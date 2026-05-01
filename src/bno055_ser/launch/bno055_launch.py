#!/usr/bin/env python3
"""
Launch file for BNO055 Serial IMU driver.

Standalone:
  ros2 launch bno055_ser bno055_launch.py

Override parameters file:
  ros2 launch bno055_ser bno055_launch.py params_file:=/path/to/my_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bno055_ser')
    default_params = os.path.join(pkg, 'config', 'bno055_params.yaml')

    return LaunchDescription([

        # ── Overrideable launch arguments ─────────────────────────────────
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to BNO055 parameter YAML file'
        ),

        # ── BNO055 driver node ────────────────────────────────────────────
        Node(
            package='bno055_ser',
            executable='bno055_node',
            name='bno055_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
