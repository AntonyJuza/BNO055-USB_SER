#!/usr/bin/env python3
"""Launch file for BNO055 IMU driver."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the config file path
    config_file = os.path.join(
        get_package_share_directory('bno055_driver'),
        'config',
        'bno055_params.yaml'
    )

    bno055_node = Node(
        package='bno055_driver',
        executable='bno055_node',
        name='bno055_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        bno055_node,
    ])
