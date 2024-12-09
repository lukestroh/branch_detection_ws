#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction
)
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

import os

import rclpy.logging
logger = rclpy.logging.get_logger("view_robot.launch")


def generate_launch_description():
    declared_configs = [
        dict(name='serial_port', default_value='/dev/ttyACM0'),
        dict(name='sensor_type', default_value='vl53l8cx', choices=['vl53l8cx', 'vl6180']),
        dict(name='sensor_quantity', default_value='1'),
        dict(name='use_mock_hardware', default_value='false'),
    ]

    declared_args = [
        DeclareLaunchArgument(
            name=config.get('name'),
            default_value=config.get('default_value'),
            choices=config.get('choices'),
            description=config.get('description')
        ) for config in declared_configs
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])