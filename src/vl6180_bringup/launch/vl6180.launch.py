#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

import os
import json

import rclpy.logging
logger = rclpy.logging.get_logger("vl6180.launch")

def setup_launch(context: LaunchContext, *args, **kwargs):

    dir_vl6180_bringup = get_package_share_directory("vl6180_bringup")
    filepath_vl6180_config = os.path.join(dir_vl6180_bringup, "config", "vl6180.yaml")

    param_sensor_quantity = LaunchConfiguration("sensor_quantity")

    node_vl6180_filtered = Node(
        package="vl6180_bringup",
        executable="vl6180_filter_node",
        name="vl6180_filter_node",
        output="screen",
        parameters=[
            {"sensor_quantity": param_sensor_quantity},
            filepath_vl6180_config,
        ],
    )

    nodes_to_launch = [
        node_vl6180_filtered,
    ]

    return nodes_to_launch

def generate_launch_description():
    declared_args = []
    declared_args.append(DeclareLaunchArgument("sensor_quantity", default_value="1"))

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld