#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_context import LaunchContext
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

from ur_moveit_config.launch_common import load_yaml

import os
import json

import rclpy.logging

logger = rclpy.logging.get_logger("vl6180.launch")


def setup_launch(context: LaunchContext, *args, **kwargs):

    dir_vl6180_bringup = get_package_share_directory("vl6180_bringup")
    filepath_vl6180_config = os.path.join(dir_vl6180_bringup, "config", "vl6180.yaml")
    # params_vl6180 = load_yaml(package_name='vl6180_bringup', file_path="config/vl6180.yaml")

    # logger.warn(f"{os.path.exists(filepath_vl6180_config)}")
    # logger.warn(f"{params_vl6180}")

    use_plot_juggler = LaunchConfiguration("use_plot_juggler")

    node_vl6180_filtered = Node(
        package="vl6180_bringup",
        executable="vl6180_filtered_node",
        name="vl6180_filtered_node",
        output="screen",
        parameters=[
            filepath_vl6180_config,
        ],
    )

    node_plot_juggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler_vl6180',
        arguments=[
            '-l',
            os.path.join(get_package_share_directory('vl6180_bringup'), 'plotjuggler/plotjuggler_config.xml')
        ],
        condition=IfCondition(use_plot_juggler)
    )

    nodes_to_launch = [
        node_vl6180_filtered,
        node_plot_juggler
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_args = []
    declared_args.append(DeclareLaunchArgument("use_plot_juggler", default_value="false"))

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld
