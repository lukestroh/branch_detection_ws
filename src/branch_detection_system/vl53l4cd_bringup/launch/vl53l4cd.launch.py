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

logger = rclpy.logging.get_logger("vl53l4cd.launch")


def setup_launch(context: LaunchContext, *args, **kwargs):

    robot_eef_part = LaunchConfiguration('robot_eef_part')

    dir_vl53l4cd_bringup = get_package_share_directory("vl53l4cd_bringup")
    filepath_vl53l4cd_config = os.path.join(dir_vl53l4cd_bringup, "config", "vl53l4cd.yaml")
    # params_vl53l4cd = load_yaml(package_name='vl53l4cd_bringup', file_path="config/vl53l4cd.yaml")

    # logger.warn(f"{os.path.exists(filepath_vl53l4cd_config)}")
    # logger.warn(f"{params_vl53l4cd}")

    use_plotjuggler = LaunchConfiguration("use_plotjuggler")

    node_vl53l4cd_filtered = Node(
        package="vl53l4cd_bringup",
        executable="vl53l4cd_filter_node",
        name="vl53l4cd_filter_node",
        output="screen",
        parameters=[
            filepath_vl53l4cd_config,
            {"robot_eef_part": robot_eef_part}
        ],
    )

    node_plot_juggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler_vl53l4cd',
        arguments=[
            '-l',
            os.path.join(get_package_share_directory('vl53l4cd_bringup'), 'plotjuggler/plotjuggler_config.xml')
        ],
        condition=IfCondition(use_plotjuggler)
    )

    nodes_to_launch = [
        node_vl53l4cd_filtered,
        node_plot_juggler
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_args = []
    declared_args.append(DeclareLaunchArgument("use_plotjuggler", default_value="false"))
    declared_args.append(DeclareLaunchArgument('robot_eef_part', default_value='mock_pruner'))

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld
