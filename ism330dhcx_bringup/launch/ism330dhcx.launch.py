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

logger = rclpy.logging.get_logger("ism330dhcx.launch")


def setup_launch(context: LaunchContext, *args, **kwargs):

    robot_eef_part = LaunchConfiguration("robot_eef_part")

    dir_ism330dhcx_bringup = get_package_share_directory("ism330dhcx_bringup")
    # filepath_ism330dhcx_config = os.path.join(dir_ism330dhcx_bringup, "config", "ism330dhcx.yaml")
    # params_ism330dhcx = load_yaml(package_name='ism330dhcx_bringup', file_path="config/ism330dhcx.yaml")

    # logger.warn(f"{os.path.exists(filepath_ism330dhcx_config)}")
    # logger.warn(f"{params_ism330dhcx}")

    imu_plotjuggler = LaunchConfiguration("imu_plotjuggler")

    node_ism330dhcx_filtered = Node(
        package="ism330dhcx_bringup",
        executable="ism330dhcx_filter_node",
        name="ism330dhcx_filter_node",
        output="screen",
        parameters=[
            # filepath_ism330dhcx_config,
            {"robot_eef_part": robot_eef_part}
        ],
    )

    node_plot_juggler = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler_ism330dhcx",
        arguments=["-l", os.path.join(dir_ism330dhcx_bringup, "plotjuggler/plotjuggler_config.xml")],
        condition=IfCondition(imu_plotjuggler),
    )

    nodes_to_launch = [node_ism330dhcx_filtered, node_plot_juggler]

    return nodes_to_launch


def generate_launch_description():
    declared_args = []
    declared_args.append(DeclareLaunchArgument("imu_plotjuggler", default_value="false"))
    declared_args.append(DeclareLaunchArgument("robot_eef_part", default_value="mock_pruner"))

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld
