#!/usr/bin/env python3
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

import rclpy.logging

logger = rclpy.logging.get_logger("reset_tests_tree.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configs

    node_fa_tree = Node(
        package="behavior_trees_python",
        executable="reset_tests_tree",
        name="reset_tests_tree",
        emulate_tty=True,
    )


    _to_launch = [
        node_fa_tree,
    ]

    return _to_launch


def generate_launch_description():
    declared_args = []

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld
