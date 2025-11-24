#!/usr/bin/env python3
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

import rclpy.logging

logger = rclpy.logging.get_logger("rpy_reset_tests_tree.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configs
    record_bag = LaunchConfiguration("record_bag")
    record_loc = LaunchConfiguration("record_loc")

    node_fa_tree = Node(
        package="behavior_trees_python",
        executable="rpy_reset_tests_tree",
        name="rpy_reset_tests_tree",
        emulate_tty=True,
        parameters=[
            {
                "record_bag": record_bag,
                "record_loc": record_loc,
            }
        ],
    )

    _to_launch = [
        node_fa_tree,
    ]

    return _to_launch


def generate_launch_description():
    declared_configs = [
        dict(
            name="record_loc",
            default_value="",
            description="Optional string parameter describing the location of where the trial is run.",
        ),
        dict(
            name="record_bag",
            default_value="false",
            choices=["true", "false"],
            description=r"Records a bag file with format bds_{datetime}.sq3",
        ),
    ]

    declared_args = [
        DeclareLaunchArgument(
            name=config.get("name"),
            default_value=config.get("default_value"),
            choices=config.get("choices"),
            description=config.get("description"),
        )
        for config in declared_configs
    ]

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld
