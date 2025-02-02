#!/usr/bin/env python3
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition

from ament_index_python.packages import get_package_share_directory


import os

import rclpy.logging

logger = rclpy.logging.get_logger("final_approach_controller.launch")


def launch_setup(context, *args, **kwargs) -> list:
    robot_base_part = LaunchConfiguration("robot_base_part")
    robot_eef_part = LaunchConfiguration("robot_eef_part")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    node_final_approach_controller = Node(
        package="final_approach_controller",
        executable="final_approach_controller",
        name="final_approach_controller",
        output="both",
    )

    node_cut_point_rotate_axis_controller = Node(
        package="final_approach_controller",
        executable="cut_point_rotate_axis_controller",
        name="cut_point_rotate_axis_controller",
        output="both",
    )

    logger.warn(f"{robot_base_part.perform(context)}")

    node_find_branch_roll_wrist_controller = Node(
        package="final_approach_controller",
        executable="find_branch_roll_wrist_controller",
        name="find_branch_roll_wrist_controller",
        output="both",
        parameters=[
            {"robot_base_part": robot_base_part},
            {'robot_eef_part': robot_eef_part},
            {"use_mock_hardware": use_mock_hardware},
        ],
    )

    _to_return = [
        node_final_approach_controller,
        node_cut_point_rotate_axis_controller,
        node_find_branch_roll_wrist_controller,
    ]

    return _to_return


def generate_launch_description():

    declared_configs = [
        dict(name="robot_base_part", default_value=''),
        dict(name="robot_eef_part", default_value=''),
        dict(name="headless_mode", default_value="true"),
        dict(name="use_mock_hardware", default_value="false"),
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
