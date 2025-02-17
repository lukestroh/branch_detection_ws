#!/usr/bin/env python3
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

import rclpy.logging

logger = rclpy.logging.get_logger("io_processor_tree.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Launch configs
    robot_base_part = LaunchConfiguration('robot_base_part')
    robot_eef_part = LaunchConfiguration('robot_eef_part')


    node_fa_tree = Node(
        package="behavior_trees_python",
        executable="teleop_node",
        name="teleop_node",
        emulate_tty=True,
    )

    node_io_processor = Node(
        package="behavior_trees_python",
        executable="io_tree_node",
        name="io_tree_node",
        emulate_tty=True
    )

    node_io_manager = Node(
        package='behavior_trees_python',
        executable='io_manager_node',
        name='io_manager_node',
    )

    node_set_point_service = Node(
        package='behavior_trees_python',
        executable='set_point_service_node',
        name='set_point_service_node',
    )

    node_set_point_from_endpoint_service = Node(
        package='behavior_trees_python',
        executable='set_point_from_endpoint_service_node',
        name='set_point_from_endpoint_service_node',
        parameters=[
            {'robot_base_part': robot_base_part},
            {'robot_eef_part': robot_eef_part}

        ]
    )

    node_joystick = Node(
        package="joy",
        executable='joy_node',
        name='joystick_node',
    )

    

    _to_launch = [
        node_fa_tree,
        node_io_processor,
        node_io_manager,
        node_joystick,
        node_set_point_service,
        node_set_point_from_endpoint_service
    ]

    return _to_launch


def generate_launch_description():
    declared_configs = [
        dict(name="robot_base_part", default_value='amiga'),
        dict(name='robot_eef_part', default_value='mock_pruner')
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
