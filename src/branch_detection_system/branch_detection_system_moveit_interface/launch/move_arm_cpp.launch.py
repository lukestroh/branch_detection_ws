#!/usr/bin/env python3

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_launch(context: LaunchContext, *args, **kwargs):
    robot_base_part = LaunchConfiguration("robot_base_part")

    node_move_arm = Node(
        package="branch_detection_system_moveit_interface",
        executable="move_arm",
        name="move_arm",
        parameters=[("robot_base_part", robot_base_part)],
    )

    _to_launch = [node_move_arm]

    return _to_launch


def generate_launch_description():

    declared_args = []

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld
