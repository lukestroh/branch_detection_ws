#!/usr/bin/env python3

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup_launch(context: LaunchContext, *args, **kwargs):

    node_move_arm = Node(
        package="branch_detection_system_moveit_interface",
        executable="move_arm",
        name='move_arm'
    )

    _to_launch = [
        node_move_arm
    ]

    return _to_launch


def generate_launch_description():

    declared_args = []

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld