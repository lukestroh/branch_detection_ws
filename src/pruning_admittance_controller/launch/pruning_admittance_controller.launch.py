#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

import rclpy.logging
logger = rclpy.logging.get_logger("admittance_controller.launch")

def setup_launch(context: LaunchContext, *args, **kwargs):

    use_fake_force_data = LaunchConfiguration("use_fake_force_data")


    dir_admittance_controller_pkg = get_package_share_directory("admittance_controller")
    admittance_controller_conf = os.path.join(dir_admittance_controller_pkg, "config", "admittance_controller.yaml")

    node_contact_watcher = Node(
        package="pruning_admittance_controller",
        executable="contact_watcher_node",
        name="contact_watcher",
        output="both",
        parameters=[
            admittance_controller_conf,
        ],
    )

    node_wrench_filter = Node(
        package="pruning_admittance_controller",
        executable="wrench_filter_node",
        name="wrench_filter",
        output="both",
        parameters=[
            admittance_controller_conf,
        ],
    )

    node_fake_wrench_publisher = Node(
        package="pruning_admittance_controller",
        executable="fake_wrench_pub_node",
        name="fake_wrench_publisher",
        output="screen",
        condition=IfCondition(use_fake_force_data)
    )

    _to_launch = [
        node_contact_watcher,
        node_wrench_filter,
        node_fake_wrench_publisher
    ]

    return _to_launch


def generate_launch_description():
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(name="use_fake_force_data",default_value="false", choices=['true', 'false'], description="Publish fake force/torque data for the admittance controller")
    )

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=setup_launch)])

    return ld

