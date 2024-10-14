#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction
)
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

import os

import rclpy.logging
logger = rclpy.logging.get_logger("tof.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Environmental variables
    ENV_ROS_DOMAIN_ID = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0") # TODO: set this in branch_detection_system_bringup

    # Launch configuration settings
    serial_port = LaunchConfiguration("serial_port")
    sensor_type = LaunchConfiguration("sensor_type")
    sensor_quantity = LaunchConfiguration("sensor_quantity")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    dir_tof_bringup = get_package_share_directory("tof_bringup")
    dir_vl53l8cx_bringup = get_package_share_directory("vl53l8cx_bringup")
    dir_vl6180_bringup = get_package_share_directory("vl6180_bringup")
    # dir_vl53l0x_bringup = get_package_share_directory("vl53l0x_bringup")
    filepath_node_tof_config = os.path.join(dir_tof_bringup, "config", "node_tof_config.yaml")

    sensor_config_file = PathJoinSubstitution([
        dir_tof_bringup,
        "config",
        sensor_type.perform(context) + ".yaml"
    ])

    # Launch files and nodes
    node_micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=[
            "serial",
            "--dev",
            serial_port,
            f"ROS_DOMAIN_ID={ENV_ROS_DOMAIN_ID}"
        ],
        condition=UnlessCondition(use_mock_hardware)
    )

    launch_vl53l8cx_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(dir_vl53l8cx_bringup, "launch", "vl53l8cx.launch.py"),
        ),
        launch_arguments=[
            ("sensor_quantity", sensor_quantity),
        ],
        condition=IfCondition([str(sensor_type.perform(context) == "vl53l8cx")])
    )
    
    launch_vl6180_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(dir_vl6180_bringup, "launch", "vl6180.launch.py"),
        ),
        launch_arguments=[
            ("sensor_quantity", sensor_quantity),
        ],
        condition=IfCondition([str(sensor_type.perform(context) == "vl6180")])
    )



    nodes_to_return = [
        node_micro_ros_agent,
        launch_vl53l8cx_bringup,
        launch_vl6180_bringup
    ]

    return nodes_to_return


def generate_launch_description():
    declared_args = []
    # Launch configuration settings
    declared_args.append(
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0")
    )
    declared_args.append(
        DeclareLaunchArgument("sensor_type", default_value="vl53l8cx", choices=["vl53l0x", "vl53l8cx", "vl6180"])
    )
    declared_args.append(
        DeclareLaunchArgument("sensor_quantity", default_value="1")
    )
    declared_args.append(
        DeclareLaunchArgument("use_mock_hardware", default_value="false")
    )


    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
