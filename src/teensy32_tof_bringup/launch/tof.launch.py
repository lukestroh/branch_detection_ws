#!/usr/bin/env python3
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

from ament_index_python.packages import get_package_share_directory


import rclpy.logging
logger = rclpy.logging.get_logger("vl53l0x_launch_logger")


import os
import numpy as np


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Environmental variables
    ENV_ROS_DOMAIN_ID = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0")

    # Launch configuration settings
    serial_port = LaunchConfiguration("serial_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    demo_mode = LaunchConfiguration("demo_mode")
    sensor_type = LaunchConfiguration("sensor_type")    

    dir_runtime_package = get_package_share_directory("teensy32_tof_bringup")
    filepath_tof_node_config = os.path.join(dir_runtime_package, "config", "node_tof_config.yaml")

    filepath_sensor_config = os.path.join(
        dir_runtime_package,
        "config",
        sensor_type.perform(context) + ".yaml"
    )

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

    node_tof_filtered = Node(
        package="teensy32_tof_bringup",
        executable="tof",
        name="tof_node",
        output="screen",
        parameters=[
            filepath_tof_node_config,
            filepath_sensor_config,
            {"demo_mode": demo_mode},
            {"use_mock_hardware": use_mock_hardware},
        ]
    )

    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(dir_runtime_package, 'rviz', "config.rviz")],
        condition=IfCondition(demo_mode) # Use this rviz setting only if running in demo mode
    )

    # Create static transforms TODO: Update these once PCB is made
    # TODO: dyanamically publish these from the node once dynamic sensor plug-and-play is implemented.
    node_tf_tool0_tof0 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = [
            "--x", "0.0",
            "--y", "-0.05",
            "--z", "0.007",
            "--frame-id", "tool0",
            "--child-frame-id", "tof0",
        ],
        output="screen",
        condition=UnlessCondition(demo_mode)
    )
    node_tf_tool0_tof1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = [
            "--x", "0.0",
            "--y", "0.05",
            "--z", "0.007",
            "--frame-id", "tool0",
            "--child-frame-id", "tof1",
        ],
        output="screen",
        condition=UnlessCondition(demo_mode)
    )

    # node_tf_tool0_center = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments = [
    #         "--x", "0.0",
    #         "--y", "0.00",
    #         "--z", "0.007",
    #         "--frame-id", "tool0",
    #         "--child-frame-id", "tof_center",
    #     ],
    #     output="screen",
    #     condition=UnlessCondition(demo_mode)
    # )

    nodes_to_start = [
        ENV_ROS_DOMAIN_ID,
        node_micro_ros_agent,
        node_tof_filtered,
        node_rviz,
        node_tf_tool0_tof0,
        node_tf_tool0_tof1,
        # node_tf_tool0_center
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            name="serial_port",
            default_value="/dev/ttyACM0",
            description="Port name for serial device."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            name="use_mock_hardware",
            default_value="false",
            description="True when testing the setup in simulation. When using actual ToF hardware, set to false."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            name="demo_mode",
            default_value="false",
            description="True when testing *only* the ToF sensor.",
            choices=['true', 'false']
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            name="sensor_type", # TODO: make multiple sensor types possible
            default_value="vl53l0x",
            description="Time-of-flight sensor used in sensing. The type of sensor will dictate how the filters perform.",
            choices = ["vl53l0x"]
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            name="num_sensors",
            default_value="2",
            description="Number of sensors being used"
        )
    )
    
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
