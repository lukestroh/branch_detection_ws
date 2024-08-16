#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context, *args, **kwargs) -> list:
    """ToF launch setup"""

    nodes_to_run = []

    return nodes_to_run



def generate_launch_description():
    ENV_ROS_DOMAIN_ID = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0")

    declared_args = []
    # Generic
    declared_args.append(
        DeclareLaunchArgument(
            name="use_sim",
            default_value="false",
            description="True when testing the setup in simulation. When fixed to actual UR hardware, set to false."
        )
    ) 
    # UR Robot
    declared_args.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Robot description name (required for URDF parsing)."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_robot_ip",
            default_value="169.254.174.50",
            description="UR robot IP"
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            name="teensy_serial_port",
            default_value="/dev/ttyACM0",
            description="Port name for serial device."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            name="tof_demo_mode",
            default_value="false", # TODO: change to false for production
            description="Command to run the ToF stuff independently for testing purposes."
        )
    )

    # ===============================
    # Launch configuration settings
    # ===============================
    # Generic
    use_sim = LaunchConfiguration("use_sim")
    # UR robot
    ur_type = LaunchConfiguration("ur_type")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    # ToF
    teensy_serial_port = LaunchConfiguration("teensy_serial_port")
    tof_demo_mode = LaunchConfiguration("tof_demo_mode")


    launch_ur_basic = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("branch_detection_system_bringup"),
                "launch",
                "ur_basic.launch.py"
            )
        ),
        launch_arguments=[
            ("ur_type", ur_type),
            ("ur_robot_ip", ur_robot_ip),
            ("use_mock_hardware", use_sim),
        ],
        condition=UnlessCondition(tof_demo_mode)
    )

    launch_tof_processor = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("teensy32_tof_bringup"), # TODO: change to just "teensy", add board type to args
                "launch",
                "tof.launch.py"
            )
        ),
        launch_arguments=[
            ("serial_port", teensy_serial_port),
            ("use_mock_hardware", use_sim),
            ("demo_mode", tof_demo_mode)
        ]
    )

    launch_particle_filter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("particle_filter_bringup"),
                "launch",
                "particle_filter.launch.py"
            )
        )
    )


    ld = LaunchDescription(
        declared_args
        + [
            ENV_ROS_DOMAIN_ID,
            # launch_ur_basic,
            launch_tof_processor,
            launch_particle_filter
        ]
    )

    return ld