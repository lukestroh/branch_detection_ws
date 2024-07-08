#!/usr/bin/env python3
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_args = []
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
            "use_mock_hardware",
            default_value="true",
            description="If true, passes `use_mock_hardware:=true` to all hardware interfaces."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Run the UR robot in headless mode."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "initial_ur_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Default UR robot controller"
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")

    initial_ur_controller = LaunchConfiguration("initial_ur_controller")

    set_joint_controller = SetLaunchConfiguration(
        "initial_ur_controller",
        value="joint_trajectory_controller",
        condition=LaunchConfigurationEquals("use_mock_hardware", expected_value="true")
    )

    launch_ur_base = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ),
        ),
        launch_arguments=[
            ("robot_ip", ur_robot_ip),
            ("ur_type", ur_type),
            ("use_fake_hardware", use_mock_hardware),
            ("headless_mode", headless_mode),
            ("initial_joint_controller", initial_ur_controller),
            ("launch_rviz", "false")
        ],
    )

    launch_ur_moveit = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            )
        ),
        launch_arguments=[
            ("ur_type", ur_type),
            ("use_fake_hardware", use_mock_hardware),
            ("launch_rviz", "true")
        ]
    )


    ld = LaunchDescription(
        declared_args
        + [
            set_joint_controller,
            launch_ur_base,
            launch_ur_moveit,
        ]
    )

    return ld