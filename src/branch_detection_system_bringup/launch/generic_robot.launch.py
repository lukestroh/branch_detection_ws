#!/usr/bin/env python3

from launch import LaunchDescription, LaunchContext

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os

from moveit_configs_utils import MoveItConfigsBuilder


import rclpy.logging

logger = rclpy.logging.get_logger("generic_robot.launch")


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    # ============================
    #       Launch Configs
    # ============================
    ur_type = LaunchConfiguration("ur_type")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")

    branch_detection_system_bringup_pkg = LaunchConfiguration("branch_detection_system_bringup_pkg")
    robot_description_file = LaunchConfiguration("robot_description_file")

    robot_description_mappings = {}

    # ============================
    #     MoveItConfigsBuilder
    # ============================
    mcb = MoveItConfigsBuilder(
        robot_name="robot",
        # robot_description=os.path.join(
        #     FindPackageShare(branch_detection_system_bringup_pkg).perform(context=context),
        #     "urdf",
        #     robot_description_file.perform(context=context),
        # ),
        package_name="branch_detection_system_bringup",
    )
    mcb.robot_description(
        file_path=os.path.join(
            FindPackageShare(branch_detection_system_bringup_pkg).perform(context=context),
            "urdf",
            robot_description_file.perform(context=context),
        ),
        mappings=robot_description_mappings,
    )

    moveit_configs = mcb.to_moveit_configs()

    logger.warn(moveit_configs.robot_description)

    _to_run = [moveit_configs.robot_description]

    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(
            name="branch_detection_system_bringup_pkg",
            default_value="branch_detection_system_bringup",
            description="Runtime package for system bringup.",
        ),
        dict(
            name="headless_mode", default_value="true", description="Run the UR robot in headless mode (recommended)."
        ),
        dict(
            name="initial_ur_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Default UR robot controller",
        ),
        dict(
            name="robot_description_file",
            default_value="robot.urdf.xacro",
            description="URDF file for the generic robot",
        ),
        dict(name="ur_type", default_value="ur5e", description="UR arm type"),
        dict(name="ur_robot_ip", default_value="169.254.174.50", description="UR robot IP address"),
        dict(
            name="use_mock_hardware",
            default_value="true",
            description="If true, passes `use_mock_hardware:=true` to all hardware interfaces",
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
