#!/usr/bin/env python3

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context: LaunchContext, *args, **kwargs):

    launch_fr3 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("franka_bringup"), "launch", "franka.launch.py")
        ),
        launch_arguments=[],
    )

    _to_run = [launch_fr3]

    return _to_run


def generate_launch_description():
    declared_configs = [
        dict(name="fr3_ip_address", default_value="169.254.177.80"),
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
