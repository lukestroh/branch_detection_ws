#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    declared_args = []

    launch_fr3 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("franka_bringup"),
                "launch",
                "franka.launch.py"
            )
        ),
        launch_arguments=[

        ]
    )

    ld = LaunchDescription(
        declared_args
        + [
            launch_fr3
          ]
    )

    return ld

