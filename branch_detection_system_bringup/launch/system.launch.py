#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

from ament_index_python.packages import get_package_share_directory
import datetime as dt
import os

import rclpy.logging

logger = rclpy.logging.get_logger("branch_detection_system_bringup.launch")


def launch_setup(context, *args, **kwargs) -> list:
    ENV_ROS_DOMAIN_ID = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0")

    # ===============================
    # Launch configuration settings
    # ===============================

    # High-level launch configs
    record_bag = LaunchConfiguration("record_bag")
    record_loc = LaunchConfiguration("record_loc")

    # Robot parts
    robot_base_part = LaunchConfiguration("robot_base_part")
    robot_eef_part = LaunchConfiguration("robot_eef_part")

    # Hardware
    microros_serial_port = LaunchConfiguration("microros_serial_port")
    tof_sensor_type = LaunchConfiguration("tof_sensor_type")

    # Custom controllers
    use_admittance_controller = LaunchConfiguration("use_admittance_controller")
    use_final_approach_controller = LaunchConfiguration("use_final_approach_controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_behavior_trees_python = LaunchConfiguration("use_behavior_trees_python")

    # UR Robot
    ur_prefix = LaunchConfiguration("ur_prefix")
    ur_type = LaunchConfiguration("ur_type")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    headless_mode = LaunchConfiguration("headless_mode")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    # =======================
    #     Launch files
    # =======================

    launch_ur_basic = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("branch_detection_system_bringup"), "launch", "ur_basic.launch.py")
        ),
        launch_arguments=[
            # ("system_moveit_config_pkg", system_moveit_config_pkg),
            # ("system_description_package", system_description_pkg),
            # ("system_description_file", system_description_file),
            # ("system_semantic_description_file", system_semantic_description_file),
            ("ur_prefix", ur_prefix),
            ("ur_type", ur_type),
            ("ur_robot_ip", ur_robot_ip),
            ("use_mock_hardware", use_mock_hardware),
            ("headless_mode", headless_mode),
            ("mock_sensor_commands", mock_sensor_commands),
        ],
    )

    launch_admittance_controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("pruning_admittance_controller"),
                "launch",
                "admittance_controller.launch.py",
            )
        ),
        condition=IfCondition(use_admittance_controller),
    )

    launch_tof_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tof_bringup"),  # TODO:
                "launch",
                "tof.launch.py",
            )
        ),
        launch_arguments=[
            ("serial_port", microros_serial_port),
            ("sensor_type", tof_sensor_type),
            ("sensor_quantity", "2"),
            ("use_mock_hardware", use_mock_hardware),
        ],
    )

    launch_imu_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ism330dhcx_bringup"), "launch", "ism330dhcx.launch.py"),
        ),
        launch_arguments=[
            ("use_mock_hardware", use_mock_hardware),
        ],
    )

    launch_final_approach_controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("final_approach_controller"),
                "launch",
                "final_approach_controller.launch.py",
            )
        ),
        launch_arguments=[
            ("robot_base_part", robot_base_part),
            ("robot_eef_part", robot_eef_part),
            ("tof_sensor_type", tof_sensor_type),
            ("use_mock_hardware", use_mock_hardware),
        ],
        condition=IfCondition(use_final_approach_controller),
    )

    # launch_particle_filter = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("particle_filter_bringup"), "launch", "particle_filter.launch.py")
    #     )
    # )

    # ###########################
    #      Recording data
    # ###########################

    # _record_loc_str = record_loc.perform(context=context)
    # if _record_loc_str != "":
    #     _record_loc = _record_loc_str + "__"
    # else:
    #     _record_loc = _record_loc_str
    # _filepath_bags = os.path.join(
    #     os.path.expanduser("~"),
    #     "branch_detection_ws",
    #     "bags",
    #     f"bds__{_record_loc}{dt.datetime.strftime(dt.datetime.now(), format=r'%Y%m%d_%H-%M-%S')}",
    # )
    # logger.error(record_bag.perform(context))
    # _execute_process_record_bag = ExecuteProcess(
    #     cmd=[
    #         "ros2 " "bag",
    #         "record ",
    #         "--all ",
    #         "--compression-mode file "  # other option is by `message`
    #         "--compression-format zstd ",
    #         "--output ",
    #         _filepath_bags,
    #     ],
    #     shell=True,
    #     output="screen",
    #     log_cmd=True,
    #     condition=IfCondition(record_bag),
    # )

    node_record_bag = Node(
        package="branch_detection_system_bringup",
        executable="record_bag_node",
        name="record_bag_node",
        output="both",
        parameters=[{"record_bag": record_bag}, {"record_loc": record_loc}],
    )

    _to_run = [
        ENV_ROS_DOMAIN_ID,
        # launch_admittance_controller,
        launch_tof_bringup,
        launch_imu_bringup,
        launch_ur_basic,
        # launch_move_group_control,
        launch_final_approach_controller,
        # process
        # delay_launch_final_approach_controller_after_timeout
        # delay_launch_behavior_trees_timer_action
        # _execute_process_record_bag,
        node_record_bag,
    ]

    return _to_run


def generate_launch_description():

    declared_configs = [
        dict(
            name="record_loc",
            default_value="",
            description="Optional string parameter describing the location of recording the bag.",
        ),
        dict(
            name="record_bag",
            default_value="false",
            choices=["true", "false"],
            description=r"Records a bag file with format bds_{datetime}.sq3",
        ),
        dict(name="headless_mode", default_value="true"),
        dict(name="microros_serial_port", default_value="/dev/ttyACM0", description="Port name for serial device."),
        dict(name="mock_sensor_commands", default_value="false"),
        dict(
            name="tof_sensor_type",
            default_value="vl53l4cd",
            description="tof type",
            choices=["vl53l4cd", "vl53l8cx", "vl6180"],
        ),
        dict(
            name="use_admittance_controller",
            default_value="true",
            description="Launches the admittance controller nodes.",
            choices=["true", "false"],
        ),
        dict(
            name="use_behavior_trees_python",
            default_value="false",
            description="If true, launches behavior_trees Python implementation",
        ),
        dict(
            name="use_final_approach_controller",
            default_value="true",
            description="If true, launches final approach controller",
            choices=["true", "false"],
        ),
        dict(
            name="use_mock_hardware",
            default_value="false",
            description="True when running in neither a simulation environment nor on real hardware.",
            choices=["true", "false"],
        ),
        # dict(
        #     name="system_bringup_pkg",
        #     default_value="branch_detection_system_bringup",
        #     description="Custom UR urdf package",
        # ),
        # dict(name="system_description_file", default_value="robot.urdf.xacro", description="urdf/xacro file"),
        # dict(name="system_semantic_description_file", default_value="robot.srdf", description="srdf/xacro file"),
        # dict(name="system_description_pkg", default_value="branch_detection_system_description"),
        # dict(name="system_moveit_config_pkg", default_value="branch_detection_system_moveit_config"),
        dict(name="ur_prefix", default_value="ur5e__"),
        dict(name="ur_type", default_value="ur5e", description="Robot description name (required for URDF parsing)."),
        dict(name="ur_robot_ip", default_value="169.254.177.230", description="UR robot IP"),
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
