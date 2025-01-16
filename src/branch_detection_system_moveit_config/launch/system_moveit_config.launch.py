#!/usr/bin/env python3

"""
create an srdf that also autogenerates from macro stack
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml


import os
import pprint as pp

import rclpy.logging

logger = rclpy.logging.get_logger("system_moveit_config.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    # ======================
    #    Launch Configs
    # ======================
    headless_mode = LaunchConfiguration("headless_mode")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    

    # ======================
    #      Controllers
    # ======================
    # MoveIt Trajectory controllers
    filepath_moveit_controllers = PathJoinSubstitution(
        [get_package_share_directory("branch_detection_system_moveit_config"), "config", "moveit_controllers.yaml"]
    )
    parameterfile_moveit_controllers = ParameterFile(filepath_moveit_controllers, allow_substs=True)
    parameterfile_moveit_controllers.evaluate(context=context)
    yamlcontent_moveit_controllers = load_yaml(
        package_name="branch_detection_system_moveit_config",
        file_path=os.path.join("config", str(parameterfile_moveit_controllers.param_file)),
    )
    # The scaled_joint_trajectory_controller does not work on mock_hardware, switch to regular joint_trajectory_controller
    change_controllers = context.perform_substitution(use_mock_hardware)
    if change_controllers.lower() == "true":
        yamlcontent_moveit_controllers["scaled_joint_trajectory_controller"]["default"] = False
        yamlcontent_moveit_controllers["joint_trajectory_controller"]["default"] = True

    moveit_controllers = dict(
        moveit_simple_controller_manager=yamlcontent_moveit_controllers,
        moveit_controller_manager="moveit_simple_controller_manager/MoveItSimpleControllerManager",
    )

    # ======================
    #     Robot config
    # ======================
    robot_conf = load_yaml(
        package_name="branch_detection_system_description", file_path=os.path.join("config", "robot_conf.yaml")
    )

    parent_child_mappings = {}

    # Add the required urdf args from each element of the robot_stack config
    for i, robot_part in enumerate(robot_conf["robot_stack"]):
        robot_part = robot_part.strip().lower()
        # Assign parent frames
        if i == 0:
            parent_child_mappings.update({f"parent{i}": "world"})
        else:
            parent_child_mappings.update({f"parent{i}": robot_conf["robot_stack"][i - 1]})
        # Assign part frame ids
        parent_child_mappings.update({f"robot_part{i}": robot_conf["robot_stack"][i]})
        # Add each robot part's config to the robot_conf
        part_conf = load_yaml(
            package_name="branch_detection_system_description", file_path=os.path.join("config", f"{robot_part}.yaml")
        )
        if part_conf is not None:
            parent_child_mappings.update(part_conf)
        else:
            raise ValueError(f"Robot part {robot_part} not found in 'branch_detection_system_description'")

    _mappings = {
        "name": "pruning_robot",
        "robot_stack_qty": str(len(robot_conf["robot_stack"])),
        "headless_mode": headless_mode,
        "mock_sensor_commands": mock_sensor_commands,
        "use_mock_hardware": use_mock_hardware,
        "use_fake_hardware": use_mock_hardware,  # UR5 humble hasn't updated
        "urdf_base_path": os.path.join(get_package_share_directory("branch_detection_system_description"), "urdf"),
        "mesh_base_path": os.path.join(get_package_share_directory("branch_detection_system_description"), "meshes"),
    }
    _mappings.update(parent_child_mappings)

    mcb = MoveItConfigsBuilder(
        robot_name="branch_detection_system", package_name="branch_detection_system_moveit_config"
    )
    mcb.robot_description(
        file_path=os.path.join(
            get_package_share_directory("branch_detection_system_description"), "urdf/robot/robot.urdf.xacro"
        ),
        mappings=_mappings,
    )
    mcb.robot_description_semantic(
        file_path=os.path.join(
            get_package_share_directory("branch_detection_system_moveit_config"), "srdf/robot.srdf.xacro"
        ),
        mappings=_mappings,
    )
    mcb.robot_description_kinematics(
        file_path=os.path.join(
            get_package_share_directory("branch_detection_system_moveit_config"), "config/kinematics.yaml"
        )
    )
    mcb.planning_pipelines(
        default_planning_pipeline="ompl", pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
    )

    moveit_configs = mcb.to_moveit_configs()
    # logger.warn(f"{moveit_configs.robot_description['robot_description'].value[0].perform(context)}")
    # logger.warn(f"{moveit_configs.robot_description_semantic['robot_description_semantic'].value[0].perform(context)}")

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_controllers,
        ],
    )

    

    _to_start = [node_move_group]

    return _to_start


def generate_launch_description():
    declared_configs = [
        dict(name="headless_mode", default_value="true", choices=["true", "false"]),
        dict(name="mock_sensor_commands", default_value="false", choices=["true", "false"]),
        dict(name="ur_type", default_value="ur5e"),
        dict(name="use_mock_hardware", default_value="false", choices=["true", "false"]),
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

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
