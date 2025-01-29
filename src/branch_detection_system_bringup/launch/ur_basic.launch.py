#!/usr/bin/env python3
import xml.etree
from launch import LaunchDescription, LaunchContext

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml

import os

import rclpy.logging
logger = rclpy.logging.get_logger("ur_basic.launch")


def launch_setup(context: LaunchContext, *args, **kwargs):
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    launch_rviz = LaunchConfiguration("launch_rviz")

    system_description_package = LaunchConfiguration("system_description_package")
    system_description_file = LaunchConfiguration("system_description_file")
    system_semantic_description_file = LaunchConfiguration("system_semantic_description_file")

    # tf_prefix = LaunchConfiguration("tf_prefix")
    launch_servo = LaunchConfiguration("launch_servo")
    ur_type = LaunchConfiguration("ur_type")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    initial_ur_controller = LaunchConfiguration("initial_ur_controller")
    start_servo_mode = LaunchConfiguration("start_servo_mode")

    reverse_ip = LaunchConfiguration("reverse_ip")
    reverse_port = LaunchConfiguration("reverse_port")
    script_command_port = LaunchConfiguration("script_command_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    trajectory_port = LaunchConfiguration("trajectory_port")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # set_joint_controller = SetLaunchConfiguration(
    #     "initial_ur_controller",
    #     value="joint_trajectory_controller",
    #     condition=LaunchConfigurationEquals("use_mock_hardware", expected_value="true"),
    # )

    #################################################################################################################

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
        "ur_type": ur_type.perform(context),
        "robot_ip": ur_robot_ip.perform(context),
        "tf_prefix": parent_child_mappings["ur_prefix"],
        "robot_stack_qty": str(len(robot_conf["robot_stack"])),
        "headless_mode": headless_mode,
        "mock_sensor_commands": mock_sensor_commands,
        "use_mock_hardware": use_mock_hardware,
        "use_fake_hardware": use_mock_hardware,  # UR5 humble hasn't updated
        # "urdf_base_path": os.path.join(get_package_share_directory("branch_detection_system_description"), "urdf"),
        "mesh_base_path": os.path.join(get_package_share_directory("branch_detection_system_description"), "meshes"),
        "initial_positions_file": os.path.join(
            get_package_share_directory("branch_detection_system_description"), "config/initial_positions.yaml"
        ),
        "kinematics_params_file": os.path.join(get_package_share_directory("branch_detection_system_description"), "config", "cindy_ur5e_calibration.yaml"),
        "joint_limit_params": os.path.join(get_package_share_directory("ur_description"), "config", ur_type.perform(context), "joint_limits.yaml"),
        "physical_params": os.path.join(get_package_share_directory("ur_description"), "config", ur_type.perform(context), "physical_parameters.yaml"),
        "visual_params": os.path.join(get_package_share_directory("ur_description"), "config", ur_type.perform(context), "visual_parameters.yaml"),
        "script_filename": os.path.join(get_package_share_directory("ur_client_library"), "resources", "external_control.urscript"),
        "input_recipe_filename": os.path.join(get_package_share_directory("ur_robot_driver"), "resources", "rtde_input_recipe.txt"),
        "output_recipe_filename": os.path.join(get_package_share_directory("ur_robot_driver"), "resources", "rtde_output_recipe.txt"),
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "script_command_port": "50004",
        "reverse_port": "50001",
        "script_sender_port": "50002",
        "trajectory_port": "50003",
        # "use_tool_communication": "false",


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
    mcb.joint_limits(
        file_path=os.path.join(
            get_package_share_directory("branch_detection_system_moveit_config"), "config/joint_limits.yaml"
        )
    )
    mcb.planning_pipelines(
        default_planning_pipeline="ompl",
        pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
    )
    mcb.planning_scene_monitor()
    mcb.pilz_cartesian_limits(
        file_path=os.path.join(
            get_package_share_directory("linear_slider_moveit_config"),
            "config/pilz_cartesian_limits.yaml",
        )
    )
    mcb.trajectory_execution(
        file_path=os.path.join(
            get_package_share_directory("branch_detection_system_moveit_config"), "config/moveit_controllers.yaml"
        ),
        moveit_manage_controllers=False
    )
    moveit_configs = mcb.to_moveit_configs()

    # logger.error(f"{moveit_configs.robot_description_semantic}")
    
    # ##############################################################
    # # SAVE HARD-CODED URDF
    # import xml.etree.ElementTree as ET
    
    # et = ET.XML(moveit_configs.robot_description['robot_description'].value[0].perform(context))
    # tree = ET.ElementTree(et)
    # ET.indent(tree)
    # tree.write("/home/luke/branch_detection_ws/src/branch_detection_system_description/urdf/tmp/robot.urdf", encoding='utf-8', xml_declaration=True)

    # ##############################################################

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            get_package_share_directory("ur_robot_driver"),
            "config",
            ur_type.perform(context) + "_update_rate.yaml",
        ]
    )
    initial_joint_controllers = PathJoinSubstitution(
        [get_package_share_directory("ur_robot_driver"), "config", "ur_controllers.yaml"]
    )

    node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_configs.robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_mock_hardware),
    )


    # parameterfile_initial_joint_controllers = ParameterFile(initial_joint_controllers, allow_substs=True)
    # parameterfile_initial_joint_controllers.evaluate(context=context)

    # parameterfile_initial_joint_controllers.evaluate(context=context)
    # yamlcontent_servo_config = load_yaml(
    #     package_name="branch_detection_system_moveit_config",
    #     file_path=os.path.join("config", str(parameterfile_initial_joint_controllers.param_file)),
    # )


    node_ur_control = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            moveit_configs.robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
            # yamlcontent_servo_config
        ],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
    )

    node_dashboard_client = Node(
        package="ur_robot_driver",
        condition=IfCondition(AndSubstitution(launch_dashboard_client, NotSubstitution(use_mock_hardware))),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": ur_robot_ip}],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            moveit_configs.robot_description,
            # {"publish_frequency": 100.0},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory("branch_detection_system_description"), "rviz", "view_robot.rviz"]
    )    

    node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ####################################################################################################################
    ####################################################################################################################

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("branch_detection_system_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("branch_detection_system_moveit_config", "config/moveit_controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    # change_controllers = context.perform_substitution(use_mock_hardware)
    # if change_controllers == "true":
    #     controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    #     controllers_yaml["joint_trajectory_controller"]["default"] = True

    # moveit_controllers = {
    #     "moveit_simple_controller_manager": controllers_yaml,
    #     "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    # }
    if use_mock_hardware.perform(context) == "true":
        moveit_configs.trajectory_execution["scaled_joint_trajectory_controller"]["default"] = False
        moveit_configs.trajectory_execution["joint_trajectory_controller"]["default"] = True


    # TODO: If trajectory_execution is part of mcb, check to see if this should be in yaml file?
    params_trajectory_execution = {
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # logger.warn(f"{moveit_configs.trajectory_execution}")


    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    warehouse_server_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_warehouse_services",
        output="screen",
        parameters=[
            warehouse_ros_config,
        ],
    )

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
            ompl_planning_pipeline_config,
            # trajectory_execution,
            # moveit_controllers,
            {"moveit_simple_controller_manager": moveit_configs.trajectory_execution,
             "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            params_trajectory_execution,
            moveit_configs.planning_scene_monitor,
            {"use_sim_time": use_mock_hardware},
            warehouse_ros_config,
        ],
    )

    node_move_arm = Node(
        package="branch_detection_system_moveit_interface",
        executable="move_arm",
        name='move_arm',
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            # moveit_configs.robot_description_kinematics,
        ]
    )

    # MoveIt Servo
    filepath_servo_config = PathJoinSubstitution(
        [
            get_package_share_directory("branch_detection_system_moveit_config"),
            "config",
            "ur_servo.yaml",
        ]
    )
    parameterfile_servo_config = ParameterFile(filepath_servo_config, allow_substs=True)
    parameterfile_servo_config.evaluate(context=context)
    yamlcontent_servo_config = load_yaml(
        package_name="branch_detection_system_moveit_config",
        file_path=os.path.join("config", str(parameterfile_servo_config.param_file)),
    )
    # logger.warn(f"{servo_yaml_content}")
    servo_params = dict(moveit_servo=yamlcontent_servo_config)
    node_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
        ],
        condition=IfCondition(launch_servo),
    )

    node_rviz = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
            ompl_planning_pipeline_config,
            warehouse_ros_config,
        ],
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10",
            ]
            + inactive_flags
            + controllers,
        )
    
    controllers_active = [
        # "joint_state_broadcaster",
        
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        # "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        # "passthrough_trajectory_controller",
    ]
    
    if start_servo_mode.perform(context) == "true":
        controllers_active.insert(0, "forward_position_controller")
        controllers_inactive.remove("forward_position_controller")
    else:
        if use_mock_hardware.perform(context) == "true":
            controllers_active.insert(0, "joint_trajectory_controller")
            controllers_inactive.remove('joint_trajectory_controller')
        else:
            controllers_active.insert(0, "scaled_joint_trajectory_controller")
            controllers_inactive.remove('scaled_joint_trajectory_controller')
    

    controller_spawners = [controller_spawner(list(controllers_active))] + [
        controller_spawner(list(controllers_inactive), active=False)
    ]

    # Delay rviz start after joint_state_broadcaster to avoid unnecessary warning output
    register_event_delay_rviz_after_JSB_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=node_joint_state_broadcaster_spawner, on_start=[node_rviz])
    )

    # robot_controllers = ["scaled_joint_trajectory_controller"]
    # robot_controller_spawners = []
    # for controller in robot_controllers:
    #     robot_controller_spawners.append(
    #         Node(
    #             package="controller_manager", executable="spawner", arguments=[controller, "-c", "/controller_manager"]
    #         )
    #     )

    # Delay loading and activation of robot_controller after 'joint_state_broadcaster'
    register_events_delay_robot_controller_spawners_after_JSB_spawner = []
    for controller in controller_spawners:
        register_events_delay_robot_controller_spawners_after_JSB_spawner.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(target_action=node_joint_state_broadcaster_spawner, on_exit=[controller])
            )
        )

    ####################################################################################################################
    ####################################################################################################################

    _to_start = [
        node_robot_state_publisher,
        node_joint_state_broadcaster_spawner,
        node_ros2_control,
        node_ur_control,
        node_dashboard_client,
        register_event_delay_rviz_after_JSB_spawner,
        node_move_group,
        node_move_arm,
        node_servo,
        warehouse_server_node,
    ] + register_events_delay_robot_controller_spawners_after_JSB_spawner

    return _to_start


def generate_launch_description():
    declared_configs = [
        dict(name="activate_joint_controller", default_value="true"),
        dict(name="headless_mode", default_value="true", description="Run the UR robot in headless mode."),
        dict(
            name="initial_ur_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Default UR robot controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "passthrough_trajectory_controller",
            ],
        ),
        dict(name="launch_dashboard_client", default_value="true"),
        dict(name="launch_rviz", default_value="true"),
        dict(name="launch_servo", default_value="true"),
        dict(name="mock_sensor_commands", default_value="false"),
        dict(name="start_servo_mode", default_value="true", description="If true, starts the forward_velocity_controller rather than the joint_trajectory_controller"),
        dict(
            name="system_description_package",
            default_value="branch_detection_system_description",
            description="Custom UR urdf package",
        ),
        dict(name="system_description_file", default_value="robot.urdf.xacro", description="urdf/xacro file"),
        dict(name="system_semantic_description_file", default_value="robot.srdf", description="srdf/xacro file"),
        dict(name="tf_prefix", default_value="ur5e__"),
        dict(name="ur_type", default_value="ur5e", description="Robot description name (required for URDF parsing)."),
        dict(name="ur_robot_ip", default_value="169.254.174.50", description="UR robot IP"),
        dict(
            name="use_mock_hardware",
            default_value="true",
            description="If true, passes `use_mock_hardware:=true` to all hardware interfaces.",
        ),
        dict(name="warehouse_sqlite_path", default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")),
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
