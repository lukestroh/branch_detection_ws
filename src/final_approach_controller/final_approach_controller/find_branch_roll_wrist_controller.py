#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time


from final_approach_controller_msgs.action import RunFindBranchRollWrist
from final_approach_controller_msgs.msg import ToFBranchFitStamped
import final_approach_controller.curve_fitting as cf
from final_approach_controller.tf_node import TFNode
from vl6180_msgs.msg import Vl6180FilteredStamped

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    MotionPlanResponse,
    RobotState,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import modern_robotics as mr
import numpy as np
import scipy.optimize as so
from scipy.spatial.transform import Rotation
import pprint as pp
from collections import deque
import time
import secrets
from threading import Event
import traceback

import pandas as pd
import os
import plotly.graph_objects as go


class FindBranchRollWristController(TFNode):
    def __init__(self):
        super().__init__(node_name="find_branch_roll_wrist_controller", cache_time=Duration(seconds=20))
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")

        # Launch arguments
        _param_use_mock_hardware: bool = (
            self.declare_parameter(name="use_mock_hardware", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        )
        if _param_use_mock_hardware:
            self._move_group_controller = "joint_trajectory_controller"
        else:
            self._move_group_controller = "scaled_joint_trajectory_controller"
        self._servo_controller = "forward_velocity_controller"

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # Action servers
        self._action_svr_run_find_branch_roll_wrist = ActionServer(
            node=self,
            action_type=RunFindBranchRollWrist,
            action_name="run_find_branch_roll_wrist",
            goal_callback=self._action_goal_cb_run_find_branch_roll_wrist,
            cancel_callback=self._action_cancel_cb_run_find_branch_roll_wrist,
            execute_callback=self._action_exe_cb_run_find_branch_roll_wrist,
            callback_group=self.callback_group,
        )

        # Action clients
        self._action_client_move_group = ActionClient(
            node=self,
            action_name="move_action",
            action_type=MoveGroup,
            callback_group=self.callback_group,
        )

        # Service clients
        self._srv_client_start_servo = self.create_client(
            srv_type=Trigger, srv_name="/servo_node/start_servo", callback_group=self.callback_group
        )

        self._srv_client_stop_servo = self.create_client(
            srv_type=Trigger, srv_name="/servo_node/stop_servo", callback_group=self.callback_group
        )

        self._srv_switch_ctrls = self.create_client(
            srv_type=SwitchController,
            srv_name="/controller_manager/switch_controller",
            callback_group=self.callback_group,
        )

        # Subscribers
        self._sub_tof_filtered = self.create_subscription(
            msg_type=Vl6180FilteredStamped,
            topic="/vl6180/filtered",
            callback=self._sub_cb_tof_filtered,
            callback_group=self.callback_group,
            qos_profile=1,
        )
        self._sub_joint_states = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self._sub_cb_joint_states,
            callback_group=self.callback_group,
            qos_profile=1,
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self.callback_group,
            qos_profile=1,
        )
        # Fit data publisher
        self._pub_fit = self.create_publisher(
            msg_type=ToFBranchFitStamped,
            topic="find_branch_roll_wrist/tof_branch_fit",
            callback_group=self.callback_group,
            qos_profile=5,
        )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=3.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_run_controller = None
        self._timer_run_quadratic_fit = None
        self._timer_debug = self.create_timer(timer_period_sec=1.0, callback=self._timer_cb_debug)

        # Messages
        self.msg_twist = TwistStamped()
        self.msg_tof_branch_fit = ToFBranchFitStamped()

        # Transforms
        self.tf_mp_base_to_tof0 = np.identity(4)
        self.tf_mp_base_to_tof1 = np.identity(4)
        self.tf_mp_cut_point_to_base = np.identity(4)
        self.tf_tof0_to_cut_point = np.identity(4)
        self.tf_tof0_to_tof1 = np.identity(4)

        # Controller attributes
        # self.generator = np.random.default_rng(seed=secrets.randbits(128))
        self.generator = np.random.default_rng(1)
        self.controller_running = False
        self.rotations_complete = False
        self.neg_rot_complete = False
        self.pos_rot_complete = False
        self.max_angular_vel = np.pi / 16
        self.vl6180_far_plane = 0.200  # 0.19 based on testing, but give it small window
        self.vl6180_precision = 0.001
        self.tof0_branch_found = self.tof1_branch_found = False
        self.get_final_pose_ready = False  # True when two valid tof fits have been recorded. Indicates to controller that it is ready to solve for a final pose

        # df = pd.read_csv(os.path.expanduser("~/branch_detection_ws/analysis/csv/tof_data.csv"))
        self.debug_plot = True
        self.d_tof0 = 0.255
        self.d_tof1 = 0.255
        self.timestamp_readings = []
        self.timestamps_tof0_filtered = []
        self.timestamps_tof1_filtered = []
        self.d_tof0_readings = []
        self.d_tof1_readings = []
        self.d_tof0_readings_filtered = []
        self.d_tof1_readings_filtered = []
        self.tof0_distance_center = None
        self.tof1_distance_center = None

        self.start_time = self.get_clock().now()
        self.start_controller_tf = np.identity(4, dtype=float)
        self.feedback_pub_prev_time = self.get_clock().now()
        self._goal_handle: ServerGoalHandle | None = None
        self.goal_handle_aborted: bool = False

        # Controller handlers
        self._action_client_move_group_done_event = Event()

        return

    # ===============================
    #        Action callbacks
    # ===============================

    def _action_cancel_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        return CancelResponse.ACCEPT

    def _action_exe_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.controller_running = True
        self._goal_handle = goal_handle

        self.start_controller_tf = self.lookup_transform(
            target_frame="cart__base",
            source_frame="mock_pruner__tool0",
            sync=True,
            as_matrix=True
        )

        if self._timer_run_controller is None:
            self._timer_run_controller = self.create_timer(
                timer_period_sec=1 / 30, callback=self._timer_cb_run_controller, callback_group=self.callback_group
            )
            # self._timer_run_quadradic_fit = self.create_timer(
            #     timer_period_sec=1.0, callback=self._timer_cb_run_quadradic_fit, callback_group=self.callback_group
            # )
        else:
            self._timer_run_controller.reset()
            # self._timer_run_quadradic_fit.reset()

        try:
            feedback_msg = RunFindBranchRollWrist.Feedback()
            result = RunFindBranchRollWrist.Result()

            while self.controller_running:
                if goal_handle.status == GoalStatus.STATUS_CANCELED:
                    self._timer_run_controller.cancel()
                    result.success = False
                    return result

                if goal_handle.status == GoalStatus.STATUS_ABORTED:
                    self._timer_run_controller.cancel()
                    result.success = False
                    return result

                if goal_handle.status == GoalStatus.STATUS_EXECUTING:
                    # Cancel action if requested
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.info("FindBranchRollWristController canceled.")
                        self.reset_controller()
                        result.success = False
                        return result

                    # Action feedback
                    if self.get_clock().now() - self.feedback_pub_prev_time >= Duration(seconds=1):
                        feedback_msg.tof0 = self.d_tof0
                        feedback_msg.tof1 = self.d_tof1
                        goal_handle.publish_feedback(feedback=feedback_msg)
                        self.feedback_pub_prev_time = self.get_clock().now()

                    ################################################################################################################
                    # if both have a fit, publish zero message, do pose math, call service, kill timer, controller_running = False
                    if not self.rotations_complete:
                        if not self.neg_rot_complete:
                            # # rotate to the closest side
                            # if self.joint_states[-1] < 0 and self.joint_states[-1] > -1 * np.pi:
                            # negative angular rotation
                            angular_z = -1 * self.max_angular_vel
                            if np.isclose(
                                self.joint_states[2], np.pi / 2, atol=0.01
                            ):  # TODO: (long term) make sure wrist mount config is standard
                                # Try to fit data
                                self.tof0_time_center, self.tof0_distance_center = (
                                    self.get_branch_center_time_and_distance(
                                        timestamps=self.timestamp_readings,
                                        readings=self.d_tof0_readings,
                                        sensor_name="tof0",
                                        debug_plot=self.debug_plot,
                                    )
                                )
                                self.tof1_time_center, self.tof1_distance_center = (
                                    self.get_branch_center_time_and_distance(
                                        timestamps=self.timestamp_readings,
                                        readings=self.d_tof1_readings,
                                        sensor_name="tof1",
                                        debug_plot=self.debug_plot,
                                    )
                                )
                                if self.tof0_time_center is not None and self.tof1_time_center is not None:
                                    self.publish_zero_twist()
                                    self.rotations_complete = True

                                self.neg_rot_complete = True

                        elif not self.pos_rot_complete:
                            # if self.joint_states[-1] > 0 and self.joint_states[-1] < np.pi:
                            # positive angular rotation
                            angular_z = self.max_angular_vel
                            if np.isclose(self.joint_states[2], np.pi + np.pi / 2, atol=0.01):
                                # Try to fit data TODO: if fit is upside down, nix it
                                self.tof0_time_center, self.tof0_distance_center = (
                                    self.get_branch_center_time_and_distance(
                                        timestamps=self.timestamp_readings,
                                        readings=self.d_tof0_readings,
                                        sensor_name="tof0",
                                        debug_plot=self.debug_plot,
                                    )
                                )

                                self.tof1_time_center, self.tof1_distance_center = (
                                    self.get_branch_center_time_and_distance(
                                        timestamps=self.timestamp_readings,
                                        readings=self.d_tof1_readings,
                                        sensor_name="tof1",
                                        debug_plot=self.debug_plot,
                                    )
                                )
                                if self.tof0_time_center is not None and self.tof1_time_center is not None:
                                    self.publish_zero_twist()
                                    self.rotations_complete = True

                                self.pos_rot_complete = True

                        self.msg_twist.twist.linear.x = 0.0
                        self.msg_twist.twist.linear.y = 0.0
                        self.msg_twist.twist.linear.z = 0.0
                        self.msg_twist.twist.angular.x = 0.0
                        self.msg_twist.twist.angular.y = 0.0
                        self.msg_twist.twist.angular.z = angular_z
                        self.msg_twist.header.frame_id = "mock_pruner__tool0"  # TODO: Get name dynamically
                        self.msg_twist.header.stamp = self.get_clock().now().to_msg()
                        self._pub_servo.publish(self.msg_twist)

                        if self.neg_rot_complete and self.pos_rot_complete:
                            self.publish_zero_twist()
                            self.rotations_complete = True

                    else:
                        if (
                            self.tof0_time_center is None or self.tof1_time_center is None
                        ):  # TODO: Check and/or logic here
                            if not self._goal_handle.status == GoalStatus.STATUS_ABORTED:
                                self.warn("Could not find the branch. Aborting FindBranchRollWristController.")
                                self._goal_handle.abort()
                                self._timer_run_controller.cancel()
                                # self._timer_run_quadratic_fit.cancel()
                                self.reset_controller()

                                result.success = False
                            return result

                        else:
                            # Stop servo
                            self._srv_client_stop_servo.call(request=Trigger.Request())

                            # Switch controllers
                            switch_ctrlr_req = SwitchController.Request(
                                activate_controllers=[self._move_group_controller],
                                deactivate_controllers=[self._servo_controller],
                                strictness=2,  # STRICT
                            )
                            self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)

                            # If the eef is moving, we need a common frame, which should be world or cart__base
                            tf_tof0_to_base__time_center_pose = self.lookup_transform(
                                target_frame="cart__base",  # TODO: probably best to dynamically get robot base, whatever it is.
                                source_frame="mock_pruner__tof0",
                                time=self.tof0_time_center,
                                sync=True,
                                as_matrix=True,
                            )
                            tf_tof1_to_base__time_center_pose = self.lookup_transform(
                                target_frame="cart__base",
                                source_frame="mock_pruner__tof1",
                                time=self.tof1_time_center,
                                sync=True,
                                as_matrix=True,
                            )

                            self.warn(f"POSE:\n{tf_tof0_to_base__time_center_pose}")
                            self.warn(f"POSE:\n{tf_tof1_to_base__time_center_pose}")

                            # Get branch locations in world coordinates
                            tof0_vec_tof0_frame = np.array([[0, 0, self.tof0_distance_center, 1]]).T
                            tof1_vec_tof1_frame = np.array([[0, 0, self.tof1_distance_center, 1]]).T

                            # self.error(tof0_vec_tof0_frame)

                            tof0_vec_base_frame = tf_tof0_to_base__time_center_pose @ tof0_vec_tof0_frame
                            tof1_vec_base_frame = tf_tof1_to_base__time_center_pose @ tof1_vec_tof1_frame
                            self.warn(f"VEC:\n{tof0_vec_base_frame}")
                            self.warn(f"VEC:\n{tof1_vec_base_frame}")
                            tof0_vec_base_frame = tof0_vec_base_frame.flatten()
                            tof1_vec_base_frame = tof1_vec_base_frame.flatten()
                            self.warn(tof0_vec_base_frame)

                            # Get the centerpoint of these two points.
                            branch_center_point = np.mean([tof0_vec_base_frame, tof1_vec_base_frame], axis=0).flatten()
                            self.warn(f"CENTER:\n{branch_center_point}")
                            # branch_center_point = branch_center_point / np.linalg.norm(branch_center_point)
                            # Get normalized vector perpendicular to this point
                            ############################################################################################
                            # world_z = [0, 0, 1]
                            # branch_norm_vec = np.cross(world_z, tof0_vec_base_frame[0:3] - tof1_vec_base_frame[0:3])
                            
                            # try:
                            #     branch_norm_vec = branch_norm_vec / np.linalg.norm(branch_norm_vec)
                            # except ZeroDivisionError:
                            #     branch_norm_vec = [0, -1, 0]

                            # self.error(branch_norm_vec)
                            # self.error(branch_center_point[0:3] - branch_norm_vec * 0.1)

                            # # Calc point 10cm from the branch in orientation direction
                            # desired_eef_xyz = branch_center_point[0:3] - (branch_norm_vec * 0.10) # TODO: check if subtraction is correct
                            ############################################################################################
                            world_z = [0,0,1]
                            tf_cut_point_to_base = self.lookup_transform(
                                target_frame="cart__base",
                                source_frame="mock_pruner__tool0",
                                sync=True,
                                as_matrix=True
                            )
                            cut_point_world_orientation_vec = tf_cut_point_to_base[0:3, 0:3] @ world_z
                            cut_point_world_orientation_quat = Rotation.from_matrix(tf_cut_point_to_base[0:3, 0:3]).as_quat()
                        

                            desired_eef_xyz = branch_center_point[0:3] - (cut_point_world_orientation_vec * 0.1)

                            if self.debug_plot:
                                fig = go.Figure()
                                fig.add_trace(
                                    go.Scatter3d(
                                        x=[0],
                                        y=[0],
                                        z=[0],
                                        name="cart__base"
                                    )
                                )
                                fig.add_trace(
                                    go.Scatter3d(
                                        x=[tof0_vec_base_frame[0]],
                                        y=[tof0_vec_base_frame[1]],
                                        z=[tof0_vec_base_frame[2]],
                                        name="tof0_reading",
                                    )
                                )
                                fig.add_trace(
                                    go.Scatter3d(
                                        x=[tof1_vec_base_frame[0]],
                                        y=[tof1_vec_base_frame[1]],
                                        z=[tof1_vec_base_frame[2]],
                                        name="tof1_reading",
                                    )
                                )
                                fig.add_trace(
                                    go.Scatter3d(
                                        x=[branch_center_point[0]],
                                        y=[branch_center_point[1]],
                                        z=[branch_center_point[2]],
                                        name="branch_center_point",
                                    )
                                )
                                fig.add_trace(
                                    go.Scatter3d(
                                        x=[desired_eef_xyz[0]],
                                        y=[desired_eef_xyz[1]],
                                        z=[desired_eef_xyz[2]],
                                        name="desired_eef_xyz",
                                    )
                                )
                                fig.update_layout(scene=dict(aspectmode='data'))
                                fig.show()

                            # If the branch points to the left, the cross product will be pointing toward the robot
                        
 
                            _motion_plan_request = MotionPlanRequest()
                            _goal_pose_constraint = Constraints()
                            _position_constraint = PositionConstraint()
                            _orientation_constraint = OrientationConstraint()

                            _position_constraint.header.frame_id = "cart__base"
                            _position_constraint.link_name = "mock_pruner__tool0"
                            _position_constraint.target_point_offset.x = desired_eef_xyz[0]
                            _position_constraint.target_point_offset.y = desired_eef_xyz[1]
                            _position_constraint.target_point_offset.z = desired_eef_xyz[2]
                            _goal_pose_constraint.position_constraints.append(_position_constraint)

                            # rot_axis = np.cross(world_z, eef_orientation_vec)
                            # rot_angle = np.arccos(np.dot(world_z, eef_orientation_vec))
                            # orientation_quat = Rotation.from_rotvec(rotvec=rot_angle * rot_axis).as_quat()
                            # self.warn(rot_axis)
                            # self.warn(rot_angle * 180 / np.pi)
                            # self.warn(f"ORIENTATION: {orientation_quat}")
                            _orientation_constraint.header.frame_id = "cart__base"
                            _orientation_constraint.link_name = "mock_pruner__tool0"
                            _orientation_constraint.orientation.x = cut_point_world_orientation_quat[0]
                            _orientation_constraint.orientation.x = cut_point_world_orientation_quat[1]
                            _orientation_constraint.orientation.x = cut_point_world_orientation_quat[2]
                            _orientation_constraint.orientation.x = cut_point_world_orientation_quat[3]

                            # self.arm_prefix = "ur__"  # TODO: Get prefix params from launch
                            # self.robot_name = "pruning_robot"  # TODO: fix SRDF name structure as well
                            # kwargs = {"position_constraints": _position_constraint, "orientation_constraints": _orientation_constraint}
                            # _motion_plan_request = MotionPlanRequest(
                            #     group_name = f"{self.arm_prefix}{self.robot_name}_manipulator",
                            #     goal_constraints=[Constraints(**kwargs)],
                            #     allowed_planning_time=5.0
                            # )
                            # _move_group_goal = MoveGroup.Goal()
                            # _move_group_goal.request = _motion_plan_request

                            
                            #########################################################################
                            _goal_pose_constraint.orientation_constraints.append(_orientation_constraint)

                            _motion_plan_request.workspace_parameters.header.frame_id = "cart__base"
                            _motion_plan_request.goal_constraints.append(_goal_pose_constraint)
                            _motion_plan_request.allowed_planning_time = 5.0
                            _motion_plan_request.num_planning_attempts = 10
                            # _motion_plan_request.start_state = RobotState(joint_state=JointState(position=self.joint_states))
                            self.arm_prefix = "ur5e__"  # TODO: Get prefix params from launch
                            self.robot_name = "pruning_robot"  # TODO: fix SRDF name structure as well
                            _motion_plan_request.group_name = f"{self.arm_prefix}{self.robot_name}_manipulator"
                            # _motion_plan_request.planner_id = "RRTConnect" # linear
                            # _motion_plan_request.pipeline_id = "ompl"

                            _move_group_goal = MoveGroup.Goal()
                            _move_group_goal.request = _motion_plan_request
                            _move_group_goal.planning_options = PlanningOptions(plan_only=False)

                            future = self._action_client_move_group.send_goal_async(goal=_move_group_goal)
                            ####################################################################################
                            # future.add_done_callback(self._action_client_move_group_done_cb)
                            # get angle between the two to determine direction
                            # np.dot()
                            # self.warn(dir_vec)
                            # TODO: need initial edge case where a ToF is reading at the start of the controller so that we save it's position and don't need to fit (also avoid poor parabola fit)

                            # self.warn(branch_center_point)

                            self._goal_handle.succeed()
                            result.success = True
                            self.controller_running = False
                            return result
            ############################################################################################################

        except Exception as e:
            self.get_logger().fatal(f"{traceback.format_exc()}")
            result.success = False
            self._goal_handle.abort()
        finally:
            self._timer_run_controller.cancel()
            self.publish_zero_twist()
            # self._timer_run_quadradic_fit.cancel()
            # self.controller_running = False # TODO: reset controller?
            self.reset_controller()
            self.info("FindBranchRollWristController has terminated.")
        return result

    def _action_goal_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        # self.goal_handle_aborted = False
        return GoalResponse.ACCEPT
    
    def _action_client_move_group_done_cb(self):

        return

    # ===============================
    #         Timer callbacks
    # ===============================
    def _timer_cb_setup_tf_frames(self):
        frame_sets = [
            {"parent": "mock_pruner__base", "child": "mock_pruner__tof0"},
            {"parent": "mock_pruner__base", "child": "mock_pruner__tof1"},
            {"parent": "mock_pruner__base", "child": "mock_pruner__tool0"},
        ]

        transforms = []
        # Wait until all transforms are loaded
        for tf_name in frame_sets:
            while True:
                self.get_logger().info(f"Waiting for tf...", throttle_duration_sec=1.0)
                tf = self.lookup_transform(
                    target_frame=tf_name["parent"],  # we need to bring tof data into parent frame.
                    source_frame=tf_name["child"],
                    time=Time(),
                    sync=True,
                    timeout=Duration(seconds=1),
                    as_matrix=True,
                )
                if tf is not None:
                    transforms.append(tf)
                    break

        # Kill timer
        self._timer_setup_tf_frames.destroy()
        # Solve for additional transforms
        self.tf_mp_tof0_to_base, self.tf_mp_tof1_to_base, self.tf_mp_cut_point_to_base = transforms
        self.tf_cut_point_to_tof0 = mr.TransInv(self.tf_mp_cut_point_to_base) @ self.tf_mp_tof0_to_base
        self.tf_tof0_to_tof1 = mr.TransInv(self.tf_mp_tof1_to_base) @ self.tf_mp_tof0_to_base
        tof0_to_tof1_pos_vec = self.tf_tof0_to_tof1[:3, 3]
        self._tof_linear_distance = np.linalg.norm(tof0_to_tof1_pos_vec)
        if not np.all(np.isclose(self.tf_tof0_to_tof1[:3, :3], np.identity(3), atol=1e-3)):
            raise ValueError("The two ToF frames are not aligned with each other.")
        return

    def _timer_cb_run_controller(self):
        return

    def _timer_cb_run_quadradic_fit(self):
        return

    def _timer_cb_debug(self):
        return

    # ===============================
    #     Subscription callbacks
    # ===============================

    def _sub_cb_tof_filtered(self, msg: Vl6180FilteredStamped):
        # Do some checks, make sure that the readings make sense in intuitive way.
        # Make sure readings do not exceed maximum. # TODO: Find a way to get sensor parameters in here
        self.d_tof0 = msg.data[0] / 1000  # mm to m TODO: move this to vl6180 package
        self.d_tof1 = msg.data[1] / 1000

        if self.controller_running:
            if not self.tof0_branch_found:  # TODO: These flags are doing nothing currently.
                self.d_tof0_readings.append(self.d_tof0)
            if not self.tof1_branch_found:
                self.d_tof1_readings.append(self.d_tof1)
            if not self.tof0_branch_found or not self.tof1_branch_found:
                self.timestamp_readings.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        return

    def _sub_cb_joint_states(self, msg: JointState):
        self.joint_states = msg.position
        # self.warn(joint_states)
        return

    def reset_controller(self) -> None:
        self.controller_running = False
        self.rotations_complete = False
        self.neg_rot_complete = False
        self.pos_rot_complete = False
        self.tof0_time_center = None
        self.tof0_distance_center = None
        self.tof1_time_center = None
        self.tof1_distance_center = None
        self.d_tof0_readings = []
        self.d_tof1_readings = []
        self.d_tof0_readings_filtered = []
        self.d_tof1_readings_filtered = []
        self.timestamp_readings = []
        self.timestamps_tof0_filtered = []
        self.timestamps_tof1_filtered = []
        self.start_controller_tf = np.identity(4)
        self._action_client_move_group_done_event.clear()
        return

    def publish_zero_twist(self):
        self.msg_twist.twist.linear.x = 0.0
        self.msg_twist.twist.linear.y = 0.0
        self.msg_twist.twist.linear.z = 0.0
        self.msg_twist.twist.angular.x = 0.0
        self.msg_twist.twist.angular.y = 0.0
        self.msg_twist.twist.angular.z = 0.0
        self.msg_twist.header.frame_id = "mock_pruner__tool0"  # TODO: if changing to EEF, change ur_servo.yaml
        self.msg_twist.header.stamp = self.get_clock().now().to_msg()
        self._pub_servo.publish(self.msg_twist)
        return

    def get_branch_center_time_and_distance(
        self, timestamps, readings, sensor_name: str, debug_plot: bool = False
    ) -> Time | None:
        try:
            readings_filtered = np.where(np.asarray(readings) < self.vl6180_far_plane, readings, np.nan)
            timestamps_filtered = np.where(np.isnan(readings_filtered), np.nan, timestamps)
            readings_filtered = readings_filtered[~np.isnan(readings_filtered)]
            timestamps_filtered = timestamps_filtered[~np.isnan(timestamps_filtered)]
            normalized_timestamps_filtered = timestamps_filtered - timestamps_filtered[0]
        except (ValueError, IndexError) as e:
            self.info(f"{e}: Did not find branch, aborting fit for this rotation.")
            # if debug_plot:
            #     fig = go.Figure()
            #     fig.add_trace(
            #         go.Scatter(
            #             x=normalized_timestamps_filtered,
            #             y=readings_filtered,
            #             name="filtered_data",
            #         )
            #     )
            #     fig.add_trace(go.Scatter(x=t_fit, y=fit_data, name="fit_data"))
            #     fig.add_trace(go.Scatter(x=np.asarray(timestamps) - timestamps[0], y=readings, name="raw_data"))
            #     fig.update_layout(title=dict(text=sensor_name))
            #     fig.show()
            return None
        
        

        fit_params, fit_covs = so.curve_fit(
            f=cf.parabola,
            xdata=normalized_timestamps_filtered,
            ydata=readings_filtered,
            nan_policy="omit",
            maxfev=5000,
            sigma=self.vl6180_precision,
            absolute_sigma=True,
        )
        # If the parabola is negative, reject fit.
        if fit_params[0] < 0:
            return None

        t_fit = np.linspace(
            min(normalized_timestamps_filtered),
            max(normalized_timestamps_filtered),
            len(normalized_timestamps_filtered),
        )
        fit_data = cf.parabola(t_fit, *fit_params)
        idx_min = np.argmin(fit_data)
        timestamp_min = timestamps_filtered[idx_min]
        fit_min = float(fit_data[idx_min])
        split_time = np.modf(timestamp_min)
        time_center = Time(seconds=int(split_time[1]), nanoseconds=split_time[0] * 1e9)

        if debug_plot:
            fig = go.Figure()
            fig.add_trace(
                go.Scatter(
                    x=normalized_timestamps_filtered,
                    y=readings_filtered,
                    name="filtered_data",
                )
            )
            fig.add_trace(go.Scatter(x=t_fit, y=fit_data, name="fit_data"))
            fig.add_trace(go.Scatter(x=np.asarray(timestamps) - timestamps[0], y=readings, name="raw_data"))
            fig.update_layout(title=dict(text=sensor_name))
            fig.show()

        # TODO: reinstate publisher
        # self.msg_tof_branch_fit.timestamps = list(timestamps_filtered)
        # self.msg_tof_branch_fit.tof_data = list(readings_filtered)
        # self.msg_tof_branch_fit.fit_data = list(fit_data)
        # self.msg_tof_branch_fit.sensor_name = sensor_name
        # self.msg_tof_branch_fit.header.stamp = self.get_clock().now().to_msg()

        # self._pub_fit.publish(self.msg_tof_branch_fit)

        return time_center, fit_min

    def send_move_group_goal(self):

        return


def main():
    rclpy.init()
    find_branch_roll_wrist_controller = FindBranchRollWristController()
    executor = MultiThreadedExecutor()
    rclpy.spin(find_branch_roll_wrist_controller, executor=executor)
    find_branch_roll_wrist_controller.destroy_node()
    rclpy.shutdown()
    return
