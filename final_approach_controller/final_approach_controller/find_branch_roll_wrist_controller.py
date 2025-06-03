#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time
from rclpy.qos import QoSProfile

from action_msgs.msg import GoalStatus
from branch_detection_system_moveit_msgs.srv import MoveToPose
from controller_manager_msgs.srv import SwitchController
from final_approach_controller_msgs.action import RunFindBranchRollWrist
from final_approach_controller_msgs.msg import ToFBranchFitStamped, WindowedData, TimestampTofMin
from geometry_msgs.msg import TwistStamped, Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float64
from std_srvs.srv import Trigger
from tof_msgs.msg import TofStamped
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from vl53l4cd_msgs.msg import Vl53l4cdStamped

import branch_detection_system_analysis.plot.debug_plots as dplot
import final_approach_controller.curve_fitting as cf
from final_approach_controller.tf_node import TFNode
import modern_robotics as mr
import numpy as np
import os
import plotly.graph_objects as go
import plotly.io as pio
import scipy.optimize as so
from scipy.spatial.transform import Rotation
from threading import Event, Lock
import traceback

import pandas as pd
import py_trees


class FindBranchRollWristController(TFNode):
    def __init__(self):
        super().__init__(node_name="find_branch_roll_wrist_controller", cache_time=Duration(seconds=30))

        # Parameters
        _param_use_mock_hardware: bool = (
            self.declare_parameter(name="use_mock_hardware", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        )
        if _param_use_mock_hardware:
            self._move_group_controller = "joint_trajectory_controller"
        else:
            self._move_group_controller = "scaled_joint_trajectory_controller"
        self._servo_controller = "forward_position_controller"

        self._param_robot_base_part: str = (
            self.declare_parameter(name="robot_base_part", value=Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )

        self._param_robot_eef_part: str = (
            self.declare_parameter(name="robot_eef_part", value=Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )

        # Threading locks
        self._data_lock = Lock()
        self._timer_lock = Lock()
        self._branch_found_lock = Lock()
        self._servo_msg_lock = Lock()

        # Callback group
        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._parabola_fitting_cb_group = MutuallyExclusiveCallbackGroup()
        self._pub_servo_cb_group = MutuallyExclusiveCallbackGroup()

        # Action servers
        self._action_svr_run_find_branch_roll_wrist = ActionServer(
            node=self,
            action_type=RunFindBranchRollWrist,
            action_name="run_find_branch_roll_wrist",
            goal_callback=self._action_goal_cb_run_find_branch_roll_wrist,
            cancel_callback=self._action_cancel_cb_run_find_branch_roll_wrist,
            execute_callback=self._action_exe_cb_run_find_branch_roll_wrist,
            callback_group=self._reentrant_cb_group,
        )

        # Action clients
        self._action_client_move_group = ActionClient(
            node=self,
            action_name="move_action",
            action_type=MoveGroup,
            callback_group=self._reentrant_cb_group,
        )

        # Service clients
        self._srv_cartesian_move_to_pose = self.create_client(
            srv_type=MoveToPose, srv_name="/cartesian_move_to_pose", callback_group=self._reentrant_cb_group
        )

        while not self._srv_cartesian_move_to_pose.wait_for_service(timeout_sec=1.0):
            self.warn("Waiting for Cartesian move to pose service...")

        self._srv_client_start_servo = self.create_client(
            srv_type=Trigger, srv_name="/servo_node/start_servo", callback_group=self._reentrant_cb_group
        )
        self._srv_client_start_servo.wait_for_service()
        self._srv_client_stop_servo = self.create_client(
            srv_type=Trigger, srv_name="/servo_node/stop_servo", callback_group=self._reentrant_cb_group
        )
        self._srv_client_stop_servo.wait_for_service()

        self._srv_switch_ctrls = self.create_client(
            srv_type=SwitchController,
            srv_name="/controller_manager/switch_controller",
            callback_group=self._reentrant_cb_group,
        )

        # Subscribers
        self._sub_tof_raw = self.create_subscription(
            msg_type=Vl53l4cdStamped,
            topic="/microROS/vl53l4cd/data",
            callback=self._sub_cb_tof_raw,
            callback_group=self._reentrant_cb_group,
            qos_profile=5,
        )
        self._sub_tof_filtered = self.create_subscription(
            msg_type=TofStamped,
            topic="/vl53l4cd/filtered",
            callback=self._sub_cb_tof_filtered,
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )
        self._sub_bag_record_path = self.create_subscription(
            msg_type=String,
            topic="/bag_record_path",
            callback=self._sub_cb_bag_record_path,
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )
        self._sub_joint_states = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self._sub_cb_joint_states,
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self._reentrant_cb_group,
            qos_profile=QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=10
            ),
        )
        self._pub_alignment_success = self.create_publisher(
            msg_type=Bool,
            topic="/fbrw_controller/alignment_success",
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )
        self._pub_localization_success = self.create_publisher(
            msg_type=Bool,
            topic="/fbrw_controller/localization_success",
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )
        self._pub_rotation_speed = self.create_publisher(
            msg_type=Float64,
            topic="/fbwr_controller/rotation_speed",
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )
        self._pub_windowed_data = self.create_publisher(
            msg_type=WindowedData,
            topic="/fbrw_controller/windowed_data",
            callback_group=self._reentrant_cb_group,
            qos_profile=50,
        )
        self._pub_ts_tof_min = self.create_publisher(
            msg_type=TimestampTofMin,
            topic="/fbrw_controller/ts_tof_min",
            callback_group=self._reentrant_cb_group,
            qos_profile=10,
        )

        # self._pub_fit = self.create_publisher(
        #     msg_type=ToFBranchFitStamped,
        #     topic="find_branch_roll_wrist/tof_branch_fit",
        #     callback_group=self._reentrant_cb_group,
        #     qos_profile=5,
        # )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=3.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_pub_servo = None

        # Messages
        self._msg_twist = TwistStamped()
        self._msg_twist.twist.linear.x = 0.0
        self._msg_twist.twist.linear.y = 0.0
        self._msg_twist.twist.linear.z = 0.0
        self._msg_twist.twist.angular.x = 0.0
        self._msg_twist.twist.angular.y = 0.0
        self._msg_twist.header.frame_id = f"{self._param_robot_eef_part}__tool0"
        # self.msg_tof_branch_fit = ToFBranchFitStamped()
        self._msg_ts_tof_min = TimestampTofMin()

        # Action requests
        self.move_to_pose_req = MoveToPose.Request()

        # Transforms
        self.tf_mp_base_to_tof0 = np.identity(4)
        self.tf_mp_base_to_tof1 = np.identity(4)
        self.tf_mp_cut_point_to_base = np.identity(4)
        self.tf_tof0_to_cut_point = np.identity(4)
        self.tf_tof0_to_tof1 = np.identity(4)

        # Controller attributes
        self.reset_controller()
        self.feedback_pub_prev_time = self.get_clock().now()
        self.start_states_recorded = False
        if _param_use_mock_hardware:
            self.max_angular_vel = np.pi / 16
        else:
            self.max_angular_vel = np.pi / 16 * 10  # For some reason the UR5e scales down servoing movement very hard?

        self.filter_far_plane = 0.25

        # self.eef_weight = 0.355  # TODO: measure again. Measured IRL

        # Sensor attributes
        self.tof_far_plane = 0.200  # 0.19 based on testing, but give it small window. TODO: Get from param file
        self.tof_precision = 0.001
        self.d_tof0 = 0.0  # 0.255
        self.d_tof1 = 0.0  # 0.255
        self.d_tof0_raw = 0.0  # 0.255
        self.d_tof1_raw = 0.0  # 0.255
        self.start_joint_states = None

        # Debug parameters
        self.bag_record_path: str = ""
        self.debug_plot = True
        return

    # ===============================
    #        Action callbacks
    # ===============================
    def _action_cancel_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        self.info("Canceling quadratic fit timer")
        self.publish_zero_twist()
        with self._timer_lock:
            if not self._timer_pub_servo.is_canceled():
                self._timer_pub_servo.cancel()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        self.reset_controller()
        return CancelResponse.ACCEPT

    def _action_goal_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT

    async def _action_exe_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        # Let the tofs settle
        self.get_clock().sleep_for(Duration(seconds=1.0))

        # feedback_msg = RunFindBranchRollWrist.Feedback()
        _result = RunFindBranchRollWrist.Result()
        self.controller_running = True

        if not self.start_states_recorded:
            self.start_controller_tf = self.lookup_transform(
                target_frame=f"{self._param_robot_base_part}__base",
                source_frame=f"{self._param_robot_eef_part}__tool0",
                sync=True,
                as_matrix=True,
            )
            self.start_joint_states = self.joint_states

        await self.start_servo()
        self._pub_rotation_speed.publish(Float64(data=self.max_angular_vel))

        with self._timer_lock:
            if self._timer_pub_servo is None:
                self._timer_pub_servo = self.create_timer(
                    timer_period_sec=1 / 250,
                    callback=self._timer_cb_pub_servo,
                    callback_group=self._reentrant_cb_group,
                )
            else:
                self._timer_pub_servo.reset()

        try:
            while self.controller_running:
                self.get_clock().sleep_for(Duration(seconds=0.005))  # loop runs too fast, slow it down!

                if not self.check_action_goal_status(goal_handle=goal_handle):
                    _result.success = False
                    return _result

                # if both have a fit, publish zero message, do pose math, call service, kill timer, controller_running = False
                tof0_branch_found, tof1_branch_found = self.check_if_branch_found()

                if not self.rotations_complete:
                    self.actuate_wrist()
                else:
                    if self.tof0_time_center is None or self.tof1_time_center is None:  # TODO: Check and/or logic here
                        if not goal_handle.status == GoalStatus.STATUS_ABORTED:
                            self.warn("Could not find the branch. Aborting FindBranchRollWristController.")
                            goal_handle.abort()
                            # self.reset_controller() # Done in 'finally'
                            self._pub_localization_success.publish(msg=Bool(data=False))
                            self._pub_alignment_success.publish(msg=Bool(data=False))
                            _result.success = False
                        return _result

                    else:
                        self._pub_localization_success.publish(msg=Bool(data=True))
                        if rclpy.ok():
                            await self.stop_servo()

                            await self.switch_controllers(
                                activate_controllers=self._move_group_controller,
                                deactivate_controllers=self._servo_controller,
                            )
                        else:
                            raise Exception("rclpy is not ok :(")

                        # Get branch info, find desired xyz + quat
                        branch_center_point, branch_vec_normalized, tof0_vec_base_frame, tof1_vec_base_frame = (
                            self.get_branch_vec_from_tof(return_frames=self.debug_plot)
                        )

                        desired_eef_xyz = self.get_desired_position_from_branch_vec(
                            branch_center_point=branch_center_point, branch_vec=branch_vec_normalized
                        )

                        desired_orientation_quat, desired_orientation_vec = (
                            self.get_desired_orientation_from_branch_vec(
                                branch_center_point=branch_center_point,
                                branch_vec=branch_vec_normalized,
                                desired_eef_xyz=desired_eef_xyz,
                            )
                        )

                        ######################################################################################
                        if self.debug_plot:
                            fig = dplot.plot_branch_projection(
                                tof0=tof0_vec_base_frame,
                                tof1=tof1_vec_base_frame,
                                branch_center_pos=branch_center_point,
                                branch_vec_ori=branch_vec_normalized,
                                base_origin_name=self._param_robot_base_part,
                                desired_eef_pos=desired_eef_xyz,
                                desired_eef_ori=desired_orientation_vec,
                                save_fig=True,
                                save_fig_dir=self.bag_record_path,
                            )

                        #####################################################
                        self.info(f"Moving to pose {desired_eef_xyz}, {desired_orientation_vec}")
                        self.move_to_pose_req.goal.position.x = desired_eef_xyz[0]
                        self.move_to_pose_req.goal.position.y = desired_eef_xyz[1]
                        self.move_to_pose_req.goal.position.z = desired_eef_xyz[2]
                        self.move_to_pose_req.goal.orientation.x = desired_orientation_quat[0]
                        self.move_to_pose_req.goal.orientation.y = desired_orientation_quat[1]
                        self.move_to_pose_req.goal.orientation.z = desired_orientation_quat[2]
                        self.move_to_pose_req.goal.orientation.w = desired_orientation_quat[3]

                        self.info("Sending goal")

                        move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(
                            request=self.move_to_pose_req
                        )
                        move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
                        await move_group_future

                        # Wait a second for moving average filter to settle.
                        self.get_clock().sleep_for(Duration(seconds=1.0))

                        if move_group_future.result() is None or not move_group_future.result().result:
                            goal_handle.abort()
                            _result.success = False
                        else:
                            if self.d_tof0 < self.tof_far_plane and self.d_tof1 < self.tof_far_plane:
                                goal_handle.succeed()
                                _result.success = True
                            else:
                                goal_handle.abort()
                                _result.success = False
                                self.error(
                                    f"Failed to navigate to pose where both sensors can read the branch.\nd_tof0: {self.d_tof0}, d_tof1: {self.d_tof1}, far_plane: {self.tof_far_plane}"
                                )

                        self.controller_running = False

                        await self.switch_controllers(
                            activate_controllers=self._servo_controller,
                            deactivate_controllers=self._move_group_controller,
                        )

                        if _result.success:
                            self._pub_alignment_success.publish(msg=Bool(data=True))
                        else:
                            self._pub_alignment_success.publish(msg=Bool(data=False))

                        return _result
            ############################################################################################################

        except Exception as e:
            self.get_logger().fatal(f"{traceback.format_exc()}")
            _result.success = False
            goal_handle.abort()
        finally:
            if rclpy.ok():
                try:
                    self.publish_zero_twist()
                    await self.stop_servo()
                    self.reset_controller()
                    self.info("FindBranchRollWristController has terminated.")

                except Exception as e:
                    self.fatal(traceback.format_exc())
                # await self.switch_controllers(activate_controllers=self._servo_controller, deactivate_controllers=self._move_group_controller)

                self.get_clock().sleep_for(Duration(seconds=2.0))

        return _result

    # ===============================
    #        Future callbacks
    # ===============================
    def _done_cb_srv_cartesian_move_to_pose(self, future: Future):
        # goal_handle: MoveToPose.Response = future.result()
        # self.
        self.info("Move plan/execute finished.")
        # self.info(f"Cartesian move to pose result: {goal_handle.result}.")
        return

    # ===============================
    #         Timer callbacks
    # ===============================
    def _timer_cb_setup_tf_frames(self):
        frame_sets = [
            {"parent": f"{self._param_robot_eef_part}__base", "child": f"{self._param_robot_eef_part}__tof0"},
            {"parent": f"{self._param_robot_eef_part}__base", "child": f"{self._param_robot_eef_part}__tof1"},
            {"parent": f"{self._param_robot_eef_part}__base", "child": f"{self._param_robot_eef_part}__tool0"},
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
        self.info("Static TF frames acquired.")
        return

    def _timer_cb_pub_servo(self):
        with self._timer_lock:
            if self._timer_pub_servo.is_canceled():
                return
        with self._servo_msg_lock:
            self._pub_servo.publish(self._msg_twist)
        return

    # ===============================
    #     Subscription callbacks
    # ===============================
    def _sub_cb_tof_raw(self, msg: Vl53l4cdStamped):
        if msg.dev_id == 0:
            self.d_tof0_raw = msg.distance / 1000
        elif msg.dev_id == 1:
            self.d_tof1_raw = msg.distance / 1000

        if self.controller_running:
            timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._data_lock:
                if msg.dev_id == 0:
                    if not self.tof0_branch_found:
                        self.d_tof0_raw_readings.append(self.d_tof0_raw)
                        self.ts_tof0_raw.append(timestamp_float)
                elif msg.dev_id == 1:
                    if not self.tof1_branch_found:
                        self.d_tof1_raw_readings.append(self.d_tof1_raw)
                        self.ts_tof1_raw.append(timestamp_float)
        return

    def _sub_cb_tof_filtered(self, msg: TofStamped):
        # Do some checks, make sure that the readings make sense in intuitive way.
        # Make sure readings do not exceed maximum. # TODO: Find a way to get sensor parameters in here
        if msg.dev_id == 0:
            self.d_tof0 = msg.data[0]
        elif msg.dev_id == 1:
            self.d_tof1 = msg.data[0]

        # Append data to lists if the controller is running
        if self.controller_running:
            timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._data_lock:
                if msg.dev_id == 0:
                    if not self.tof0_branch_found:
                        self.d_tof0_readings.append(self.d_tof0)
                        self.ts_tof0.append(timestamp_float)
                elif msg.dev_id == 1:
                    if not self.tof1_branch_found:
                        self.d_tof1_readings.append(self.d_tof1)
                        self.ts_tof1.append(timestamp_float)

        return

    def _sub_cb_joint_states(self, msg: JointState):
        self.joint_states = msg.position

        if self.controller_running:
            timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._data_lock:
                self.joint_states_cache.append(msg.position)
                self.ts_joint_states_cache.append(timestamp_float)
        return

    def _sub_cb_bag_record_path(self, msg: String):
        self.warn(f"{msg.data}")
        self.bag_record_path = msg.data
        return

    # ===============================
    #         Class methods
    # ===============================
    def reset_controller(self) -> None:
        self.controller_running = False
        self.rotations_complete = False
        self.neg_rot_complete = False
        self.pos_rot_complete = False
        with self._branch_found_lock:
            self.tof0_branch_found = False
            self.tof1_branch_found = False
        self.tof0_time_center = None
        self.tof1_time_center = None
        self.tof0_distance_center = None
        self.tof1_distance_center = None
        # self.start_controller_tf = np.identity(4)  # TODO: unused.

        self.reset_data_caches()

        self.info("Controller parameters have been reset")
        return

    def reset_data_caches(self):
        with self._data_lock:
            self.d_tof0_raw_readings = []
            self.d_tof0_readings = []
            self.ts_tof0 = []
            self.ts_tof0_raw = []
            self.d_tof1_raw_readings = []
            self.d_tof1_readings = []
            self.ts_tof1 = []
            self.ts_tof1_raw = []
            self.joint_states_cache = []
            self.ts_joint_states_cache = []
        return

    def check_action_goal_status(self, goal_handle: ServerGoalHandle) -> bool:
        if goal_handle.status == GoalStatus.STATUS_CANCELED:
            with self._timer_lock:
                if not self._timer_pub_servo.is_canceled():
                    self.publish_zero_twist()
                    self._timer_pub_servo.cancel()
            self.warn("FindBranchRollWristController canceled.")
            self._pub_localization_success.publish(msg=Bool(data=False))
            self._pub_alignment_success.publish(msg=Bool(data=False))
            self.reset_controller()
            return False

        if goal_handle.status == GoalStatus.STATUS_ABORTED:
            with self._timer_lock:
                if not self._timer_pub_servo.is_canceled():
                    self.publish_zero_twist()
                    self._timer_pub_servo.cancel()
            self.error("FindBranchRollWristController aborted.")
            self._pub_localization_success.publish(msg=Bool(data=False))
            self._pub_alignment_success.publish(msg=Bool(data=False))
            self.reset_controller()
            return False

        if goal_handle.status == GoalStatus.STATUS_EXECUTING:
            return True

    def check_if_branch_found(self) -> tuple:
        with self._branch_found_lock:
            tof0_branch_found = self.tof0_branch_found
            tof1_branch_found = self.tof1_branch_found

        if tof0_branch_found and tof1_branch_found:
            self.info("Branch found!")
            self.info(f"tof0: {tof0_branch_found}, tof1: {tof1_branch_found}")

            self.neg_rot_complete = True
            self.pos_rot_complete = True
            self.rotations_complete = True

            self.publish_zero_twist()
            self.info("Branch readings found for both ToFs!")
        return (tof0_branch_found, tof1_branch_found)

    def actuate_wrist(self):
        ##############################################################################################
        # Make this a behavior?
        if not self.neg_rot_complete:
            angular_z = -1 * self.max_angular_vel
            if np.isclose(self.start_joint_states[2] - self.joint_states[2], np.pi / 2, atol=0.05):
                # TODO: (long term) make sure wrist mount config is standard
                self.publish_zero_twist()
                self.neg_rot_complete = True
                self.run_quadratic_fit()
                ##############################################################################################
                self.reset_data_caches()
        elif not self.pos_rot_complete:
            # if self.joint_states[-1] > 0 and self.joint_states[-1] < np.pi:
            # positive angular rotation
            angular_z = self.max_angular_vel
            if np.isclose(self.start_joint_states[2] - self.joint_states[2], -np.pi / 2, atol=0.05):
                self.publish_zero_twist()
                self.pos_rot_complete = True
                self.run_quadratic_fit()
        with self._servo_msg_lock:
            self._msg_twist.twist.angular.z = angular_z
            self._msg_twist.header.stamp = self.get_clock().now().to_msg()
        if self.neg_rot_complete and self.pos_rot_complete:
            self.publish_zero_twist()
            self.rotations_complete = True
            with self._timer_lock:
                if not self._timer_pub_servo.is_canceled():
                    self._timer_pub_servo.cancel()
        return

    def run_quadratic_fit(self):
        with self._branch_found_lock:
            tof0_branch_found = self.tof0_branch_found

        with self._data_lock:
            joint_states_copy = np.array(self.joint_states_cache)
            joint_states_ts_copy = np.array(self.ts_joint_states_cache)

        if not tof0_branch_found:
            # Make a copy of the recorded data so that the subscriber may continue to append
            with self._data_lock:
                ts_tof0_raw_copy = list(self.ts_tof0_raw)
                ts_tof0_copy = list(self.ts_tof0)
                d_tof0_readings_raw_copy = list(self.d_tof0_raw_readings)
                d_tof0_readings_copy = list(self.d_tof0_readings)

            df_dict = {
                "tof0_raw": pd.DataFrame(
                    data=np.array([ts_tof0_raw_copy, d_tof0_readings_raw_copy]).T,
                    columns=["tof0_raw_ts", "tof0_raw_data"],
                ),
                "tof0_filtered": pd.DataFrame(
                    data=np.array([ts_tof0_copy, d_tof0_readings_copy]).T,
                    columns=["tof0_filtered_ts", "tof0_filtered_data"],
                ),
                "joint_states": pd.DataFrame(
                    data=np.array([joint_states_ts_copy, joint_states_copy[:, 2]]).T,
                    columns=["joint_states_ts", "joint_states_data"],
                ),
            }
            tof0_time_and_dist = cf.get_branch_center_time_and_distance(
                df_dict=df_dict,
                filter_far_plane=self.filter_far_plane,
                sensor_name="tof0",
                debug_plot=True,
                save_fig=True,
                save_fig_path=self.bag_record_path,
                window_size=2.0,
                window_overlap_ratio=9 / 10,
                min_samples=10,
                max_trials=20,
                residual_threshold=0.008,
                node=self,
            )
            self.warn(f"TOF0: {tof0_time_and_dist}")
            if tof0_time_and_dist is not None:
                self.tof0_time_center, self.tof0_distance_center = tof0_time_and_dist
                self._msg_ts_tof_min.sensor_id = 0
                self._msg_ts_tof_min.timestamp = self.tof0_time_center
                self._msg_ts_tof_min.data = self.tof0_distance_center
                self._pub_ts_tof_min.publish(msg=self._msg_ts_tof_min)
                with self._branch_found_lock:
                    self.tof0_branch_found = True
        else:
            self.info("Branch already detected by tof0. Skipping.")

        with self._branch_found_lock:
            tof1_branch_found = self.tof1_branch_found
        if not tof1_branch_found:
            with self._data_lock:
                ts_tof1_raw_copy = list(self.ts_tof1_raw)
                ts_tof1_copy = list(self.ts_tof1)
                d_tof1_readings_raw_copy = list(self.d_tof1_raw_readings)
                d_tof1_readings_copy = list(self.d_tof1_readings)

            df_dict = {
                "tof1_raw": pd.DataFrame(
                    data=np.array([ts_tof1_raw_copy, d_tof1_readings_raw_copy]).T,
                    columns=["tof1_raw_ts", "tof1_raw_data"],
                ),
                "tof1_filtered": pd.DataFrame(
                    data=np.array([ts_tof1_copy, d_tof1_readings_copy]).T,
                    columns=["tof1_filtered_ts", "tof1_filtered_data"],
                ),
                "joint_states": pd.DataFrame(
                    data=np.array([joint_states_ts_copy, joint_states_copy[:, 2]]).T,
                    columns=["joint_states_ts", "joint_states_data"],
                ),
            }
            tof1_time_and_dist = cf.get_branch_center_time_and_distance(
                df_dict=df_dict,
                filter_far_plane=self.filter_far_plane,
                sensor_name="tof1",
                debug_plot=True,
                save_fig=True,
                save_fig_path=self.bag_record_path,
                window_size=2.0,
                window_overlap_ratio=9 / 10,
                min_samples=10,
                max_trials=20,
                residual_threshold=0.008,
                node=self,
            )
            self.warn(f"TOF1: {tof1_time_and_dist}")
            if tof1_time_and_dist is not None:
                self.tof1_time_center, self.tof1_distance_center = tof1_time_and_dist
                self._msg_ts_tof_min.sensor_id = 1
                self._msg_ts_tof_min.timestamp = self.tof1_time_center
                self._msg_ts_tof_min.data = self.tof1_distance_center
                self._pub_ts_tof_min.publish(msg=self._msg_ts_tof_min)
                with self._branch_found_lock:
                    self.tof1_branch_found = True
        else:
            self.info("Branch already detected by tof1. Skipping.")
        return

    def get_tof_vec_base_frame(
        self,
        ts: float,
        tof: float,
        sensor_name: str,
    ):
        # If the eef is moving, we need a common frame, which should be world or <robot-part>__base
        # Get tof poses at calculated signal minimum times
        tf_tof_to_base__time_center_pose = self.lookup_transform(
            target_frame=f"{self._param_robot_base_part}__base",  # TODO: probably best to dynamically get robot base, whatever it is.
            source_frame=f"{self._param_robot_eef_part}__{sensor_name}",
            time=Time(seconds=ts),
            sync=True,
            as_matrix=True,
        )

        # Get branch locations in world coordinates
        tof_vec_tof_frame = np.array([[0, 0, tof, 1]]).T

        tof_vec_base_frame = tf_tof_to_base__time_center_pose @ tof_vec_tof_frame
        tof_vec_base_frame = tof_vec_base_frame.flatten()[0:3]

        return tof_vec_base_frame

    def get_branch_vec_from_tof(self, return_frames: bool = False):
        # Project tof readings in base frame
        tof0_vec_base_frame = self.get_tof_vec_base_frame(
            ts=self.tof0_time_center, tof=self.tof0_distance_center, sensor_name="tof0"
        )
        tof1_vec_base_frame = self.get_tof_vec_base_frame(
            ts=self.tof1_time_center, tof=self.tof1_distance_center, sensor_name="tof1"
        )

        # Get the centerpoint of these two points.
        branch_center_point = np.mean([tof0_vec_base_frame, tof1_vec_base_frame], axis=0)  # C

        branch_vec = tof0_vec_base_frame - tof1_vec_base_frame
        branch_vec_normalized = branch_vec / np.linalg.norm(branch_vec)  # N

        if return_frames:
            return (branch_center_point, branch_vec_normalized), (tof0_vec_base_frame, tof1_vec_base_frame)
        else:
            return (branch_center_point, branch_vec_normalized), (None, None)

    def get_desired_position_from_branch_vec(self, branch_center_point: np.ndarray, branch_vec: np.ndarray):
        """
        Get closest point on a circle from point, given circle center,
        point, plane normal
        https://www.geometrictools.com/Documentation/DistanceToCircle3.pdf

        :param branch_vec: Unit vector of the branch in robot base domain.
        :type branch_vec: np.ndarray
        """
        tf_cut_point_to_base = self.lookup_transform(
            target_frame=f"{self._param_robot_base_part}__base",
            source_frame=f"{self._param_robot_eef_part}__tool0",
            sync=True,
            as_matrix=True,
            time=self.get_clock().now(),
        )
        curr_pose = tf_cut_point_to_base[0:3, 3]  # P

        delta = curr_pose - branch_center_point
        _Q_C = delta - np.dot(branch_vec, delta) * branch_vec

        desired_radius_from_branch = 0.10  # m

        desired_eef_xyz = branch_center_point + _Q_C / np.linalg.norm(_Q_C) * desired_radius_from_branch
        return desired_eef_xyz

    def get_desired_orientation_from_branch_vec(
        self, branch_center_point, branch_vec, desired_eef_xyz, return_vec: bool = False
    ):
        """Creates a set of basis vectors defining the desired coordinate system and returns a quaternion from the robot base frame"""
        desired_orientation_vec_to_branch = branch_center_point - desired_eef_xyz
        desired_orientation_vec_to_branch_norm = desired_orientation_vec_to_branch / np.linalg.norm(
            desired_orientation_vec_to_branch
        )
        desired_y_axis = np.cross(desired_orientation_vec_to_branch_norm, branch_vec)
        # Form the rotation matrix from our basis vectors
        rot_mat = np.column_stack((branch_vec, desired_y_axis, desired_orientation_vec_to_branch_norm))
        desired_orientation_rot = Rotation.from_matrix(rot_mat)
        desired_orientation_quat = desired_orientation_rot.as_quat()

        return desired_orientation_quat, desired_orientation_vec_to_branch_norm

    def publish_zero_twist(self):
        with self._servo_msg_lock:
            self._msg_twist.twist.linear.x = 0.0
            self._msg_twist.twist.linear.y = 0.0
            self._msg_twist.twist.linear.z = 0.0
            self._msg_twist.twist.angular.x = 0.0
            self._msg_twist.twist.angular.y = 0.0
            self._msg_twist.twist.angular.z = 0.0
            self._msg_twist.header.frame_id = (
                f"{self._param_robot_eef_part}__tool0"  # TODO: if changing to EEF, change ur_servo.yaml
            )
            self._msg_twist.header.stamp = self.get_clock().now().to_msg()
            self._pub_servo.publish(self._msg_twist)
        return

    async def switch_controllers(self, activate_controllers: list[str], deactivate_controllers: list[str]) -> None:
        try:
            switch_ctrlr_req = SwitchController.Request(
                activate_controllers=[activate_controllers],
                deactivate_controllers=[deactivate_controllers],
                strictness=SwitchController.Request.STRICT,
            )
            SwitchController.Response()
            switch_ctrlr_future: Future = self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)
            await switch_ctrlr_future
            if switch_ctrlr_future.result().ok:
                self.info(f"Successfully deactivated {deactivate_controllers}, activated {activate_controllers}")
            else:
                self.error("Failed to switch controllers,")
        except Exception as e:
            self.error(f"{e}")
            pass
        return

    async def start_servo(self) -> None:
        start_servo_future: Future = self._srv_client_start_servo.call_async(request=Trigger.Request())
        await start_servo_future
        if start_servo_future.result().success:
            self.info(f"Servo started.")
        else:
            self.error(f"Servo failed to start.")
        return

    async def stop_servo(self) -> None:
        stop_servo_future: Future = self._srv_client_stop_servo.call_async(request=Trigger.Request())
        await stop_servo_future
        if stop_servo_future.result().success:
            self.info(f"Servo stopped.")
        else:
            self.error(f"Servo failed to stop.")
        return


def main():
    rclpy.init()
    find_branch_roll_wrist_controller = FindBranchRollWristController()
    executor = MultiThreadedExecutor()
    rclpy.spin(find_branch_roll_wrist_controller, executor=executor)
    find_branch_roll_wrist_controller.destroy_node()
    rclpy.shutdown()
    return
