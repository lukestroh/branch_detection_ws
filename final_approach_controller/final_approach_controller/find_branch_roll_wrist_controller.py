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
from final_approach_controller_msgs.msg import ToFBranchFitStamped, WindowedData, TimestampTofMin, EventStamped
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
import branch_detection_system_analysis.plot.plotting_backend as pb
import final_approach_controller.curve_fitting as cf
import final_approach_controller.data_processing as dp
from final_approach_controller.tf_node import TFNode
from final_approach_controller.timer_state import TimerState

import copy
import modern_robotics as mr
import numpy as np
import os
import plotly.graph_objects as go
import plotly.io as pio
import scipy.optimize as so
import scipy.signal as ssi
from scipy.spatial.transform import Rotation
from threading import Event, Lock
import traceback

import pandas as pd
import py_trees


class FindBranchRollWristController(TFNode):
    def __init__(self):
        super().__init__(node_name="find_branch_roll_wrist_controller", cache_time=Duration(seconds=40))

        # Parameters
        self._param_far_plane_filter = (
            self.declare_parameter("far_plane_filter", value=Parameter.Type.DOUBLE).get_parameter_value().double_value
        )
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
        self._lock_raw_data = Lock()
        self._lock_filtered_data = Lock()
        self._lock_joint_states_data = Lock()
        self._lock_msg_twist = Lock()
        self._lock_rotations_complete = Lock()
        self._lock_timer_state_pub_servo = Lock()

        # Callback group
        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._cb_group_pub_servo = MutuallyExclusiveCallbackGroup()

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
            qos_profile=100,
        )
        self._sub_tof_filtered = self.create_subscription(
            msg_type=TofStamped,
            topic="/vl53l4cd/filtered",
            callback=self._sub_cb_tof_filtered,
            callback_group=self._reentrant_cb_group,
            qos_profile=100,
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
            qos_profile=100,
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self._cb_group_pub_servo,
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
        self._pub_rotation_started = self.create_publisher(
            msg_type=EventStamped,
            topic="/fbrw_controller/rotation_started",
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )
        self._pub_rotation_stopped = self.create_publisher(
            msg_type=EventStamped,
            topic="/fbrw_controller/rotation_stopped",
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )

        # self._pub_fit = self.create_publisher(
        #     msg_type=ToFBranchFitStamped,
        #     topic="find_branch_roll_wrist/tof_branch_fit",
        #     callback_group=self._reentrant_cb_group,
        #     qos_profile=5,
        # )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=3.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_state_pub_servo = TimerState.STOPPED
        self._timer_pub_servo = self.create_timer(
            timer_period_sec=1 / 50,
            callback=self._timer_cb_pub_servo,
            callback_group=self._cb_group_pub_servo,
        )

        # Messages
        self._msg_twist = TwistStamped()
        self._msg_twist.twist.linear.x = 0.0
        self._msg_twist.twist.linear.y = 0.0
        self._msg_twist.twist.linear.z = 0.0
        self._msg_twist.twist.angular.x = 0.0
        self._msg_twist.twist.angular.y = 0.0
        self._msg_twist.header.frame_id = f"{self._param_robot_eef_part}__tool0"
        # self.msg_tof_branch_fit = ToFBranchFitStamped()
        # self._msg_ts_tof_min = TimestampTofMin()

        self._msg_event_stamped = EventStamped()

        # Action requests
        self._move_to_pose_req = MoveToPose.Request()

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
            self.max_angular_vel = np.pi / 2
        else:
            self.max_angular_vel = np.pi / 8 * 10  # For some reason the UR5e scales down servoing movement very hard?

        # self.eef_weight = 0.355  # TODO: measure again. Measured IRL

        # Sensor attributes
        self.tof_far_plane = 0.250  # TODO: Get from param file
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

    def stop_servo_pub_timer(self):
        with self._lock_timer_state_pub_servo:
            if self._timer_state_pub_servo == TimerState.RUNNING:
                self._timer_state_pub_servo = TimerState.STOPPED
        return

    def start_servo_pub_timer(self):
        with self._lock_timer_state_pub_servo:
            if self._timer_state_pub_servo == TimerState.STOPPED:
                self._timer_state_pub_servo = TimerState.RUNNING
        return

    # ===============================
    #        Action callbacks
    # ===============================
    def _action_cancel_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        self.info("Canceling quadratic fit timer")
        self.publish_zero_twist()
        self.stop_servo_pub_timer()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        self.reset_controller()
        return CancelResponse.ACCEPT

    def _action_goal_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT

    async def _action_exe_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        # Let the tofs settle
        self.info("Beginning controller, initializing sensors...")
        self.get_clock().sleep_for(Duration(seconds=2.0))

        # feedback_msg = RunFindBranchRollWrist.Feedback()
        _result = RunFindBranchRollWrist.Result()

        # Reset all caches, controller flags
        self.reset_controller()

        trials_initial_joint_position: RunFindBranchRollWrist.Goal = goal_handle.request

        try:
            wrist_3_initial_position = trials_initial_joint_position.initial_joint_position[2]
        except IndexError:
            self.fatal(f"Start joints array was not populated")
            goal_handle.abort()
            _result.success = False
            return _result

        if not self.start_states_recorded:
            self.start_controller_tf = self.lookup_transform(
                target_frame=f"{self._param_robot_base_part}__base",
                source_frame=f"{self._param_robot_eef_part}__tool0",
                sync=True,
                as_matrix=True,
            )
            self.start_joint_states = copy.deepcopy(self.joint_states)

        self.info(trials_initial_joint_position)
        self.warn(self.start_joint_states)

        await self.start_servo()
        self._pub_rotation_speed.publish(Float64(data=self.max_angular_vel))
        self.publish_zero_twist()
        self.start_servo_pub_timer()
        self._msg_event_stamped.event = "start_rotations"
        self._msg_event_stamped.header.stamp = self.get_clock().now().to_msg()
        self._pub_rotation_started.publish(msg=self._msg_event_stamped)

        try:
            ##############################################################################################
            # Actuate wrist, collect data via subscriber callbacks
            while True:
                with self._lock_rotations_complete:
                    if self.rotations_complete:
                        self.stop_servo_pub_timer()
                        self.publish_zero_twist()
                        self._msg_event_stamped.event = "stop_rotations"
                        self._msg_event_stamped.header.stamp = self.get_clock().now().to_msg()
                        self._pub_rotation_stopped.publish(msg=self._msg_event_stamped)
                        break
                # self.get_clock().sleep_for(Duration(seconds=0.004))  # loop runs too fast, slow it down!
                if not self.check_action_goal_status(goal_handle=goal_handle):
                    goal_handle.abort()
                    _result.success = False
                    return _result
                self.actuate_wrist(wrist3_initial_pos=wrist_3_initial_position)
            ###############################################################################################

            # If rotating the wrist has completed, parse the data and find quadratic fits
            self.find_best_quadratic_fits()

            if (
                self.time_and_center_res_dict["s0"].get("time") is None
                or self.time_and_center_res_dict["s1"].get("time") is None
            ):
                self._pub_localization_success.publish(msg=Bool(data=False))
                self._pub_alignment_success.publish(msg=Bool(data=False))
                # if self.tof0_time_center is None or self.tof1_time_center is None:  # TODO: Check and/or logic here
                if not goal_handle.status == GoalStatus.STATUS_ABORTED:
                    self.warn("Could not find the branch. Aborting FindBranchRollWristController.")
                    goal_handle.abort()
                    _result.success = False
                return _result

            else:
                self._pub_localization_success.publish(msg=Bool(data=True))

                # Get branch info, find desired xyz + quat
                branch_center_point, branch_vec_normalized, tof0_vec_base_frame, tof1_vec_base_frame = (
                    self.get_branch_vec_from_tof(return_frames=self.debug_plot)
                )
                desired_eef_xyz = self.get_desired_position_from_branch_vec(
                    branch_center_point=branch_center_point, branch_vec=branch_vec_normalized
                )
                desired_orientation_quat, desired_orientation_vec = self.get_desired_orientation_from_branch_vec(
                    branch_center_point=branch_center_point,
                    branch_vec=branch_vec_normalized,
                    desired_eef_xyz=desired_eef_xyz,
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
                ######################################################################################

                # Move to desired pose
                await self.stop_servo()
                await self.switch_controllers(
                    activate_controllers=self._move_group_controller,
                    deactivate_controllers=self._servo_controller,
                )

                self.info(f"Moving to pose {desired_eef_xyz}, {desired_orientation_vec}")
                self._move_to_pose_req.goal.position.x = desired_eef_xyz[0]
                self._move_to_pose_req.goal.position.y = desired_eef_xyz[1]
                self._move_to_pose_req.goal.position.z = desired_eef_xyz[2]
                self._move_to_pose_req.goal.orientation.x = desired_orientation_quat[0]
                self._move_to_pose_req.goal.orientation.y = desired_orientation_quat[1]
                self._move_to_pose_req.goal.orientation.z = desired_orientation_quat[2]
                self._move_to_pose_req.goal.orientation.w = desired_orientation_quat[3]

                self.info("Sending goal")

                move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(request=self._move_to_pose_req)
                move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
                await move_group_future

                # Wait a bit for moving average filter to settle.
                self.get_clock().sleep_for(Duration(seconds=2.0))

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
        self.info("Move plan/execute finished.")
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
        with self._lock_timer_state_pub_servo:
            if self._timer_state_pub_servo != TimerState.RUNNING:
                return
        with self._lock_msg_twist:
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

        with self._lock_rotations_complete:
            rot_complete_flag = self.rotations_complete
        if rot_complete_flag:
            return

        # Append data to lists if the actuation is running
        timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock_raw_data:
            if msg.dev_id == 0:
                # if not self.tof0_branch_found:
                self.d_tof0_raw_readings.append(self.d_tof0_raw)
                self.ts_tof0_raw.append(timestamp_float)
            elif msg.dev_id == 1:
                # if not self.tof1_branch_found:
                self.d_tof1_raw_readings.append(self.d_tof1_raw)
                self.ts_tof1_raw.append(timestamp_float)
        return

    def _sub_cb_tof_filtered(self, msg: TofStamped):
        # Do some checks, make sure that the readings make sense in intuitive way.
        # Make sure readings do not exceed maximum. # TODO: Find a way to get sensor parameters in here
        # Append data to lists if the actuation is running
        if msg.dev_id == 0:
            self.d_tof0 = msg.data[0]
        elif msg.dev_id == 1:
            self.d_tof1 = msg.data[0]

        with self._lock_rotations_complete:
            rot_complete_flag = self.rotations_complete
        if rot_complete_flag:
            return

        timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock_filtered_data:
            if msg.dev_id == 0:
                # if not self.tof0_branch_found:
                self.d_tof0_readings.append(self.d_tof0)
                self.ts_tof0.append(timestamp_float)
            elif msg.dev_id == 1:
                # if not self.tof1_branch_found:
                self.d_tof1_readings.append(self.d_tof1)
                self.ts_tof1.append(timestamp_float)

        return

    def _sub_cb_joint_states(self, msg: JointState):
        self.joint_states = msg.position

        with self._lock_rotations_complete:
            rot_complete_flag = self.rotations_complete
        if not rot_complete_flag:
            timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._lock_joint_states_data:
                self.joint_states_cache.append(msg.position)
                self.ts_joint_states_cache.append(timestamp_float)
        return

    def _sub_cb_bag_record_path(self, msg: String):
        self.warn(f"{msg.data}")
        self.bag_record_path = msg.data
        return

    # ===============================
    #      Controller methods
    # ===============================
    def publish_zero_twist(self):
        with self._lock_msg_twist:
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

    async def switch_controllers(self, activate_controllers, deactivate_controllers) -> None:
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

    # ===============================
    #         Class methods
    # ===============================
    # def _wrap_angles_to_circle(self, angles: np.ndarray | float):
    #     return (angles + np.pi) % (2 * np.pi) - np.pi

    # def angular_distance(self, a, b):
    #     return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))

    def reset_controller(self) -> None:
        with self._lock_rotations_complete:
            self.rotations_complete = False

        self.time_and_center_res_dict = {}

        self.reset_data_caches()

        self.info("Controller parameters have been reset")
        return

    def reset_data_caches(self):
        with self._lock_raw_data:
            self.ts_tof0_raw = []
            self.d_tof0_raw_readings = []
            self.ts_tof1_raw = []
            self.d_tof1_raw_readings = []

        with self._lock_filtered_data:
            self.ts_tof0 = []
            self.d_tof0_readings = []
            self.ts_tof1 = []
            self.d_tof1_readings = []

        with self._lock_joint_states_data:
            self.joint_states_cache = []
            self.ts_joint_states_cache = []
        return

    def check_action_goal_status(self, goal_handle: ServerGoalHandle) -> bool:
        if goal_handle.status == GoalStatus.STATUS_CANCELED:
            self.stop_servo_pub_timer()
            self.publish_zero_twist()
            self.warn("FindBranchRollWristController canceled.")
            self._pub_localization_success.publish(msg=Bool(data=False))
            self._pub_alignment_success.publish(msg=Bool(data=False))
            self.reset_controller()
            return False

        if goal_handle.status == GoalStatus.STATUS_ABORTED:
            self.stop_servo_pub_timer()
            self.publish_zero_twist()
            self.error("FindBranchRollWristController aborted.")
            self._pub_localization_success.publish(msg=Bool(data=False))
            self._pub_alignment_success.publish(msg=Bool(data=False))
            self.reset_controller()
            return False

        if goal_handle.status == GoalStatus.STATUS_EXECUTING:
            return True

    def actuate_wrist(self, wrist3_initial_pos):
        """Determine direction of actuation, assign rotation speed to twist message. If rotation has reached termination point, set flag."""

        if self.start_joint_states[2] > 0.0:
            angular_z = -1 * self.max_angular_vel
            delta_theta_limit = np.pi
        else:
            angular_z = self.max_angular_vel
            delta_theta_limit = -1 * np.pi

        if np.isclose(self.start_joint_states[2] - self.joint_states[2], delta_theta_limit, atol=np.radians(1)):
            with self._lock_rotations_complete:
                self.rotations_complete = True

            self.stop_servo_pub_timer()
            self.publish_zero_twist()

            return
        with self._lock_msg_twist:
            self._msg_twist.twist.angular.z = angular_z
            self._msg_twist.header.stamp = self.get_clock().now().to_msg()
        return

    def aggregate_tof_data(self, save_fig: bool, save_fig_path: str, show_fig: bool) -> dict:
        """Concatenate all tof data so that it wraps around 360 degrees"""
        with self._lock_raw_data:
            sensor_data_dict = {"tof0": {}, "tof1": {}}
            sensor_data_dict["tof0"]["raw_tof_ts"] = list(self.ts_tof0_raw)
            sensor_data_dict["tof0"]["raw_tof_data"] = list(self.d_tof0_raw_readings)
            sensor_data_dict["tof1"]["raw_tof_ts"] = list(self.ts_tof1_raw)
            sensor_data_dict["tof1"]["raw_tof_data"] = list(self.d_tof1_raw_readings)

        with self._lock_filtered_data:
            sensor_data_dict["tof0"]["tof_ts"] = list(self.ts_tof0)
            sensor_data_dict["tof0"]["tof_data"] = list(self.d_tof0_readings)
            sensor_data_dict["tof1"]["tof_ts"] = list(self.ts_tof1)
            sensor_data_dict["tof1"]["tof_data"] = list(self.d_tof1_readings)

        with self._lock_joint_states_data:
            joint_angle_dict = {}
            joint_angle_dict["ts"] = list(self.ts_joint_states_cache)
            joint_angle_dict["data"] = list(self.joint_states_cache)

        _tof0_js_ts, joint_states_tof0_data = pb.get_joint_angles_at_closest_timestamps(
            joint_angle_dict=joint_angle_dict, timestamps=sensor_data_dict["tof0"]["tof_ts"]
        )
        joint_states_tof0_data[:, 2] += np.pi / 2
        _tof1_js_ts, joint_states_tof1_data = pb.get_joint_angles_at_closest_timestamps(
            joint_angle_dict=joint_angle_dict, timestamps=sensor_data_dict["tof1"]["tof_ts"]
        )
        joint_states_tof1_data[:, 2] -= np.pi / 2

        sensor_data_dict["tof0"]["joint_states_ts"] = _tof0_js_ts
        sensor_data_dict["tof0"]["joint_states_data"] = joint_states_tof0_data
        sensor_data_dict["tof0"]["sensor_id"] = [0] * len(sensor_data_dict["tof0"]["tof_ts"])

        sensor_data_dict["tof1"]["joint_states_ts"] = _tof1_js_ts
        sensor_data_dict["tof1"]["joint_states_data"] = joint_states_tof1_data
        sensor_data_dict["tof1"]["sensor_id"] = [1] * len(sensor_data_dict["tof1"]["tof_ts"])

        # TODO: Debug plot here
        if self.debug_plot:
            parabola_fig = dplot.plot_tof_vs_joint_state(data=sensor_data_dict["tof0"], name="tof0")
            parabola_fig = dplot.plot_tof_vs_joint_state(data=sensor_data_dict["tof1"], name="tof1", fig=parabola_fig)
            proj_2d_fig = dplot.plot_2d_tof_projection(
                data=sensor_data_dict["tof0"], far_plane_filter=self._param_far_plane_filter, name="tof0"
            )
            proj_2d_fig = dplot.plot_2d_tof_projection(
                data=sensor_data_dict["tof1"],
                far_plane_filter=self._param_far_plane_filter,
                name="tof1",
                fig=proj_2d_fig,
            )
            proj_3d_fig = dplot.plot_3d_tof_projection(
                data=sensor_data_dict["tof0"], far_plane_filter=self._param_far_plane_filter, name="tof0"
            )
            proj_3d_fig = dplot.plot_3d_tof_projection(
                data=sensor_data_dict["tof1"],
                far_plane_filter=self._param_far_plane_filter,
                name="tof1",
                fig=proj_3d_fig,
            )

            if save_fig:
                pio.write_html(
                    fig=parabola_fig,
                    file=os.path.join(save_fig_path, "tof_vs_joint_state_by_sensor.html"),
                    auto_open=show_fig,
                )
                pio.write_html(
                    fig=proj_2d_fig,
                    file=os.path.join(save_fig_path, "tof_vs_joint_state_2d_proj.html"),
                    auto_open=show_fig,
                )
                pio.write_html(
                    fig=proj_3d_fig,
                    file=os.path.join(save_fig_path, "tof_vs_joint_state_3d_proj.html"),
                    auto_open=show_fig,
                )
            else:
                if show_fig:
                    parabola_fig.show()
                    proj_2d_fig.show()
                    proj_3d_fig.show()

        all_data_dict = {}
        all_data_dict["raw_tof_ts"] = np.concatenate(
            [sensor_data_dict["tof0"]["raw_tof_ts"], sensor_data_dict["tof1"]["raw_tof_ts"]]
        )
        all_data_dict["raw_tof_data"] = np.concatenate(
            [sensor_data_dict["tof0"]["raw_tof_data"], sensor_data_dict["tof1"]["raw_tof_data"]]
        )
        all_data_dict["tof_ts"] = np.concatenate(
            [sensor_data_dict["tof0"]["tof_ts"], sensor_data_dict["tof1"]["tof_ts"]]
        )
        all_data_dict["tof_data"] = np.concatenate(
            [sensor_data_dict["tof0"]["tof_data"], sensor_data_dict["tof1"]["tof_data"]]
        )
        all_data_dict["joint_states_ts"] = np.concatenate(
            [sensor_data_dict["tof0"]["joint_states_ts"], sensor_data_dict["tof1"]["joint_states_ts"]]
        )
        all_data_dict["joint_states_data"] = np.concatenate(
            (sensor_data_dict["tof0"]["joint_states_data"], sensor_data_dict["tof1"]["joint_states_data"])
        )
        all_data_dict["sensor_id"] = np.concatenate(
            (sensor_data_dict["tof0"]["sensor_id"], sensor_data_dict["tof1"]["sensor_id"])
        )

        # Sort all of my data by wrist 3 joint state
        sorted_indices = np.argsort(all_data_dict["joint_states_data"][:, 2])
        # self.debug(all_data_dict['raw_tof_data'].shape)
        # self.debug(all_data_dict['tof_data'].shape)
        # self.debug(all_data_dict['joint_states_data'].shape)
        # self.debug(sorted_indices.shape)
        # all_data_dict["raw_tof_ts"] = all_data_dict["raw_tof_ts"][sorted_indices]
        # all_data_dict["raw_tof_data"] = all_data_dict["raw_tof_data"][sorted_indices]
        all_data_dict["tof_ts"] = all_data_dict["tof_ts"][sorted_indices]
        all_data_dict["tof_data"] = all_data_dict["tof_data"][sorted_indices]
        all_data_dict["joint_states_ts"] = all_data_dict["joint_states_ts"][sorted_indices]
        all_data_dict["joint_states_data"] = all_data_dict["joint_states_data"][sorted_indices]
        all_data_dict["sensor_id"] = all_data_dict["sensor_id"][sorted_indices]

        return all_data_dict, sensor_data_dict

    def find_best_quadratic_fits(self):
        all_data_dict, sensor_data_dict = self.aggregate_tof_data(
            save_fig=True, save_fig_path=self.bag_record_path, show_fig=True
        )

        separated_data_dict = dp.separate_tof_data_by_curve(
            node=self, all_data_dict=all_data_dict, save_fig=True, save_fig_path=self.bag_record_path, show_fig=True
        )

        # Shift section joint angles as needed if discontiunity exists
        if (
            separated_data_dict["s0"]["joint_states_data"][:, 2][0]
            < separated_data_dict["s1"]["joint_states_data"][:, 2][0]
        ):
            separated_data_dict["s0"]["joint_states_data"][:, 2] = dp.amend_joint_angle_discontinuity(
                node=self,
                joint_angles=separated_data_dict["s0"]["joint_states_data"][:, 2],
                indices=separated_data_dict["s0"]["indices"],
            )
        else:
            separated_data_dict["s1"]["joint_states_data"][:, 2] = dp.amend_joint_angle_discontinuity(
                node=self,
                joint_angles=separated_data_dict["s1"]["joint_states_data"][:, 2],
                indices=separated_data_dict["s1"]["indices"],
            )

        # Curve fit
        for section_name, section in separated_data_dict.items():
            self.time_and_center_res_dict[section_name] = {}
            sec_time_and_dist = cf.get_branch_center_time_and_distance(
                data=section,
                far_plane_filter=self._param_far_plane_filter,
                section_name=section_name,
                debug_plot=True,
                show_fig=False,
                save_fig=True,
                save_fig_path=self.bag_record_path,
                window_size=0.4,
                window_overlap_ratio=7 / 10,
                min_samples=10,
                max_trials=20,
                residual_threshold=0.008,
                node=self,
            )

            if sec_time_and_dist is not None:
                self.info(f"{section_name}: {sec_time_and_dist}")
                (
                    self.time_and_center_res_dict[section_name]["time"],
                    self.time_and_center_res_dict[section_name]["min_dist"],
                    self.time_and_center_res_dict[section_name]["sensor_id"],
                ) = sec_time_and_dist

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
            ts=self.time_and_center_res_dict["s0"]["time"],
            tof=self.time_and_center_res_dict["s0"]["min_dist"],
            sensor_name=f"tof{self.time_and_center_res_dict['s0']['sensor_id']}",
        )
        tof1_vec_base_frame = self.get_tof_vec_base_frame(
            ts=self.time_and_center_res_dict["s1"]["time"],
            tof=self.time_and_center_res_dict["s1"]["min_dist"],
            sensor_name=f"tof{self.time_and_center_res_dict['s1']['sensor_id']}",
        )

        # Get the centerpoint of these two points.
        branch_center_point = np.mean([tof0_vec_base_frame, tof1_vec_base_frame], axis=0)  # C

        branch_vec = tof0_vec_base_frame - tof1_vec_base_frame
        branch_vec_normalized = branch_vec / np.linalg.norm(branch_vec)  # N

        if return_frames:
            return branch_center_point, branch_vec_normalized, tof0_vec_base_frame, tof1_vec_base_frame
        else:
            return branch_center_point, branch_vec_normalized, None, None

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


def main():
    rclpy.init()
    find_branch_roll_wrist_controller = FindBranchRollWristController()
    executor = MultiThreadedExecutor()
    rclpy.spin(find_branch_roll_wrist_controller, executor=executor)
    find_branch_roll_wrist_controller.destroy_node()
    rclpy.shutdown()
    return
