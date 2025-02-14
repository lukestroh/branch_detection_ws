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

import final_approach_controller.plotly_helpers as ph

from branch_detection_system_moveit_msgs.srv import MoveToPose
from final_approach_controller_msgs.action import RunFindBranchRollWrist
from final_approach_controller_msgs.msg import ToFBranchFitStamped
import final_approach_controller.curve_fitting as cf
from final_approach_controller.tf_node import TFNode
from vl6180_msgs.msg import Vl6180, Vl6180FilteredStamped
from vl53l4cd_msgs.msg import Vl53l4cdStamped
from tof_msgs.msg import TofStamped

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped, Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup

# from moveit_msgs.srv import GetCartesianPath
# from moveit_msgs.msg import (
#     RobotState,
#     MotionPlanRequest,
#     JointConstraint,
#     OrientationConstraint,
#     PositionConstraint,
#     Constraints,
#     PlanningOptions,
# )
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

import modern_robotics as mr
import numpy as np
import scipy.optimize as so
from scipy.spatial.transform import Rotation

from threading import Event, Lock
import traceback
import plotly.graph_objects as go

# import copy


class FindBranchRollWristController(TFNode):
    def __init__(self):
        super().__init__(node_name="find_branch_roll_wrist_controller", cache_time=Duration(seconds=30))
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Launch arguments
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

        # self.error(f"{self._param_robot_base_part}")
        # self.error(f"{self._param_robot_eef_part}")

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
        self._srv_move_to_pose = self.create_client(
            srv_type=MoveToPose, srv_name="/move_to_pose", callback_group=self._reentrant_cb_group
        )
        # self._srv_move_to_pose.wait_for_service()

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
        # self._sub_tof0_filtered = self.create_subscription(
        #     msg_type=TofStamped,
        #     topic='/vl53l4cd/tof0/filtered',
        #     callback=self._sub_cb_tof0_filtered,
        #     callback_group=self._reentrant_cb_group,
        #     qos_profile=5
        # )
        # self._sub_tof1_filtered = self.create_subscription(
        #     msg_type=TofStamped,
        #     topic='/vl53l4cd/tof1/filtered',
        #     callback=self._sub_cb_tof1_filtered,
        #     callback_group=self._reentrant_cb_group,
        #     qos_profile=5
        # )
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
        # Fit data publisher
        self._pub_fit = self.create_publisher(
            msg_type=ToFBranchFitStamped,
            topic="find_branch_roll_wrist/tof_branch_fit",
            callback_group=self._reentrant_cb_group,
            qos_profile=5,
        )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=3.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_run_quadratic_fit = None
        self._timer_pub_servo = None
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
        self.reset_controller()
        self.feedback_pub_prev_time = self.get_clock().now()
        self.start_states_recorded = False
        if _param_use_mock_hardware:
            self.max_angular_vel = np.pi / 16
        else:
            self.max_angular_vel = np.pi / 16 * 10  # For some reason the UR5e scales down servoing movement very hard?

        # self.max_angular_vel = np.pi / 2

        self.debug_plot = True
        self.eef_weight = 0.355  # TODO: measure again. Measured IRL

        # Sensor attributes
        self.vl6180_far_plane = 0.200  # 0.19 based on testing, but give it small window. TODO: Get from param file
        self.vl6180_precision = 0.001

        self.d_tof0 = 0.255
        self.d_tof1 = 0.255
        self.d_tof0_raw = 0.255
        self.d_tof1_raw = 0.255

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
            # if not self._timer_run_quadratic_fit.is_canceled():
            #     self._timer_run_quadratic_fit.cancel()
        goal_handle.abort()
        self.reset_controller()
        return CancelResponse.ACCEPT

    async def _action_exe_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.controller_running = True

        start_servo_future: Future = self._srv_client_start_servo.call_async(request=Trigger.Request())
        await start_servo_future
        if start_servo_future.result().success:
            self.info(f"Servo started")
        else:
            self.error(f"Servo failed to start")

        if not self.start_states_recorded:
            self.start_controller_tf = self.lookup_transform(
                target_frame=f"{self._param_robot_base_part}__base",
                source_frame=f"{self._param_robot_eef_part}__tool0",
                sync=True,
                as_matrix=True,
            )
            self.start_joint_states = self.joint_states

        with self._timer_lock:
            # if self._timer_run_quadratic_fit is None:
            #     self._timer_run_quadratic_fit = self.create_timer(
            #         timer_period_sec=4.0,
            #         callback=self._timer_cb_run_quadratic_fit,
            #         callback_group=self._parabola_fitting_cb_group,
            #     )
            # else:
            #     self._timer_run_quadratic_fit.reset()
            if self._timer_pub_servo is None:
                self._timer_pub_servo = self.create_timer(
                    timer_period_sec=1 / 250,
                    callback=self._timer_cb_pub_servo,
                    # callback_group=self._pub_servo_cb_group
                    callback_group=self._reentrant_cb_group,
                )
            else:
                self._timer_pub_servo.reset()

        try:
            feedback_msg = RunFindBranchRollWrist.Feedback()
            result = RunFindBranchRollWrist.Result()

            # TODO: check if initial reading of sensor. If so, set flag to data found and record tf pose time.

            while self.controller_running:
                # self.debug_counter += 1
                self.get_clock().sleep_for(Duration(seconds=0.005))  # loop runs too fast, slow it down!

                if goal_handle.status == GoalStatus.STATUS_CANCELED:
                    # self._timer_run_controller.cancel()
                    with self._timer_lock:
                        # if not self._timer_run_quadratic_fit.is_canceled():
                        #     self._timer_run_quadratic_fit.cancel()
                        if not self._timer_pub_servo.is_canceled():
                            self._timer_pub_servo.cancel()
                    result.success = False
                    return result

                if goal_handle.status == GoalStatus.STATUS_ABORTED:
                    with self._timer_lock:
                        # if not self._timer_run_quadratic_fit.is_canceled():
                        #     self._timer_run_quadratic_fit.cancel()
                        if not self._timer_pub_servo.is_canceled():
                            self._timer_pub_servo.cancel()
                    result.success = False
                    return result

                if goal_handle.status == GoalStatus.STATUS_EXECUTING:
                    # self.warn(f"gen rot complete: {self.rotations_complete}")
                    # self.warn(f"neg rot complete: {self.neg_rot_complete}")
                    # self.warn(f"pos rot complete: {self.pos_rot_complete}")

                    # Cancel action if requested
                    if goal_handle.is_cancel_requested:
                        self.publish_zero_twist()
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

                    # self.warn(f"tof0 found: {self.tof0_branch_found}")
                    # self.warn(f"tof1 found: {self.tof1_branch_found}")
                    with self._branch_found_lock:

                        # self.info("")
                        tof0_branch_found = self.tof0_branch_found
                        tof1_branch_found = self.tof1_branch_found
                        # self.debug_counter -= 1

                    if tof0_branch_found and tof1_branch_found:
                        self.info("Branch found!")
                        self.info(f"tof0: {tof0_branch_found}, tof1: {tof1_branch_found}")

                        self.neg_rot_complete = True
                        self.pos_rot_complete = True
                        self.rotations_complete = True

                        # with self._timer_lock:
                        # if not self._timer_run_quadratic_fit.is_canceled():
                        #     self._timer_run_quadratic_fit.cancel()
                        self.publish_zero_twist()
                        self.info("Branch readings found for both ToFs!")

                    if not self.rotations_complete:
                        if not self.neg_rot_complete:
                            # self.info(self.start_joint_states[2] - self.joint_states[2])         
                            # # rotate to the closest side
                            # if self.joint_states[-1] < 0 and self.joint_states[-1] > -1 * np.pi:
                            # negative angular rotation
                            angular_z = -1 * self.max_angular_vel
                            if np.isclose(self.start_joint_states[2] - self.joint_states[2], np.pi / 2, atol=0.05):
                                # TODO: (long term) make sure wrist mount config is standard
                                self.run_quadratic_fit()
                                self.neg_rot_complete = True
                                self.publish_zero_twist()
                                # self.get_clock().sleep_for(Duration(seconds=3))
                                if not tof0_branch_found:
                                    self.d_tof0_raw_readings = []
                                    self.d_tof0_readings = []
                                    self.timestamp_readings_tof0 = []
                                    self.timestamp_readings_tof0_raw = []
                                if not tof1_branch_found:
                                    self.d_tof0_raw_readings = []
                                    self.d_tof1_readings = []
                                    self.timestamp_readings_tof1 = []
                                    self.timestamp_readings_tof0_raw = []

                        elif not self.pos_rot_complete:
                            # if self.joint_states[-1] > 0 and self.joint_states[-1] < np.pi:
                            # positive angular rotation
                            angular_z = self.max_angular_vel
                            if np.isclose(self.start_joint_states[2] - self.joint_states[2], -np.pi / 2, atol=0.05):
                                self.run_quadratic_fit()
                                self.pos_rot_complete = True
                                self.publish_zero_twist()

                        with self._servo_msg_lock:  # TODO: Fill out once
                            self.msg_twist.twist.linear.x = 0.0
                            self.msg_twist.twist.linear.y = 0.0
                            self.msg_twist.twist.linear.z = 0.0
                            self.msg_twist.twist.angular.x = 0.0
                            self.msg_twist.twist.angular.y = 0.0
                            self.msg_twist.twist.angular.z = angular_z
                            self.msg_twist.header.frame_id = f"{self._param_robot_eef_part}__tool0"
                            self.msg_twist.header.stamp = self.get_clock().now().to_msg()

                        if self.neg_rot_complete and self.pos_rot_complete:
                            self.publish_zero_twist()
                            self.rotations_complete = True

                    else:
                        if (
                            self.tof0_time_center is None or self.tof1_time_center is None
                        ):  # TODO: Check and/or logic here
                            if not goal_handle.status == GoalStatus.STATUS_ABORTED:
                                self.warn("Could not find the branch. Aborting FindBranchRollWristController.")
                                # with self._timer_lock:
                                #     if not self._timer_run_quadratic_fit.is_canceled():
                                #         self._timer_run_quadratic_fit.cancel()
                                goal_handle.abort()
                                # self.reset_controller() # Done in 'finally'

                                result.success = False
                            return result

                        else:
                            # Stop servo
                            self.publish_zero_twist()  # Just in case
                            with self._timer_lock:
                                if not self._timer_pub_servo.is_canceled():
                                    self._timer_pub_servo.cancel()
                            self.info("Stopping servo...")
                            stop_servo_response: Trigger.Response = self._srv_client_stop_servo.call(
                                request=Trigger.Request()
                            )
                            if not stop_servo_response.success:
                                raise Exception("Failed to stop servo.")
                            else:
                                self.info("Servo stopped.")

                            # Switch controllers
                            self.info(
                                f"Switching controllers, deactivating {self._servo_controller}, activating {self._move_group_controller}"
                            )
                            while True:
                                switch_ctrlr_req = SwitchController.Request(
                                    activate_controllers=[self._move_group_controller],
                                    deactivate_controllers=[self._servo_controller],
                                    strictness=SwitchController.Request.STRICT,
                                    # timeout=5.0
                                    # timeout=Duration(seconds=5)
                                )
                                switch_ctrlr_future: Future = self._srv_switch_ctrls.call_async(
                                    request=switch_ctrlr_req
                                )
                                await switch_ctrlr_future
                                if switch_ctrlr_future.result().ok:
                                    self.info("Successfully switched controllers")
                                    break
                                else:
                                    self.error("Failed to switch controllers,")
                                    self.get_clock().sleep_for(Duration(seconds=2.0))

                            # If the eef is moving, we need a common frame, which should be world or <robot-part>__base
                            # Get tof poses at calculated signal minimum times
                            tf_tof0_to_base__time_center_pose = self.lookup_transform(
                                target_frame=f"{self._param_robot_base_part}__base",  # TODO: probably best to dynamically get robot base, whatever it is.
                                source_frame=f"{self._param_robot_eef_part}__tof0",
                                time=self.tof0_time_center,
                                sync=True,
                                as_matrix=True,
                            )
                            tf_tof1_to_base__time_center_pose = self.lookup_transform(
                                target_frame=f"{self._param_robot_base_part}__base",
                                source_frame=f"{self._param_robot_eef_part}__tof1",
                                time=self.tof1_time_center,
                                sync=True,
                                as_matrix=True,
                            )
                            # Get the current eef pose
                            tf_cut_point_to_base = self.lookup_transform(
                                target_frame=f"{self._param_robot_base_part}__base",
                                source_frame=f"{self._param_robot_eef_part}__tool0",
                                sync=True,
                                as_matrix=True,
                                time=self.get_clock().now(),
                            )

                            # self.warn(f"POSE:\n{tf_tof0_to_base__time_center_pose}")
                            # self.warn(f"POSE:\n{tf_tof1_to_base__time_center_pose}")

                            # Get branch locations in world coordinates
                            tof0_vec_tof0_frame = np.array([[0, 0, self.tof0_distance_center, 1]]).T
                            tof1_vec_tof1_frame = np.array([[0, 0, self.tof1_distance_center, 1]]).T

                            # self.error(tof0_vec_tof0_frame)

                            tof0_vec_base_frame = tf_tof0_to_base__time_center_pose @ tof0_vec_tof0_frame
                            tof1_vec_base_frame = tf_tof1_to_base__time_center_pose @ tof1_vec_tof1_frame
                            # self.warn(f"VEC:\n{tof0_vec_base_frame}")
                            # self.warn(f"VEC:\n{tof1_vec_base_frame}")
                            tof0_vec_base_frame = tof0_vec_base_frame.flatten()[0:3]
                            tof1_vec_base_frame = tof1_vec_base_frame.flatten()[0:3]
                            # self.warn(tof0_vec_base_frame)

                            # Get the centerpoint of these two points.
                            branch_center_point = np.mean([tof0_vec_base_frame, tof1_vec_base_frame], axis=0)  # C
                            # self.warn(f"CENTER:\n{branch_center_point}")

                            # Get closest point on a circle from point, given circle center,
                            # point, plane normal
                            # https://www.geometrictools.com/Documentation/DistanceToCircle3.pdf

                            branch_vec = tof0_vec_base_frame - tof1_vec_base_frame
                            branch_vec_normalized = branch_vec / np.linalg.norm(branch_vec)  # N

                            curr_pose = tf_cut_point_to_base[0:3, 3]  # P

                            delta = curr_pose - branch_center_point
                            _Q_C = delta - np.dot(branch_vec_normalized, delta) * branch_vec_normalized

                            desired_radius_from_branch = 0.08  # m

                            desired_eef_xyz = (
                                branch_center_point + _Q_C / np.linalg.norm(_Q_C) * desired_radius_from_branch
                            )

                            desired_orientation_vec_to_branch = branch_center_point - desired_eef_xyz

                            desired_orientation_vec_to_branch_norm = desired_orientation_vec_to_branch / np.linalg.norm(
                                desired_orientation_vec_to_branch
                            )

                            # CREATE BASIS VECTORS: we need to create a coordinate basis with respect to the calculated vector. Use a temp vec, like world_z. Mimicking the 'camera' frame, this would give us the x-axis, which we want pointing to the right. Therefore, we should take desired x world_z
                            # desired_x_axis = np.cross(desired_orientation_vec_to_branch_norm, world_z)
                            # desired_x_axis = desired_x_axis / np.linalg.norm(
                            #     desired_x_axis
                            # )  # this SHOULD be 1 already....
                            # desired_y_axis = np.cross(desired_orientation_vec_to_branch_norm, desired_x_axis)
                            # desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)

                            desired_y_axis = np.cross(desired_orientation_vec_to_branch_norm, branch_vec_normalized)
                            # Form the rotation matrix from our basis vectors
                            rot_mat = np.column_stack(
                                (branch_vec_normalized, desired_y_axis, desired_orientation_vec_to_branch_norm)
                            )
                            desired_orientation_rot = Rotation.from_matrix(rot_mat)
                            desired_orientation_quat = desired_orientation_rot.as_quat()

                            ######################################################################################
                            if self.debug_plot:
                                fig = go.Figure()
                                fig.add_trace(
                                    go.Scatter3d(x=[0], y=[0], z=[0], name=f"{self._param_robot_base_part}__base")
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
                                ph.plot_vector(
                                    fig=fig,
                                    position=desired_eef_xyz,
                                    orientation=desired_orientation_vec_to_branch_norm,
                                    scale=0.5,
                                    color="blue",
                                    anchor="tail",
                                )
                                ph.plot_vector(
                                    fig=fig,
                                    position=desired_eef_xyz,
                                    orientation=branch_vec_normalized,
                                    scale=0.5,
                                    color="red",
                                    anchor="tail",
                                )
                                fig.update_layout(scene=dict(aspectmode="data"))

                                fig.show()

                            #####################################################
                            self.info(f"Moving to pose {desired_eef_xyz}, {desired_orientation_vec_to_branch}")
                            move_to_pose_req = MoveToPose.Request()
                            move_to_pose_req.goal.position.x = desired_eef_xyz[0]
                            move_to_pose_req.goal.position.y = desired_eef_xyz[1]
                            move_to_pose_req.goal.position.z = desired_eef_xyz[2]
                            move_to_pose_req.goal.orientation.x = desired_orientation_quat[0]
                            move_to_pose_req.goal.orientation.y = desired_orientation_quat[1]
                            move_to_pose_req.goal.orientation.z = desired_orientation_quat[2]
                            move_to_pose_req.goal.orientation.w = desired_orientation_quat[3]

                            # input("Hit enter to continue algorithm.")

                            
                            self.info("Sending goal")
                            

                            move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(
                                request=move_to_pose_req
                            )
                            move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
                            await move_group_future
                            # rclpy.spin_until_future_complete(node=self, future=move_group_future)

                            

                            if move_group_future.result() is None or not move_group_future.result().result:
                                goal_handle.abort()
                                result.success = False
                            else:
                                if self.d_tof0 < self.vl6180_far_plane and self.d_tof1 < self.vl6180_far_plane:
                                    goal_handle.succeed()
                                    result.success = True
                                else:
                                    goal_handle.abort()
                                    result.success = False
                                    self.error("Failed to navigate to pose where both sensors can read the branch.")

                            self.controller_running = False
                            return result
            ############################################################################################################

        except Exception as e:
            self.get_logger().fatal(f"{traceback.format_exc()}")
            result.success = False
            goal_handle.abort()
        finally:
            # self._timer_run_controller.cancel()
            self.publish_zero_twist()
            # with self._timer_lock:
            # if self._timer_run_quadratic_fit is not None:
            #     if not self._timer_run_quadratic_fit.is_canceled():
            #         self._timer_run_quadratic_fit.cancel()

            

            # switch_ctrlr_req = SwitchController.Request(
            #     activate_controllers=[self._servo_controller],
            #     deactivate_controllers=[self._move_group_controller],
            #     strictness=SwitchController.Request.STRICT,
            # )
            # switch_ctrlr_future: Future = self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)
            # await switch_ctrlr_future
            # if switch_ctrlr_future.result().ok:
            #     self.info("Successfully switched controllers")
            # else:
            #     self.error("Failed to switch controllers,")

            # if not result.success:
            #     self.info(f"Failed to find branch. Returning to start position.")
            #     while not np.isclose(self.start_joint_states[2], self.joint_states[2], atol=0.01):
            #         if self.joint_states[2] > self.start_joint_states[2]:
            #             angular_z = -1 * self.max_angular_vel
                    
            #         else:
            #             angular_z = self.max_angular_vel

            #         self.msg_twist.twist.linear.x = 0.0
            #         self.msg_twist.twist.linear.y = 0.0
            #         self.msg_twist.twist.linear.z = 0.0
            #         self.msg_twist.twist.angular.x = 0.0
            #         self.msg_twist.twist.angular.y = 0.0
            #         self.msg_twist.twist.angular.z = angular_z
            #         self.msg_twist.header.frame_id = f"{self._param_robot_eef_part}__tool0"
            #         self.msg_twist.header.stamp = self.get_clock().now().to_msg()

            #     self.info("Returned to start position")
        
            self.publish_zero_twist()
            self.reset_controller()
            self.info("FindBranchRollWristController has terminated.")

            self.stop_servo()

            # Stop forward pos con
            # switch_ctrlr_req = SwitchController.Request(
            # activate_controllers=[self._servo_controller],
            # deactivate_controllers=[self._move_group_controller],
            # strictness=SwitchController.Request.STRICT,
            # )
            # switch_ctrlr_future: Future = self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)
            # await switch_ctrlr_future
            # if switch_ctrlr_future.result().ok:
            #     self.info(f"Successfully deactivated {self._move_group_controller}, activated {self._servo_controller}")
            # else:
            #     self.error("Failed to switch controllers,")

            self.switch_controllers(activate_controllers=self._servo_controller, deactivate_controllers=self._move_group_controller)

            self.get_clock().sleep_for(Duration(seconds=2.0))
            

        return result

    def _action_goal_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT
    
    async def switch_controllers(self, activate_controllers: list[str], deactivate_controllers: list[str]) -> None:
        switch_ctrlr_req = SwitchController.Request(
            activate_controllers=[activate_controllers],
            deactivate_controllers=[deactivate_controllers],
            strictness=SwitchController.Request.STRICT,
        )
        switch_ctrlr_future: Future = self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)
        await switch_ctrlr_future
        if switch_ctrlr_future.result().ok:
            self.info(f"Successfully deactivated {deactivate_controllers}, activated {activate_controllers}")
        else:
            self.error("Failed to switch controllers,")
        return
    
    async def start_servo(self) -> None:

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
        with self._servo_msg_lock:
            self._pub_servo.publish(self.msg_twist)
        return

    # def _timer_cb_run_controller(self):
    #     return

    def _timer_cb_run_quadratic_fit(self):
        """Periodically run a fit on the data"""
        self.run_quadratic_fit()
        return

    def run_quadratic_fit(self):
        with self._branch_found_lock:
            tof0_branch_found = self.tof0_branch_found

        if not tof0_branch_found:
            # Make a copy of the recorded data so that the subscriber may continue to append
            with self._data_lock:
                timestamp_readings_tof0_raw_copy = list(self.timestamp_readings_tof0_raw)
                timestamp_readings_tof0_copy = list(self.timestamp_readings_tof0)
                d_tof0_readings_raw_copy = list(self.d_tof0_raw_readings)
                d_tof0_readings_copy = list(self.d_tof0_readings)
            tof0_time_and_dist = cf.get_branch_center_time_and_distance(
                node=self,
                raw_timestamps=timestamp_readings_tof0_raw_copy,
                filtered_timestamps=timestamp_readings_tof0_copy,
                raw_readings=d_tof0_readings_raw_copy,
                filtered_readings=d_tof0_readings_copy,
                sensor_name="tof0",
                debug_plot=self.debug_plot,
            )
            if tof0_time_and_dist is not None:
                self.tof0_time_center, self.tof0_distance_center = tof0_time_and_dist
                with self._branch_found_lock:
                    self.tof0_branch_found = True
        else:
            self.info("Branch already detected by tof0. Skipping.")

        with self._branch_found_lock:
            tof1_branch_found = self.tof1_branch_found
        if not tof1_branch_found:
            with self._data_lock:
                timestamp_readings_tof1_raw_copy = list(self.timestamp_readings_tof1_raw)
                timestamp_readings_tof1_copy = list(self.timestamp_readings_tof1)
                d_tof1_readings_raw_copy = list(self.d_tof1_raw_readings)
                d_tof1_readings_copy = list(self.d_tof1_readings)
            tof1_time_and_dist = cf.get_branch_center_time_and_distance(
                node=self,
                raw_timestamps=timestamp_readings_tof1_raw_copy,
                filtered_timestamps=timestamp_readings_tof1_copy,
                raw_readings=d_tof1_readings_raw_copy,
                filtered_readings=d_tof1_readings_copy,
                sensor_name="tof1",
                debug_plot=self.debug_plot,
            )
            if tof1_time_and_dist is not None:
                self.tof1_time_center, self.tof1_distance_center = tof1_time_and_dist
                with self._branch_found_lock:
                    self.tof1_branch_found = True
        else:
            self.info("Branch already detected by tof1. Skipping.")
        return

    def _timer_cb_debug(self):
        return

    # ===============================
    #     Subscription callbacks
    # ===============================

    def _sub_cb_tof_raw(self, msg: Vl53l4cdStamped):
        

        if msg.dev_id == 0:
            self.d_tof0_raw = msg.distance # TODO: this isn't the 'raw' value, it's the
        elif msg.dev_id == 1:
            self.d_tof1_raw = msg.distance

        if self.controller_running:
            # timestamp_tuple = self.get_clock().now().seconds_nanoseconds()
            # timestamp_float = timestamp_tuple[0] + timestamp_tuple[1] * 1e-9
            timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            # with self._branch_found_lock:
            with self._data_lock:
                if msg.dev_id == 0:
                    if not self.tof0_branch_found:
                        self.d_tof0_raw_readings.append(self.d_tof0_raw)
                        self.timestamp_readings_tof0_raw.append(timestamp_float)
                elif msg.dev_id == 1:
                    if not self.tof1_branch_found:
                        self.d_tof1_raw_readings.append(self.d_tof1_raw)
                        self.timestamp_readings_tof1_raw.append(timestamp_float)

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
                        self.timestamp_readings_tof0.append(timestamp_float)
                elif msg.dev_id == 1:
                    if not self.tof1_branch_found:
                        self.d_tof1_readings.append(self.d_tof1)
                        self.timestamp_readings_tof1.append(timestamp_float)


        return

    def _sub_cb_joint_states(self, msg: JointState):
        self.joint_states = msg.position
        return

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
        self.start_controller_tf = np.identity(4)  # TODO: unused.
        with self._data_lock:
            self.d_tof0_readings = []
            self.d_tof1_readings = []
            self.d_tof0_raw_readings = []
            self.d_tof1_raw_readings = []
            # self.d_tof0_readings_filtered = []
            # self.d_tof1_readings_filtered = []
            self.timestamp_readings_tof0 = []
            self.timestamp_readings_tof1 = []
            self.timestamp_readings_tof0_raw = []
            self.timestamp_readings_tof1_raw = []
            # self.timestamps_tof0_filtered = []
            # self.timestamps_tof1_filtered = []

        # with self._timer_lock:
        #     if self._timer_run_quadratic_fit is not None:
        #         self._timer_run_quadratic_fit.cancel()
        # self._action_client_move_group_done_event.clear()

        self.info("Controller parameters have been reset")
        return

    def publish_zero_twist(self):
        with self._servo_msg_lock:
            self.msg_twist.twist.linear.x = 0.0
            self.msg_twist.twist.linear.y = 0.0
            self.msg_twist.twist.linear.z = 0.0
            self.msg_twist.twist.angular.x = 0.0
            self.msg_twist.twist.angular.y = 0.0
            self.msg_twist.twist.angular.z = 0.0
            self.msg_twist.header.frame_id = (
                f"{self._param_robot_eef_part}__tool0"  # TODO: if changing to EEF, change ur_servo.yaml
            )
            self.msg_twist.header.stamp = self.get_clock().now().to_msg()
            self._pub_servo.publish(self.msg_twist)
        return
    


def main():
    rclpy.init()
    find_branch_roll_wrist_controller = FindBranchRollWristController()
    executor = MultiThreadedExecutor()
    rclpy.spin(find_branch_roll_wrist_controller, executor=executor)
    find_branch_roll_wrist_controller.destroy_node()
    rclpy.shutdown()
    return
