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

import final_approach_controller.plotly_helpers as ph

from branch_detection_system_moveit_msgs.srv import MoveToPose
from final_approach_controller_msgs.action import RunFindBranchRollWrist
from final_approach_controller_msgs.msg import ToFBranchFitStamped
import final_approach_controller.curve_fitting as cf
from final_approach_controller.tf_node import TFNode
from vl6180_msgs.msg import Vl6180FilteredStamped

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped, Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup

# from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    RobotState,
    MotionPlanRequest,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    Constraints,
    PlanningOptions,
)
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
import copy




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

        self._srv_cartesian_move_to_pose = self.create_client(
            srv_type=MoveToPose, srv_name="/cartesian_move_to_pose", callback_group=self._reentrant_cb_group
        )

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
        self._sub_tof_filtered = self.create_subscription(
            msg_type=Vl6180FilteredStamped,
            topic="/vl6180/filtered",
            callback=self._sub_cb_tof_filtered,
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
            qos_profile=1,
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
        self.max_angular_vel = np.pi / 16
        self.debug_plot = True
        self.eef_weight = 0.355  # TODO: measure again. Measured IRL

        # Sensor attributes
        self.vl6180_far_plane = 0.200  # 0.19 based on testing, but give it small window. TODO: Get from param file
        self.vl6180_precision = 0.001
        
        self.d_tof0 = 0.255
        self.d_tof1 = 0.255
        
        return

    # ===============================
    #        Action callbacks
    # ===============================
    def _action_cancel_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        self.info("Canceling quadratic fit timer")
        with self._timer_lock:
            self._timer_pub_servo.cancel()
            self._timer_run_quadratic_fit.cancel()
        return CancelResponse.ACCEPT

    def _action_exe_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.controller_running = True
        self._srv_client_start_servo.call(request=Trigger.Request())

        self.start_controller_tf = self.lookup_transform(
            target_frame="cart__base", source_frame="mock_pruner__tool0", sync=True, as_matrix=True
        )
        with self._timer_lock:
            if self._timer_run_quadratic_fit is None:
                self._timer_run_quadratic_fit = self.create_timer(
                    timer_period_sec=3.0,
                    callback=self._timer_cb_run_quadratic_fit,
                    callback_group=self._parabola_fitting_cb_group,
                )
            else:
                self._timer_run_quadratic_fit.reset()
            if self._timer_pub_servo is None:
                self._timer_pub_servo = self.create_timer(
                    timer_period_sec=1/250,
                    callback=self._timer_cb_pub_servo,
                    callback_group=self._pub_servo_cb_group
                )
            else:
                self._timer_pub_servo.reset()

        try:
            feedback_msg = RunFindBranchRollWrist.Feedback()
            result = RunFindBranchRollWrist.Result()

            # TODO: check if initial reading of sensor. If so, set flag to data found and record tf pose time.

            while self.controller_running:
                # self.debug_counter += 1

                if goal_handle.status == GoalStatus.STATUS_CANCELED:
                    # self._timer_run_controller.cancel()
                    with self._timer_lock:
                        if self._timer_run_quadratic_fit is not None:
                            self._timer_run_quadratic_fit.cancel()
                        self._timer_pub_servo.cancel()
                    result.success = False
                    return result

                if goal_handle.status == GoalStatus.STATUS_ABORTED:
                    with self._timer_lock:
                        if self._timer_run_quadratic_fit is not None:
                            self._timer_run_quadratic_fit.cancel()
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
                        self.info("hello world")
                        tof0_branch_found = self.tof0_branch_found
                        tof1_branch_found = self.tof1_branch_found
                        # self.debug_counter -= 1

                    if tof0_branch_found and tof1_branch_found:
                        # if self.debug_counter:
                        #     self.error(f'debug counter: {self.debug_counter}')
                        self.neg_rot_complete = True
                        self.pos_rot_complete = True
                        self.rotations_complete = True
                        self.info(f"tof0: {tof0_branch_found}, tof1: {tof1_branch_found}")
                        with self._timer_lock:
                            if self._timer_run_quadratic_fit is not None:
                                self._timer_run_quadratic_fit.cancel()
                        self.publish_zero_twist()
                        self.info("Branch readings found for both ToFs!")

                    if not self.rotations_complete:
                        if not self.neg_rot_complete:
                            # # rotate to the closest side
                            # if self.joint_states[-1] < 0 and self.joint_states[-1] > -1 * np.pi:
                            # negative angular rotation
                            angular_z = -1 * self.max_angular_vel
                            if np.isclose(self.joint_states[2], -np.pi / 2, atol=0.05):
                                # TODO: (long term) make sure wrist mount config is standard
                                self.neg_rot_complete = True
                                self.publish_zero_twist()

                        elif not self.pos_rot_complete:
                            # if self.joint_states[-1] > 0 and self.joint_states[-1] < np.pi:
                            # positive angular rotation
                            angular_z = self.max_angular_vel
                            if np.isclose(self.joint_states[2], np.pi / 2, atol=0.05):
                                self.pos_rot_complete = True
                                self.publish_zero_twist()

                        with self._servo_msg_lock:
                            self.msg_twist.twist.linear.x = 0.0
                            self.msg_twist.twist.linear.y = 0.0
                            self.msg_twist.twist.linear.z = 0.0
                            self.msg_twist.twist.angular.x = 0.0
                            self.msg_twist.twist.angular.y = 0.0
                            self.msg_twist.twist.angular.z = angular_z
                            self.msg_twist.header.frame_id = "mock_pruner__tool0"  # TODO: Get name dynamically
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
                                with self._timer_lock:
                                    if self._timer_run_quadratic_fit is not None:
                                        self._timer_run_quadratic_fit.cancel()
                                goal_handle.abort()
                                # self.reset_controller() # Done in 'finally'

                                result.success = False
                            return result

                        else:
                            # Stop servo
                            self.publish_zero_twist() # Just in case
                            with self._timer_lock:
                                self._timer_pub_servo.cancel()
                            stop_servo_response: Trigger.Response = self._srv_client_stop_servo.call(request=Trigger.Request())
                            if not stop_servo_response.success:
                                raise Exception("Failed to stop servo.")

                            # Switch controllers
                            switch_ctrlr_req = SwitchController.Request(
                                activate_controllers=[self._move_group_controller],
                                deactivate_controllers=[self._servo_controller],
                                strictness=SwitchController.Request.STRICT,
                                # timeout=Duration(seconds=2)
                            )
                            self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)

                            # If the eef is moving, we need a common frame, which should be world or cart__base
                            # Get tof poses at calculated signal minimum times
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
                            # Get the current eef pose
                            tf_cut_point_to_base = self.lookup_transform(
                                target_frame="cart__base",
                                source_frame="mock_pruner__tool0",
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
                            self.warn(tof0_vec_base_frame)

                            # Get the centerpoint of these two points.
                            branch_center_point = np.mean([tof0_vec_base_frame, tof1_vec_base_frame], axis=0)  # C
                            # self.warn(f"CENTER:\n{branch_center_point}")

                            # Get closest point on a circle from point, given circle center,
                            # point, plane normal
                            # https://www.geometrictools.com/Documentation/DistanceToCircle3.pdf
                            world_z = [0, 0, 1]

                            branch_vec = tof0_vec_base_frame - tof1_vec_base_frame
                            branch_vec_normalized = branch_vec / np.linalg.norm(branch_vec)  # N

                            curr_pose = tf_cut_point_to_base[0:3, 3]  # P

                            delta = curr_pose - branch_center_point
                            _Q_C = delta - np.dot(branch_vec_normalized, delta) * branch_vec_normalized

                            desired_radius_from_branch = 0.1  # m

                            desired_eef_xyz = (
                                branch_center_point + _Q_C / np.linalg.norm(_Q_C) * desired_radius_from_branch
                            )

                            desired_orientation_vec_to_branch = branch_center_point - desired_eef_xyz

                            desired_orientation_vec_to_branch_norm = desired_orientation_vec_to_branch / np.linalg.norm(desired_orientation_vec_to_branch)

                            # CREATE BASIS VECTORS: we need to create a coordinate basis with respect to the calculated vector. Use a temp vec, like world_z. Mimicking the 'camera' frame, this would give us the x-axis, which we want pointing to the right. Therefore, we should take desired x world_z
                            desired_x_axis = np.cross(desired_orientation_vec_to_branch_norm, world_z)
                            desired_x_axis = desired_x_axis / np.linalg.norm(desired_x_axis) # this SHOULD be 1 already....
                            desired_y_axis = np.cross(desired_orientation_vec_to_branch_norm, desired_x_axis)
                            desired_y_axis = desired_y_axis / np.linalg.norm(desired_y_axis)

                            # Form the rotation matrix from our basis vectors
                            rot_mat = np.column_stack((desired_x_axis, desired_y_axis, desired_orientation_vec_to_branch_norm))
                            desired_orientation_rot = Rotation.from_matrix(matrix=rot_mat)
                            desired_orientation_quat = desired_orientation_rot.as_quat()

                            if self.debug_plot:
                                fig = go.Figure()
                                fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], name="cart__base"))
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
                                ph.plot_vector(fig=fig, position=desired_eef_xyz, orientation=desired_orientation_vec_to_branch_norm, scale=0.5)
                                fig.update_layout(scene=dict(aspectmode="data"))

                                fig.show()

                            #####################################################
                            move_to_pose_req = MoveToPose.Request()
                            move_to_pose_req.goal.position.x = desired_eef_xyz[0]
                            move_to_pose_req.goal.position.y = desired_eef_xyz[1]
                            move_to_pose_req.goal.position.z = desired_eef_xyz[2]
                            move_to_pose_req.goal.orientation.x = desired_orientation_quat[0]
                            move_to_pose_req.goal.orientation.y = desired_orientation_quat[1]
                            move_to_pose_req.goal.orientation.z = desired_orientation_quat[2]
                            move_to_pose_req.goal.orientation.w = desired_orientation_quat[3]
                            
                            move_group_response: MoveToPose.Response = self._srv_cartesian_move_to_pose.call(request=move_to_pose_req)
                            # future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
                            # response = await future

                            if move_group_response.result:
                                goal_handle.succeed()
                                result.success = True
                            else:
                                goal_handle.abort()
                                result.success = False
                                
                            
                            ####################################################################################
                            # future.add_done_callback(self._action_client_move_group_done_cb)
                            # get angle between the two to determine direction
                            # np.dot()
                            # self.warn(dir_vec)
                            # TODO: need initial edge case where a ToF is reading at the start of the controller so that we save it's position and don't need to fit (also avoid poor parabola fit)

                            # self.warn(branch_center_point)

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
            with self._timer_lock:
                if self._timer_run_quadratic_fit is not None:
                    self._timer_run_quadratic_fit.cancel()

            self.reset_controller()
            self.info("FindBranchRollWristController has terminated.")
        return result

    def _action_goal_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        # self.goal_handle_aborted = False
        return GoalResponse.ACCEPT

    # def _action_client_execute_trajectory_done_cb(self, future: Future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.error("Trajectory goal was rejected")
    #         return
    #     else:
    #         ...
    #     return

    # ===============================
    #        Service callbacks
    # ===============================

    # def _srv_client_get_cartesian_path_done_cb(self, future: Future):
    #     response: GetCartesianPath.Response = future.result()

    #     if response.fraction == 1.0:
    #         self.execute_trajectory(trajectory=response.solution.joint_trajectory)
    #     else:
    #         self.error(f"Could not find a complete Cartesian path to goal pose. ({response.fraction * 100.0}% computed")

    #     return

    # ===============================
    #        Future callbacks
    # ===============================

    def _done_cb_srv_cartesian_move_to_pose(self, future: Future):
        goal_handle: MoveToPose.Response = future.result()
        # self.
        self.info(f"Cartesian move to pose result: {goal_handle.result}.")
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
        self.info("Static TF frames acquired.")
        return

    def _timer_cb_pub_servo(self):
        with self._servo_msg_lock:
            self._pub_servo.publish(self.msg_twist)
        return

    def _timer_cb_run_controller(self):
        return

    def _timer_cb_run_quadratic_fit(self):
        """Periodically run a fit on the data"""
        with self._branch_found_lock:
            tof0_branch_found = self.tof0_branch_found

        if not tof0_branch_found:
            # Make a copy of the recorded data so that the subscriber may continue to append
            with self._data_lock:
                timestamp_readings_tof0_copy = list(self.timestamp_readings_tof0)
                d_tof0_readings_copy = list(self.d_tof0_readings)
            tof0_time_and_dist = self.get_branch_center_time_and_distance(
                timestamps=timestamp_readings_tof0_copy,
                readings=d_tof0_readings_copy,
                sensor_name="tof0",
                debug_plot=self.debug_plot,
            )
            if tof0_time_and_dist is not None:
                self.tof0_time_center, self.tof0_distance_center = tof0_time_and_dist
                with self._branch_found_lock:
                    self.tof0_branch_found = True

        with self._branch_found_lock:
            tof1_branch_found = self.tof1_branch_found
        if not tof1_branch_found:
            with self._data_lock:
                timestamp_readings_tof1_copy = list(self.timestamp_readings_tof1)
                d_tof1_readings_copy = list(self.d_tof1_readings)
            tof1_time_and_dist = self.get_branch_center_time_and_distance(
                timestamps=timestamp_readings_tof1_copy,
                readings=d_tof1_readings_copy,
                sensor_name="tof1",
                debug_plot=self.debug_plot,
            )
            if tof1_time_and_dist is not None:
                self.tof1_time_center, self.tof1_distance_center = tof1_time_and_dist
                with self._branch_found_lock:
                    self.tof1_branch_found = True
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
            timestamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            # with self._branch_found_lock:
            with self._data_lock:
                if not self.tof0_branch_found:
                    self.d_tof0_readings.append(self.d_tof0)
                    self.timestamp_readings_tof0.append(timestamp_float)
                if not self.tof1_branch_found:
                    self.d_tof1_readings.append(self.d_tof1)
                    self.timestamp_readings_tof1.append(timestamp_float)

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
        with self._branch_found_lock:
            self.tof0_branch_found = False
            self.tof1_branch_found = False
        self.tof0_time_center = None
        self.tof1_time_center = None
        self.tof0_distance_center = None
        self.tof1_distance_center = None
        self.start_controller_tf = np.identity(4) # TODO: unused.
        with self._data_lock:
            self.d_tof0_readings = []
            self.d_tof1_readings = []
            self.d_tof0_readings_filtered = []
            self.d_tof1_readings_filtered = []
            self.timestamp_readings_tof0 = []
            self.timestamp_readings_tof1 = []
            self.timestamps_tof0_filtered = []
            self.timestamps_tof1_filtered = []

        with self._timer_lock:
            if self._timer_run_quadratic_fit is not None:
                self._timer_run_quadratic_fit.cancel()
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
            self.msg_twist.header.frame_id = "mock_pruner__tool0"  # TODO: if changing to EEF, change ur_servo.yaml
            self.msg_twist.header.stamp = self.get_clock().now().to_msg()
            self._pub_servo.publish(self.msg_twist)
        return

    # def execute_trajectory(self, trajectory):
    #     traj_goal = ExecuteTrajectory.Goal()
    #     traj_goal.trajectory = trajectory

    #     execute_traj_future = self._action_client_execute_trajectory.send_goal_async(
    #         goal=traj_goal,
    #     )
    #     execute_traj_future.add_done_callback(callback=self._action_client_execute_trajectory_done_cb)

    #     return

    def get_branch_center_time_and_distance(
        self, timestamps, readings, sensor_name: str, debug_plot: bool = False
    ) -> Time | None:
        try:
            readings_filtered = np.where(np.asarray(readings) < self.vl6180_far_plane, readings, np.nan)
            timestamps_filtered = np.where(np.isnan(readings_filtered), np.nan, np.asarray(timestamps))
            readings_filtered = readings_filtered[~np.isnan(readings_filtered)]
            timestamps_filtered = timestamps_filtered[~np.isnan(timestamps_filtered)]
            normalized_timestamps_filtered = timestamps_filtered - timestamps_filtered[0]
        except IndexError:
            self.info(f"No branch found for {sensor_name}")
            return None
        except ValueError as e:
            self.warn(traceback.format_exc())
            self.info(f"{e}: Sensor {sensor_name} did not find branch")
            self.error(f"readings: {np.asarray(readings).shape}")
            self.error(f"timestamps: {np.asarray(readings).shape}")
            self.error(f"readings filtered: {np.asarray(readings_filtered).shape}")
            # self.error(len(timestamps_filtered))
            # self.error(len(normalized_timestamps_filtered))
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

        if time_center:
            self.info(f"Branch found at distance {fit_min} at time {timestamp_min} for {sensor_name}")

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
