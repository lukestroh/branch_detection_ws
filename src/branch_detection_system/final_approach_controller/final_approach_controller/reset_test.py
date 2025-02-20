#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.qos import QoSProfile

from branch_detection_system_moveit_msgs.srv import MoveToPose
from final_approach_controller_msgs.action import RunTestReset
from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController

from final_approach_controller.tf_node import TFNode
import modern_robotics as mr
import numpy as np
from threading import Lock
import traceback


class ResetTestNode(TFNode):
    def __init__(self):
        super().__init__(node_name="reset_test_node")

        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Threading locks
        self._timer_lock = Lock()
        self._servo_msg_lock = Lock()

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

        # Callback groups
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Action servers
        self._action_srv_generate_poses_from_current_pose = ActionServer(
            node=self,
            action_name='/run_test_reset',
            action_type=RunTestReset,
            goal_callback=self._action_goal_cb_run_test_reset,
            cancel_callback=self._action_cancel_cb_run_test_reset,
            execute_callback=self._action_execute_cb_run_test_reset,
            callback_group=self._reentrant_cb_group
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

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self._reentrant_cb_group,
            qos_profile=QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=10
            ),
        )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=3.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_pub_servo = None

        # Messages
        self.msg_twist = TwistStamped()


        # Class params
        if _param_use_mock_hardware:
            self.max_linear_speed = 0.01
        else:
            self.max_linear_speed = 0.01 * 10

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
    

    # ===============================
    #        Action callbacks
    # ===============================
    def _action_cancel_cb_run_test_reset(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        self.info("Canceling quadratic fit timer")
        self.publish_zero_twist()
        with self._timer_lock:
            if not self._timer_pub_servo.is_canceled():
                self._timer_pub_servo.cancel()
            # if not self._timer_run_quadratic_fit.is_canceled():
            #     self._timer_run_quadratic_fit.cancel()
        goal_handle.canceled()
        self.reset_controller()
        return CancelResponse.ACCEPT
    
    def _action_goal_cb_run_test_reset(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT
    
    async def _action_execute_cb_run_test_reset(self, goal_handle: ServerGoalHandle):

        run_test_reset_req: RunTestReset.Goal = goal_handle.request

        run_test_reset_result = RunTestReset.Result()

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
            if run_test_reset_req.pose_idx != 0:
                await self.start_servo()
                start_servoing_time = self.get_clock().now()

                while self.get_clock().now() - start_servoing_time < Duration(seconds=10.0):
                    self.msg_twist.twist.linear.x = 0.0
                    self.msg_twist.twist.linear.y = 0.0
                    self.msg_twist.twist.linear.z = -1 * self.max_linear_speed
                    self.msg_twist.twist.angular.x = 0.0
                    self.msg_twist.twist.angular.y = 0.0
                    self.msg_twist.twist.angular.z = 0.0
                    self.msg_twist.header.frame_id = f"{self._param_robot_eef_part}__tool0"
                    self.msg_twist.header.stamp = self.get_clock().now().to_msg()

                self.get_clock().sleep_for(Duration(seconds=1.0))
                self.publish_zero_twist()
                with self._timer_lock:
                    if not self._timer_pub_servo.is_canceled:
                        self._timer_pub_servo.cancel()
                await self.stop_servo()

            # Move to new pose
            await self.switch_controllers(
                activate_controllers=self._move_group_controller,
                deactivate_controllers=self._servo_controller
            )
            
            self.info("Sending goal")

            move_group_req = MoveToPose.Request()
            move_group_req.goal = run_test_reset_req.pose
            move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(
                request=move_group_req
            )
            move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
            await move_group_future

            await self.switch_controllers(
                activate_controllers=self._servo_controller,
                deactivate_controllers=self._move_group_controller
            )

            self.info("New pose set")

            run_test_reset_result.success = True
            goal_handle.succeed()
            return run_test_reset_result

        except Exception as e:
            run_test_reset_result.success = False
            goal_handle.abort()
            with self._timer_lock:
                if not self._timer_pub_servo.is_canceled:
                    self._timer_pub_servo.cancel()
            self.fatal(traceback.format_exc())

        finally:
            if run_test_reset_req.pose_idx != 0:
                await self.switch_controllers(activate_controllers=self._servo_controller, deactivate_controllers=self._move_group_controller)
            with self._timer_lock:
                if not self._timer_pub_servo.is_canceled:
                    self._timer_pub_servo.cancel()
            
            
        return run_test_reset_result
    
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
                    time=self.get_clock().now(),
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
            # self.warn("SERVOING BACK")

            self._pub_servo.publish(self.msg_twist)
        return
    

def main():
    rclpy.init()
    reset_test_node = ResetTestNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(reset_test_node, executor=executor)
    reset_test_node.destroy_node()
    rclpy.shutdown()

    return

