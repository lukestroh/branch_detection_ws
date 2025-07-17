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

from branch_detection_system_moveit_msgs.srv import MoveToPose
from final_approach_controller.tf_node import TFNode


class AlignToBranchController(TFNode):
    def __init__(self):
        super().__init__(node_name="align_to_branch_controller", cache_time=Duration(seconds=30))
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

move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(request=move_to_pose_req)
move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
await move_group_future
# rclpy.spin_until_future_complete(node=self, future=move_group_future)

# Wait a second for moving average filter to settle.
self.get_clock().sleep_for(Duration(seconds=1.0))

if move_group_future.result() is None or not move_group_future.result().result:
    goal_handle.abort()
    result.success = False
else:
    if self.d_tof0 < self.tof_far_plane and self.d_tof1 < self.tof_far_plane:
        goal_handle.succeed()
        result.success = True
    else:
        goal_handle.abort()
        result.success = False

        self.error(
            f"Failed to navigate to pose where both sensors can read the branch.\nd_tof0: {self.d_tof0}, d_tof1: {self.d_tof1}, far_plane: {self.tof_far_plane}"
        )

self.controller_running = False
await self.switch_controllers(
    activate_controllers=self._servo_controller,
    deactivate_controllers=self._move_group_controller,
)
