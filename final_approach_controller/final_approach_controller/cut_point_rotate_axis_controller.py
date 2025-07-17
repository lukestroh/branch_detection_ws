#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time

from final_approach_controller.tf_node import TFNode

from final_approach_controller.timer_state import TimerState
from final_approach_controller_msgs.action import RunCutPointRotateAxis
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from tof_msgs.msg import TofStamped
from vl6180_msgs.msg import Vl6180FilteredStamped

import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation
import pprint as pp
from threading import Lock



class CutPointRotateAxisController(TFNode):
    def __init__(self) -> None:
        super().__init__(node_name="cut_point_rotate_axis_controller_node")

        # Locks
        self._lock_timer_state_pub_servo = Lock()

        # Parameters
        self._param_robot_eef_part = (
            self.declare_parameter("robot_eef_part", value=Parameter.Type.STRING).get_parameter_value().string_value
        )
        self._param_robot_base_part = (
            self.declare_parameter("robot_base_part", value=Parameter.Type.STRING).get_parameter_value().string_value
        )
        self._param_tof_type = (
            self.declare_parameter("tof_sensor_type", value=Parameter.Type.STRING).get_parameter_value().string_value
        )

        # Callback group
        self.callback_group = ReentrantCallbackGroup()  # allows for subscriber to persist in service, action
        self._pub_servo_cb_group = MutuallyExclusiveCallbackGroup()

        # Actions
        self._action_svr_run_final_appoach = ActionServer(
            node=self,
            action_type=RunCutPointRotateAxis,
            action_name="run_cut_point_rotate_axis",
            goal_callback=self._action_goal_cb_run_final_approach,
            cancel_callback=self._action_cancel_cb_run_cut_point_rotate_axis,
            execute_callback=self._action_exe_cb_run_cut_point_rotate_axis,
            # handle_accepted_callback=self._action_handle_accepted_cb_run_final_approach,
            callback_group=self.callback_group,
        )

        # Service clients
        self._srv_client_start_servo = self.create_client(
            srv_type=Trigger, srv_name="/servo_node/start_servo", callback_group=self.callback_group
        )
        self._srv_client_start_servo.wait_for_service()
        self._srv_client_stop_servo = self.create_client(
            srv_type=Trigger, srv_name="/servo_node/stop_servo", callback_group=self.callback_group
        )
        self._srv_client_stop_servo.wait_for_service()

        # Subscribers
        self._sub_tof_filtered = self.create_subscription(
            msg_type=TofStamped,
            topic="/vl53l4cd/filtered",
            callback=self._sub_cb_tof_filtered,
            callback_group=self.callback_group,
            qos_profile=1,
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self._pub_servo_cb_group,
            qos_profile=1,
        )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=1.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_state_pub_servo = TimerState.STOPPED
        self._timer_pub_servo = self.create_timer(
                timer_period_sec=1 / 30, callback=self._timer_cb_run_controller, callback_group=self._pub_servo_cb_group
            )
        
        # Messages
        self.msg_twist = TwistStamped()

        # Controller attributes
        self._goal_handle = None
        self.d_tof0 = 0.0
        self.d_tof1 = 0.0

        self.max_linear_speed = 0.01 * 10  # UR servo is slow?
        self.max_angular_speed = np.pi / 4 * 10
        self.K_p = 1 / self.max_angular_speed

        self.tf_mp_base_to_tof0 = np.identity(4)
        self.tf_mp_base_to_tof1 = np.identity(4)
        self.tf_mp_cut_point_to_base = np.identity(4)
        self.tf_tof0_to_cut_point = np.identity(4)
        self.tf_tof0_to_tof1 = np.identity(4)
        self._dist_cut_point_to_branch_threshold = 0.04  # This is bad, get better sensors? How to calibrate?
        self.controller_running = False

        #
        self.tof_ranging_max = 1.6  # TODO: Get from params
        self.tof_name = self._param_tof_type

        return
    
    def stop_servo_pub_timer(self):
        with self._lock_timer_state_pub_servo:
            if self._timer_state_pub_servo == TimerState.RUNNING:
                self._timer_state_pub_servo == TimerState.STOPPED
        return
    
    def start_servo_pub_timer(self):
        with self._lock_timer_state_pub_servo:
            if self._timer_state_pub_servo == TimerState.STOPPED:
                self._timer_state_pub_servo == TimerState.RUNNING
        return

    # ===============================
    #        Action callbacks
    # ===============================
    def _action_cancel_cb_run_cut_point_rotate_axis(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        return CancelResponse.ACCEPT

    def _action_goal_cb_run_final_approach(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT

    async def _action_exe_cb_run_cut_point_rotate_axis(self, goal_handle: ServerGoalHandle):
        """TODO: This is the same as final_approach_controller, let the high level controller do this in the future"""
        self.controller_running = True
        await self.start_servo()
        
        try:
            result = RunCutPointRotateAxis.Result()

            while self.controller_running:
                if goal_handle.is_cancel_requested:
                    self.stop_servo_pub_timer()
                    goal_handle.canceled()
                    self.info("CutPointRotateAxisController canceled.")
                    result.success = False
                    return result

                # if self.get_clock().now() - self.start_servo_time > Duration(seconds=5):
                #     goal_handle.canceled()
                #     result.success = False
                #     self.controller_running = False
                #     self.error("CutPointRotateWristController timed out.")
                #     return result

            result.success = True
            self.publish_zero_twist()
            goal_handle.succeed()
            return result

        except Exception as e:
            self.fatal(f"{e}")
        finally:
            self.publish_zero_twist()
            self.stop_servo_pub_timer()
            await self.stop_servo()
        return result

    

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
        # self.warn(f"\n{mr.TransInv(self.tf_mp_cut_point_to_base)}")
        self.info(f"Received static tf frames.")
        return

    def _timer_cb_run_controller(self):
        with self._lock_timer_state_pub_servo:
            if self._timer_state_pub_servo != TimerState.RUNNING:
                return
        if np.isclose(self.d_tof0, self.tof_ranging_max, atol=0.05) or np.isclose(
            self.d_tof1, self.tof_ranging_max, atol=0.05
        ):
            self.error(
                f"{self.tof_name} sensor(s) are returning unreliable data, aborting controller. Data: {self.d_tof0}, {self.d_tof1}"
            )
            self.stop_servo_pub_timer()
            self.controller_running = False
            self.publish_zero_twist()
            return

        if (
            self.d_tof0 > 0.4 or self.d_tof1 > 0.4
        ):  # TODO: This is arbitrary, fix with better value (maybe based on max desired far plane distance?)
            self.error(
                f"{self.tof_name} sensor(s) are returning data beyond the 0.4m distance, aborting. Data: {self.d_tof0}, {self.d_tof1}"
            )

        dist, theta = self.get_cut_point_info()
        dist_cut_point_to_branch = dist - self.tf_cut_point_to_tof0[2, 3]

        if np.isclose(theta, 0.0, atol=np.radians(1)):
            self.info(f"Reached terminating point at:\ndist:{dist_cut_point_to_branch}, theta: {theta}")
            self.stop_servo_pub_timer()
            self.controller_running = False
            self.publish_zero_twist()
            return

        else:
            rot_ax = self.get_rotation_axis()
            tf_rot_axis_to_cut_point = self.get_cut_point_to_rot_axis_transform(rot_ax=rot_ax)

            twist_mp_tool0_frame = self.get_twist(
                tf_rot_axis_to_cut_point=tf_rot_axis_to_cut_point, angle_from_perpendicular=theta
            )

            linear_v_mp_tool0_frame = twist_mp_tool0_frame[0:3, 0]
            linear_v_mp_tool0_frame *= self.max_linear_speed

            angular_v_mp_tool0_frame = twist_mp_tool0_frame[3:6, 0]
            angular_v_mp_tool0_frame *= self.max_angular_speed

            self.msg_twist.twist.linear.x = linear_v_mp_tool0_frame[0]
            self.msg_twist.twist.linear.y = linear_v_mp_tool0_frame[1]
            self.msg_twist.twist.linear.z = linear_v_mp_tool0_frame[2]
            self.msg_twist.twist.angular.x = angular_v_mp_tool0_frame[0]
            self.msg_twist.twist.angular.y = angular_v_mp_tool0_frame[1]
            self.msg_twist.twist.angular.z = angular_v_mp_tool0_frame[2]
            self.msg_twist.header.frame_id = f"{self._param_robot_eef_part}__tool0"
            self.msg_twist.header.stamp = self.get_clock().now().to_msg()
            self._pub_servo.publish(self.msg_twist)
        return

    # ===============================
    #     Subscription callbacks
    # ===============================
    def _sub_cb_tof_filtered(self, msg: TofStamped):
        if msg.dev_id == 0:
            self.d_tof0 = msg.data[0]
        elif msg.dev_id == 1:
            self.d_tof1 = msg.data[0]
        return

    # ===============================
    #       Controller methods
    # ===============================
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
    
    def get_cut_point_info(self) -> tuple:
        dist = (self.d_tof0 + self.d_tof1) / 2
        d_diff = self.d_tof0 - self.d_tof1
        theta = np.arctan(d_diff / self._tof_linear_distance)  # should return angle (-pi/2, pi/2)
        return dist, theta

    def get_rotation_axis(self):
        """Get the rotation axis in the camera frame."""
        rotation_point = np.mean([self.d_tof0, self.d_tof1], axis=0)
        # log.debug(f"Rotation point: {rotation_point}")
        rotation_axis = np.zeros((6, 1), dtype=float)
        # TODO: fix with proper vector. for now, just use z-axis
        rotation_axis[0:3, :] = np.array([[0, 0, rotation_point]]).T
        # rotation_axis[0:3, :] = rotation_point[0, :3].reshape(3, 1)
        rotation_axis[3:6, :] = np.cross([0, 0, self.d_tof0], [0, 0, self.d_tof1]).reshape(
            3, 1
        )  # TODO: clean up hackiness here
        # If the cross product is zero (for current scenario, should be true) then the two vectors are parallel, so we can just choose the  y-axis (camera frame).
        if np.linalg.norm(rotation_axis[3:6, :]) != 0:
            rotation_axis[3:6, :] = rotation_axis[3:6, :] / np.linalg.norm(rotation_axis[3:6, :])
        else:
            rotation_axis[3:6, :] = np.array([[0, 1, 0]]).T
        return rotation_axis

    def get_cut_point_to_rot_axis_transform(self, rot_ax: np.ndarray):
        """TODO: replace with actual transform from end effector to cut point."""
        tf_axis_to_eef = np.identity(4)
        tf_axis_to_eef[:3, 3] = rot_ax[:3, 0]
        tf_cut_point_to_rot_axis = mr.TransInv(self.tf_mp_cut_point_to_base) @ tf_axis_to_eef
        # log.warn(self.tf_base_to_cut_point)
        # log.warn(tf_axis_to_eef)
        # log.warn(tf_cut_point_to_rot_axis)
        return tf_cut_point_to_rot_axis

    def get_twist(self, tf_rot_axis_to_cut_point: np.ndarray, angle_from_perpendicular: float):
        # log.error(f"Angle from perpendicular: {angle_from_perpendicular * 180 / np.pi}")
        desired_angle = 0.0  # We want the cut point to be perpendicular to the rotation axis
        # We are in the mp base frame, so the angular velocity is along the y-axis, which points down
        angular_velocity = [0, self.K_p * (desired_angle - angle_from_perpendicular), 0]
        linear_velocity = np.cross(angular_velocity, tf_rot_axis_to_cut_point[:3, 3])

        twist = np.concatenate((linear_velocity, angular_velocity), axis=0).reshape(6, 1)
        return twist

    def publish_zero_twist(self, servo_frame="mock_pruner__tool0"):
        self.msg_twist.twist.linear.x = 0.0
        self.msg_twist.twist.linear.y = 0.0
        self.msg_twist.twist.linear.z = 0.0
        self.msg_twist.twist.angular.x = 0.0
        self.msg_twist.twist.angular.y = 0.0
        self.msg_twist.twist.angular.z = 0.0
        self.msg_twist.header.frame_id = servo_frame  # TODO: if changing to EEF, change ur_servo.yaml
        self.msg_twist.header.stamp = self.get_clock().now().to_msg()
        self._pub_servo.publish(self.msg_twist)
        return

    # def reset_controller(self):


def main():
    rclpy.init()
    cut_point_rotate_axis_controller = CutPointRotateAxisController()
    executor = MultiThreadedExecutor()
    rclpy.spin(cut_point_rotate_axis_controller, executor=executor)
    cut_point_rotate_axis_controller.destroy_node()
    rclpy.shutdown()

    return


if __name__ == "__main__":
    main()
