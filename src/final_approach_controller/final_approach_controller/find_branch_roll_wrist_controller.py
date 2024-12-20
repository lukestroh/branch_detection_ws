#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from final_approach_controller_msgs.action import RunFindBranchRollWrist
from final_approach_controller.tf_node import TFNode
from vl6180_msgs.msg import Vl6180FilteredStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation
import pprint as pp
from collections import deque
import time

  
class FindBranchRollWristController(TFNode):
    def __init__(self):
        super().__init__(node_name="find_branch_roll_wrist_controller", cache_time=Duration(seconds=10))
        self.info = lambda x: self.get_logger().info(f"\n{pp.pformat(x)}")
        self.warn = lambda x: self.get_logger().warn(f"\n{pp.pformat(x)}")
        self.error = lambda x: self.get_logger().error(f"\n{pp.pformat(x)}")

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # Actions
        self._action_svr_run_find_branch_roll_wrist = ActionServer(
            node=self,
            action_type=RunFindBranchRollWrist,
            action_name='run_find_branch_roll_wrist',
            goal_callback=self._action_goal_cb_run_find_branch_roll_wrist,
            cancel_callback=self._action_cancel_cb_run_find_branch_roll_wrist,
            execute_callback=self._action_exe_cb_run_find_branch_roll_wrist,
            callback_group=self.callback_group
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
            topic='joint_states',
            callback=self._sub_cb_joint_states,
            callback_group=self.callback_group,
            qos_profile=1
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self.callback_group,
            qos_profile=1,
        )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=3.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_run_controller = None
        self._timer_run_quadratic_fit = None
        self._timer_debug = self.create_timer(timer_period_sec=1.0, callback=self._timer_cb_debug)

        # Messages
        self.msg_twist = TwistStamped()

        # Controller attributes
        self.max_angular_vel = 0.0
        self.d_tof0_readings = []
        self.d_tof1_readings = []
        self.start_time = self.get_clock().now()
        self.feedback_pub_prev_time = self.get_clock().now()
        self._goal_handle: ServerGoalHandle | None = None
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
        self.error(f"{goal_handle.status}")
        if self._timer_run_controller is None:
            self._timer_run_controller = self.create_timer(
                timer_period_sec=1 / 30, callback=self._timer_cb_run_controller, callback_group=self.callback_group
            )
            self._timer_run_quadradic_fit = self.create_timer(
                timer_period_sec=1.0, callback=self._timer_cb_run_quadradic_fit, callback_group=self.callback_group
            )
        else:
            self._timer_run_controller.reset()
            self._timer_run_quadradic_fit.reset()

        try:
            feedback_msg = RunFindBranchRollWrist.Feedback()
            result = RunFindBranchRollWrist.Result()

            while self.controller_running:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.info("FindBranchRollWristController canceled.")
                    result.success = False
                    return result
                
                if self.get_clock().now() - self.feedback_pub_prev_time > 1.0:
                    feedback_msg.tof0 = self.d_tof0
                    feedback_msg.tof0 = self.d_tof1
                    goal_handle.publish_feedback(feedback=feedback_msg)
                
                
        except Exception as e:
            self.get_logger().fatal(f'{e}')
        finally:
            self._timer_run_controller.cancel()
        return

    def _action_goal_cb_run_find_branch_roll_wrist(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT
    
    # ===============================
    #         Timer callbacks
    # ===============================
    
    def _timer_cb_debug(self):        
        # tf_old = self.lookup_transform(
        #     target_frame="cart__base",
        #     source_frame="mock_pruner__tool0",
        #     sync=True,
        #     time=self.start_time,
        #     timeout=Duration(seconds=1),
        #     as_matrix=True
        # )
        # self.info(tf_old)
        # tf_new = self.lookup_transform(
        #     target_frame="cart__base",
        #     source_frame="mock_pruner__tool0",
        #     sync=True,
        #     time=self.get_clock().now(),
        #     timeout=Duration(seconds=1),
        #     as_matrix=True
        # )
        # self.warn(tf_new)
        return
    
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
        self.d_tof0_readings.append()
        if not np.isclose(self.d_tof0, 255.0, atol=15.0) and not np.isclose(self.d_tof1, 255.0, atol=15.0):
            self.publish_zero_twist()
            self.info("Both sensors detecting an object, stopping controller")
            self._goal_handle.is_cancel_requested = True
            # TODO: NEED SEPARATE CONDITION FOR FINDING A BRANCH!!!!
            return

        if np.isclose(self.d_tof0, 255.0, atol=5.0) and np.isclose(self.d_tof1, 255.0, atol=5.0):
            self.info("Unable to see branch with either sensor.")

        

        # periodically run fit, (maybe in another timer?), if the fit has good value (???), then pick that zero point, get the time, save for which sensor
        # if len(self.d_tof0_readings < ...):
            # don't run
        # OR
        # if np.isclose()...

        # if both have a fit, publish zero message, do pose math, call service, kill timer, controller_running = False



        # rotate to the closest side
        if self.joint_states[-1] < 0 and self.joint_states > -1 * np.pi:
            # negative angular rotation
            angular_z = -1 * self.max_angular_vel
            ...
        if self.joint_states[-1] > 0 and self.joint_states < np.pi:
            # positive angular rotation
            angular_z = self.max_angular_vel
            ...
        
        self.msg_twist.twist.linear.x = 0.0
        self.msg_twist.twist.linear.y = 0.0
        self.msg_twist.twist.linear.z = 0.0
        self.msg_twist.twist.angular.x = 0.0
        self.msg_twist.twist.angular.y = 0.0
        self.msg_twist.twist.angular.z = angular_z
        self.msg_twist.header.frame_id = "cart__base" # TODO: if changing to EEF, change ur_servo.yaml
        self.msg_twist.header.stamp = self.get_clock().now().to_msg()
        self._pub_servo.publish(self.msg_twist)

        return
    
    def _timer_cb_run_quadradic_fit(self):

        return
            
    
    # ===============================
    #     Subscription callbacks
    # ===============================

    def _sub_cb_tof_filtered(self, msg: Vl6180FilteredStamped):
        # Do some checks, make sure that the readings make sense in intuitive way.
        # Make sure readings do not exceed maximum. # TODO: Find a way to get sensor parameters in here
        self.d_tof0 = msg.data[0] / 1000  # mm to m
        self.d_tof1 = msg.data[1] / 1000
        # self.warn(self.d_tof0)
        return    
    
    def _sub_cb_joint_states(self, msg: JointState):
        self.joint_states = msg.position
        # self.warn(joint_states)
        return
    
    def publish_zero_twist(self):
        self.msg_twist.twist.linear.x = 0.0
        self.msg_twist.twist.linear.y = 0.0
        self.msg_twist.twist.linear.z = 0.0
        self.msg_twist.twist.angular.x = 0.0
        self.msg_twist.twist.angular.y = 0.0
        self.msg_twist.twist.angular.z = 0.0
        self.msg_twist.header.frame_id = "cart__base" # TODO: if changing to EEF, change ur_servo.yaml
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


    
