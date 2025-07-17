#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import WrenchStamped, TwistStamped
from std_srvs.srv import Trigger

import numpy as np


class AdmittanceControllerNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name="admittance_controller_node")

        self.callback_group = ReentrantCallbackGroup()

        # Action servers

        # Services
        self._srv_start_admittance_controller = self.create_service(
            srv_name="start_admittance_controller",
            srv_type=Trigger,
            callback=self._srv_cb_start_admittance_controller,
            callback_group=self.callback_group,
        )
        self._srv_stop_admittance_controller = self.create_service(
            srv_name="stop_admittance_controller",
            srv_type=Trigger,
            callback=self._srv_cb_stop_admittance_controller,
            callback_group=self.callback_group,
        )

        # Subscribers
        self._sub_wrench_filtered = self.create_subscription(
            msg_type=WrenchStamped,
            topic="/force_torque_sensor_broadcaster/wrench_filtered",
            callback=self._sub_cb_wrench_filtered,
            callback_group=self.callback_group,
            qos_profile=1,
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self.callback_group,
            qos_profile=5,
        )

        # Class attributes
        self.controller_running = False
        self.desired_wrench = np.array([0, 0, 0, 0, 0, -2])  # TODO: Put a label on this?
        self.max_velocity = 0.01  # 1 cm/s
        # self.global_done = False

        # Controller gains
        self.Kf = np.diag([0, 0, 0, 0, 0.01, 0.1], dtype=float)
        self.Kd = np.diag([0, 0, 0, 0, 400, 250], dtype=float)

        # Selection matrix
        self.selection_mat = np.diag([1, 0, 0, 0, 1, 1])

        # Deadband
        self.f_thresh = 0.2  # N, will ignore anything < 0.2N
        self.stop_force_thresh = 0.15
        self.stop_torque_thesh = 0.01
        self.last_dirs = ["stopped", "stopped"]
        self.last_stop_condition = False

        self.publish_freq = 500.0

        self.prev_z_vels = np.zeros(1000)
        self.prev_y_vels = np.zeros(1000)
        self.z_travel_amts = np.ones(10)
        self.y_travel_amts = np.ones(10)

        # End condition vars
        self.last_goal_checks = np.zeros(10)

        return

    def _sub_cb_wrench_filtered(self, msg: WrenchStamped) -> None:
        """I just translated this code, I don't know why we're doing W=[torque, force]"""
        w = msg.wrench
        wrench_vec = np.array([w.torque.x, w.torque.y, w.torque.z, w.force.x, w.force.y, w.force.z])

        acc_des_force_term = np.dot(-self.Kf, np.dot(self.l, self.deadzone(self.des_wrench - wrench_vec)))
        acc_des_damp_term = np.dot(-self.Kf, np.dot(self.Kd, self.vel_prev))

        # New controller -- use the acceleration and the publish rate to update the velocity
        vel_des = self.vel_prev + (1 / self.publish_freq) * (acc_des_force_term + acc_des_damp_term)

        vel_y_limited = self.impose_vel_limit(vel_des[4])
        vel_z_limited = self.impose_vel_limit(vel_des[5])


def main():
    rclpy.init()
    admittance_controller_node = AdmittanceControllerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(admittance_controller_node, executor=executor)
    admittance_controller_node.destroy_node()
    rclpy.shutdown()
    return
