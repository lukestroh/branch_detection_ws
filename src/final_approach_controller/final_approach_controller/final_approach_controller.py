#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

# from rclpy.node import Node

from final_approach_controller.tf_node import TFNode

from geometry_msgs.msg import TwistStamped
from vl6180_msgs.msg import Vl6180FilteredStamped
from final_approach_controller_msgs.srv import StartFinalApproach

import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation
import pprint as pp


class FinalApproachControllerNode(TFNode):
    def __init__(self):
        super().__init__(node_name="final_approach_controller_node")
        self.info = lambda x: self.get_logger().info(f"{pp.pformat(x)}")
        self.warn = lambda x: self.get_logger().warn(f"{pp.pformat(x)}")
        self.error = lambda x: self.get_logger().error(f"{pp.pformat(x)}")

        self.create_timer(timer_period_sec=0.1, callback=self.print_stuff)

        # Callback group
        self.callback_group = ReentrantCallbackGroup()  # allows for subscriber to persist in service

        # Services
        self._srv_run_final_approach = self.create_service(
            srv_name="final_approach_controller/start_final_approach",
            srv_type=StartFinalApproach,
            callback=self._srv_cb_run_final_approach,
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

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            callback_group=self.callback_group,
            qos_profile=1,
        )

        # Timers
        self._timer_setup_tf_frames = self.create_timer(timer_period_sec=1.0, callback=self._timer_cb_setup_tf_frames)
        self._timer_run_controller = None
        # self._timer_run_controller.cancel()

        # Messages
        self.msg_vl53l8cx_filtered = Vl6180FilteredStamped()
        self.msg_twist = TwistStamped()

        # Controller attributes
        self.d_tof0 = 0.0
        self.d_tof1 = 0.0
        self.max_linear_speed = 0.05
        # TODO::::: need to read raw data to make sure that the reading is valid??
        self.tf_mp_base_to_tof0 = self.tf_mp_base_to_tof1 = self.tf_mp_cut_point_to_base = self.tf_tof0_to_cut_point = (
            self.tf_tof0_to_tof1
        ) = np.identity(4)
        self._dist_cut_point_to_branch_threshold = 7e-3
        self.controller_running = False
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
        if not np.all(np.isclose(self.tf_tof0_to_tof1[:3, :3], np.identity(3), atol=1e-3)):
            raise ValueError("The two ToF frames are not aligned with each other.")
        return

    def _timer_cb_run_controller(self):

        dist, theta = self.get_cut_point_info()
        dist_cut_point_to_branch = dist - self.tf_cut_point_to_tof0[2, 3]
        # self.warn(dist)
        self.warn(dist_cut_point_to_branch)

        if (
            np.isclose(dist_cut_point_to_branch, 0, atol=self._dist_cut_point_to_branch_threshold)
            or (dist_cut_point_to_branch) < 0
        ):
            self.info(f"Reached terminating point at dist:{dist}, theta: {theta}")
            self.controller_running = False
            self._timer_run_controller.cancel()
            self._timer_run_controller.destroy()
            self._timer_run_controller = None
            return

        tf_cut_point_to_world = self.lookup_transform(
            source_frame="mock_pruner__tool0",
            target_frame="cart__base",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=True,
        )
        twist = self.get_twist(tf_world_to_eef=tf_cut_point_to_world, dist=dist)
        self.msg_twist.twist.linear.x = twist[0]
        self.msg_twist.twist.linear.y = twist[1]
        self.msg_twist.twist.linear.z = twist[2]
        self.msg_twist.twist.angular.x = twist[3]
        self.msg_twist.twist.angular.y = twist[4]
        self.msg_twist.twist.angular.z = twist[5]
        self.msg_twist.header.frame_id = "cart__base"
        self.msg_twist.header.stamp = self.get_clock().now().to_msg()
        self._pub_servo.publish(self.msg_twist)
        return

    def _sub_cb_tof_filtered(self, msg: Vl6180FilteredStamped):
        # Do some checks, make sure that the readings make sense in intuitive way.
        # Make sure readings do not exceed maximum. # TODO: Find a way to get sensor parameters in here
        self.d_tof0 = msg.data[0] / 1000  # mm to m
        self.d_tof1 = msg.data[1] / 1000
        # self.warn(self.d_tof0)
        return

    def print_stuff(self):
        # self.warn(self.get_cut_point_info())
        # self.get_cut_point_info()
        return

    def _srv_cb_run_final_approach(self, request, response):
        self.controller_running = True
        self._timer_run_controller = self.create_timer(
            timer_period_sec=1 / 30, callback=self._timer_cb_run_controller, callback_group=self.callback_group
        )
        # self._timer_run_controller/
        response.success = True
        return response

    def get_cut_point_info(self) -> tuple:
        tof0_to_tof1_pos_vec = self.tf_tof0_to_tof1[:3, 3]
        tof_linear_distance = np.linalg.norm(tof0_to_tof1_pos_vec)
        dist = (self.d_tof0 + self.d_tof1) / 2
        d_diff = self.d_tof0 - self.d_tof1
        theta = np.arctan(d_diff / tof_linear_distance)  # should return angle (-pi/2, pi/2)

        return dist, theta

    def get_twist(self, tf_world_to_eef: np.ndarray, dist: float) -> np.ndarray:
        """Need to get the view matrix of the cut point to the rotation axis, normalize to the max lin speed"""
        # log.info(self.tf_tof0_to_cut_point)
        if dist - self.tf_cut_point_to_tof0[2, 3] <= 0:
            return np.zeros((6, 1))
        Kp = 1 / (dist - self.tf_cut_point_to_tof0[2, 3])

        velocity = Kp * self.max_linear_speed * tf_world_to_eef[:3, :3] @ [0, 0, 1]
        # tf_eef_to_world = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['mock_pruner__tool0']['id'])).reshape([4, 4], order="F")
        twist = np.zeros(6)
        twist[0:3] = velocity / np.linalg.norm(velocity) * self.max_linear_speed
        return twist


def main():
    rclpy.init()
    fac_node = FinalApproachControllerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(fac_node, executor=executor)
    fac_node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    main()
