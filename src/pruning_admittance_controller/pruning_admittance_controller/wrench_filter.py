#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Vector3Stamped, Wrench, WrenchStamped

from collections import deque
import numpy as np


class WrenchFilterNode(Node):
    def __init__(self, deque_size: int):
        super().__init__(node_name="wrench_filter_node")

        # Parameters
        # TODO: Get robot part prefixes from launch -- part of integrating urdf construction

        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        # Subscribers
        self._sub_wrench_raw = self.create_subscription(
            msg_type=WrenchStamped, topic="/wrench", callback=self._sub_cb_wrench_raw, qos_profile=1
        )

        # Publishers
        self._pub_wrench_filtered = self.create_publisher(
            msg_type=WrenchStamped, topic="/wrench_filtered", qos_profile=1
        )

        # Internal messages
        self.msg_wrench_stamped_filtered = WrenchStamped()

        # Moving average filter
        self._sub_counter = 0
        self.deque_size = deque_size
        self.t_x_deque = deque(np.zeros(deque_size))
        self.f_y_deque = deque(np.zeros(deque_size))
        self.f_z_deque = deque(np.zeros(deque_size))
        return

    def _sub_cb_wrench_raw(self, msg: WrenchStamped) -> None:
        """Callback to handle raw wrench data"""
        self.t_x_deque.popleft()

        self.f_y_deque.popleft()
        self.f_z_deque.popleft()

        self.f_y_deque.append(msg.wrench.force.y)
        self.f_z_deque.append(msg.wrench.force.z)
        self.t_x_deque.append(msg.wrench.torque.x)

        # Don't publish anything until the buffers have filled
        if self._sub_counter > self.deque_size:
            self.msg_wrench_stamped_filtered.wrench.force.y = np.mean(self.f_y_deque)
            self.msg_wrench_stamped_filtered.wrench.force.z = np.mean(self.f_z_deque)
            self.msg_wrench_stamped_filtered.wrench.torque.x = np.mean(self.t_x_deque)

            self.msg_wrench_stamped_filtered.header.frame_id = f"ur5e__tool0"  # TODO: check where sensor is located?
            self.msg_wrench_stamped_filtered.header.stamp = self.get_clock().now().to_msg()

            # Publish filtered values
            self._pub_wrench_filtered.publish(msg=self.msg_wrench_stamped_filtered)

        self._sub_counter += 1

        return


def main():
    rclpy.init()
    contact_watcher_node = WrenchFilterNode(deque_size=50)
    executor = MultiThreadedExecutor()
    rclpy.spin(node=contact_watcher_node, executor=executor)
    contact_watcher_node.destroy_node()
    rclpy.shutdown()

    return


if __name__ == "__main__":
    main()
