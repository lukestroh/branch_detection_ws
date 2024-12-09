#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped

import numpy as np
import secrets

class FakeWrenchPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name='fake_wrench_pub_node')

        # Publishers
        self._pub_wrench_raw = self.create_publisher(
            msg_type=WrenchStamped,
            topic='/wrench',
            qos_profile=1
        )

        # Timers
        self._timer_pub_wrench_raw = self.create_timer(
            timer_period_sec=0.1,
            callback=self._timer_cb__pub_wrench_raw
        )

        # Messages
        self.msg_wrench = WrenchStamped()
        self.msg_wrench.header.frame_id = 'ur5e__tool0'

        # RNG
        self.rng = np.random.default_rng(seed=secrets.randbits(128))
        return
    
    def _timer_cb__pub_wrench_raw(self) -> None:
        self.msg_wrench.wrench.force.y = self.rng.uniform(-2, 2)
        self.msg_wrench.wrench.force.z = self.rng.uniform(-1,1)
        self.msg_wrench.wrench.torque.x = self.rng.uniform(-0.1, 0.1)

        self.msg_wrench.header.stamp = self.get_clock().now().to_msg()

        self._pub_wrench_raw.publish(msg=self.msg_wrench)
        return


def main():
    rclpy.init()
    fake_wrench_publisher_node = FakeWrenchPublisherNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node=fake_wrench_publisher_node, executor=executor)
    fake_wrench_publisher_node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    main()