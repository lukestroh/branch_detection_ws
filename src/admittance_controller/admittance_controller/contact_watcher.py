#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped

from collections import deque
import numpy as np

class ContactWatcherNode(Node):
    def __init__(self, deque_size):
        super().__init__(node_name="contact_watcher_node")

        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        # Parameters
        self.f_thresh = 0.75 # N
        
        # Subscribers
        self._sub_wrench_filtered = self.create_subscription(
            msg_type=WrenchStamped,
            topic='/wrench_filtered',
            callback=self._sub_cb_wrench_filtered,
            qos_profile=1
        )

        # 

        # Data deques
        self.deque_size = deque_size
        self.x_torques = deque(np.zeros(deque_size))
        self.y_forces = deque(np.zeros(deque_size))
        self.z_forces = deque(np.zeros(deque_size))

        self.ones = np.ones(deque_size)

        self.is_watching = True
        
        return
    
    def _sub_cb_wrench_filtered(self, msg: WrenchStamped) -> None:
        """Callback to handle filtered wrench data"""

        self.x_torques.popleft()
        self.y_forces.popleft()
        self.z_forces.popleft()
        self.x_torques.append(msg.wrench.torque.x)
        self.y_forces.append(msg.wrench.force.y)
        self.z_forces.append(msg.wrench.force.z)

        # Masks
        if self.is_watching:
            z_less_than = (np.asarray(self.z_forces) < -self.f_thresh)
            z_more_than = (np.asarray(self.z_forces) > self.f_thresh)
            z_outside_mask = (z_less_than | z_more_than)

            y_less_than = (np.asarray(self.y_forces) < -self.f_thresh)
            y_more_than = (np.asarray(self.y_forces) > self.f_thresh)
            y_outside_mask = (y_less_than | y_more_than)

            x_less_than = (np.asarray(self.x_torques) < -self.f_thresh)
            x_more_than = (np.asarray(self.x_torques) > self.f_thresh)
            x_outside_mask = (x_less_than | x_more_than)

            num_z_outside = np.sum(self.ones[z_outside_mask])
            num_y_outside = np.sum(self.ones[y_outside_mask])
            num_x_outside = np.sum(self.ones[x_outside_mask])

            if any(num > 5 for num in [num_x_outside, num_y_outside, num_z_outside]):
                self.warn("Force detected! Contact triggered")


        return
    


def main():
    rclpy.init()
    contact_watcher_node = ContactWatcherNode(deque_size=10)
    executor = MultiThreadedExecutor()
    rclpy.spin(node=contact_watcher_node, executor=executor)
    contact_watcher_node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    main()