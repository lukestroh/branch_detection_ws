#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from ism330dhcx_msgs.msg import Ism330dhcxStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Imu
from tof_msgs.msg import TofStamped

import numpy as np
import pprint as pp


class ISM330DHCXFilterNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name="ism330dhcx_filter_node")

        # Loggers
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.err = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Launch parameters

        # Subscriptions
        self._sub_ism330dhcx_raw = self.create_subscription(
            msg_type=Ism330dhcxStamped,
            topic="/microROS/ism330dhcx/data",
            callback=self._sub_cb_ism330dhcx_raw,
            qos_profile=20,
        )

        # Publishers
        self._pub_ism330dhcx_stamped = self.create_publisher(msg_type=Imu, topic="ism330dhcx_stamped", qos_profile=64)

        # Messages
        self.acc_msg = Imu()

        return

    def _sub_cb_ism330dhcx_raw(self, msg: Ism330dhcxStamped):
        # self.warn(msg)
        _data = msg.data
        size = (msg.data_config.row.size, msg.data_config.column.size)
        # self.warn(size)
        data = np.resize(_data, new_shape=size)

        # data_flipped = np.flip(data)

        for acc in data:
            self.acc_msg.header.stamp = Time(seconds=acc[0]).to_msg()
            self.acc_msg.header.frame_id = msg.header.frame_id
            self.acc_msg.linear_acceleration = Vector3(x=float(acc[1]), y=float(acc[2]), z=float(acc[3]))
            self._pub_ism330dhcx_stamped.publish(self.acc_msg)

        return


def main():
    rclpy.init()
    ism330dhcx_filter_node = ISM330DHCXFilterNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(ism330dhcx_filter_node, executor=executor)
    ism330dhcx_filter_node.destroy_node()
    rclpy.shutdown()
    return
