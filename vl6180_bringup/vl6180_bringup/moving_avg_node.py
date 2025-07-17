#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from ament_index_python.packages import get_package_share_directory

from vl6180_msgs.msg import Vl6180, Vl6180Stamped, Vl6180FilteredStamped  # Vl6180Filtered
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA


from vl6180_bringup.utils.kalman import Kalman

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from array import array
from collections import deque
import numpy as np
import json
import os
import scipy.signal as si
from typing import List, Sequence
from numpy.typing import NDArray


class VL6180FilterNode(Node):
    RANGING_ERR = -1
    RANGING_MAX = 255
    tof_model_type = "VL6180"

    def __init__(self, node_name="vl6180_filtered_node") -> None:
        super().__init__(node_name=node_name)

        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        # Launch parameters
        self.use_mock_hardware = (
            self.declare_parameter("use_mock_hardware", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        )
        # TODO: keep sensor quanitity in launch file, just launch two of these nodes ?
        # self.num_sensors = (
        #     self.declare_parameter("sensor_quantity", value=Parameter.Type.INTEGER).get_parameter_value().integer_value
        # )
        self.tof_id = self.declare_parameter("tof_id", value=Parameter.Type.INTEGER).get_parameter_value().integer_value
        self.tf_frame = f"mock_pruner__tof{self.tof_id}"

        # # Yaml parameters
        self.name = self.declare_parameter(name="name", value=Parameter.Type.STRING).get_parameter_value().string_value
        self.depth_dfov = (
            self.declare_parameter(name="depth.dfov", value=Parameter.Type.DOUBLE).get_parameter_value().double_value
        )
        self.depth_near_plane = (
            self.declare_parameter(name="depth.near_plane", value=Parameter.Type.DOUBLE)
            .get_parameter_value()
            .double_value
        )
        self.depth_far_plane = (
            self.declare_parameter(name="depth.far_plane", value=Parameter.Type.DOUBLE)
            .get_parameter_value()
            .double_value
        )

        self.depth_width = (
            self.declare_parameter(name="depth.width", value=Parameter.Type.INTEGER).get_parameter_value().integer_value
        )
        self.depth_height = (
            self.declare_parameter(name="depth.height", value=Parameter.Type.INTEGER)
            .get_parameter_value()
            .integer_value
        )

        self.num_sensors = 2
        self.info(
            f"\nTime-of-flight sensor configuration:\n\tSensor type: {self.tof_model_type}\n\tNumber of sensors: {self.num_sensors}"
        )

        # Subscriptions
        self._sub_vl6180_distance_raw = self.create_subscription(
            msg_type=Vl6180,
            # msg_type=ToFData,
            topic="/microROS/vl6180/data",
            callback=self._sub_cb_vl6180_distance_raw,
            qos_profile=1,
        )

        # # Publishers
        self._pub_tof_filtered = self.create_publisher(
            msg_type=Vl6180FilteredStamped,
            topic="/vl6180/filtered",
            qos_profile=1,
        )

        # Initialize variables
        json_covariances = json.load(
            open(os.path.join(get_package_share_directory("vl6180_bringup"), "config/covariances.json"), "r")
        )
        self.vl6180_msg_raw = Vl6180()
        self.vl6180_msg_raw.data = [0, 0]
        self.vl6180_msg_filtered = Vl6180FilteredStamped()
        self.vl6180_msg_filtered.data = [0.0, 0.0]

        self.deque_size = 20
        self.deques = [deque([self.RANGING_MAX] * self.deque_size), deque([self.RANGING_MAX] * self.deque_size)]

        # self.info(self.kalmans)
        return

    def _sub_cb_vl6180_distance_raw(self, msg: Vl6180):
        """
        Callback for raw distance data from the VL6180 sensor
        """
        self.vl6180_msg_raw = msg

        # self.info(self.deques)
        # self.warn(self.depth_near_plane)

        try:
            for i in range(2):  # TODO: hacky, this represents two sensors. Fix.
                if (msg.data[i] == self.RANGING_ERR) or (msg.data[i] == 0):
                    # TODO: This is bad logic, need an and...
                    pass
                else:
                    # self.warn(msg.data[0])

                    self.deques[i].popleft()
                    self.deques[i].append(self.vl6180_msg_raw.data[i])
                    self.vl6180_msg_filtered.data[i] = np.mean(self.deques[i])

            # self.vl6180_msg_filtered.header.frame_id = "vl6180_0" # TODO: need two nodes or publishers for two separate frames
            self.vl6180_msg_filtered.header.stamp = self.get_clock().now().to_msg()
            self._pub_tof_filtered.publish(self.vl6180_msg_filtered)
        except IndexError as e:
            self.err(f"IndexError: {e}. Check the ranging mode.")
        return


def main():
    rclpy.init()
    tof_node = VL6180FilterNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node=tof_node, executor=executor)
    tof_node.destroy_node()
    rclpy.shutdown()

    return


if __name__ == "__main__":
    main()
