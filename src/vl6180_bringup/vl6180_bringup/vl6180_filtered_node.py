#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from vl6180_msgs.msg import Vl6180, Vl6180Stamped # Vl6180Filtered
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA


from vl6180_bringup.utils.kalman import Kalman

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from array import array
import numpy as np
import json
from typing import List, Sequence
from numpy.typing import NDArray


class VL6180FilterNode(Node):
    RANGING_ERR = -1
    RANGING_MAX = 3000
    tof_model_type = "VL6180"
    def __init__(self, node_name = "vl6180_filtered_node") -> None:
        super().__init__(node_name=node_name)

        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        # Launch parameters
        self.use_mock_hardware = self.declare_parameter("use_mock_hardware", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        # TODO: keep sensor quanitity in launch file, just launch two of these nodes
        self.num_sensors = self.declare_parameter("sensor_quantity", value=Parameter.Type.INTEGER).get_parameter_value().integer_value

        # Yaml parameters
        self.dfov = self.declare_parameter(name="dfov", value=Parameter.Type.DOUBLE).get_parameter_value().double_value

        self.info(f"\nTime-of-flight sensor configuration:\n\tSensor type: {self.tof_model_type}\n\tNumber of sensors: {self.num_sensors}")

        # Subscriptions
        self._sub_vl6180_distance_raw = self.create_subscription(
            msg_type=Vl6180,
            # msg_type=ToFData,
            topic="/microROS/vl6180/distance",
            callback=self._sub_cb_vl6180_distance_raw,
            qos_profile=1,
        )

        # # Publishers
        # self._pub_tof_filtered = self.create_publisher(
        #     msg_type=Vl6180Filtered,
        #     topic="/vl6180/filtered",
        #     qos_profile=1,
        # )

        # # Initialize variables
        # self.vl6180_msg_raw = Vl61808x8()
        # self.vl6180_msg_filtered = Vl6180Filtered()
        # self.frame_size = (self.vl6180_msg_raw.config.col.size, self.vl6180_msg_raw.config.row.size)
        # self.ranging_mode = 8 # 8x8 ranging mode.
        # self.kalmans = np.empty(self.frame_size[0] * self.frame_size[1], dtype=KalmanFilter)
        # self.covariances = np.array(json_covariances["black_covariances"], dtype=float).flatten()
        # self.measurement_noise = 15.0
        # self.initial_err = 1000.0

        # self.vl6180_msg_filtered.data

        # zeros = np.zeros(self.frame_size[0] * self.frame_size[1], dtype=np.int32).tolist()
        # self.vl6180_msg_raw.data = array('i', np.zeros(self.frame_size[0] * self.frame_size[1], dtype=np.int32))
        # self.vl6180_msg_filtered.data = array('d', np.zeros(self.frame_size[0] * self.frame_size[1], dtype=np.float64))
        # # self.warn(f"val: {zeros}")

        # for i, val in enumerate(self.vl6180_msg_raw.data):
        #     kalman = KalmanFilter(dim_x=2, dim_z=1)
        #     kalman.x = np.array([[val, 0]]).transpose()
        #     kalman.F = np.array([[1,1],[0,1]])
        #     kalman.H = np.array([[1, 0]])
        #     kalman.P = np.identity(kalman.dim_x) * self.initial_err
        #     kalman.R = self.measurement_noise
        #     kalman.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=self.covariances[i])
        #     self.kalmans[i] = kalman

        # self.info(self.kalmans)
        return

    def _sub_cb_vl6180_distance_raw(self, msg: Vl6180):
        """
        Callback for raw distance data from the VL6180 sensor
        """
        self.vl6180_msg_raw = msg
        # self._pub_tof_filtered.publish(msg)
        # try:
        #     for i in range(self.frame_size[0] * self.frame_size[1]):
        #         # self.warn(i)
        #         if msg.data[i] != self.RANGING_ERR or msg.data[i] < self.RANGING_MAX:
        #             self.kalmans[i].predict()
        #             self.kalmans[i].update(self.vl6180_msg_raw.data[i])
        #             self.vl6180_msg_filtered.data[i] = self.kalmans[i].x[0,0]

        #     self.vl6180_msg_filtered.header.frame_id = "vl6180_0" # TODO: dynamically add another
        #     self.vl6180_msg_filtered.header.stamp = self.get_clock().now().to_msg()
        #     self._pub_tof_filtered.publish(self.vl6180_msg_filtered)
        # except IndexError as e:
        #     self.err(f"IndexError: {e}. Check the ranging mode.")
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
