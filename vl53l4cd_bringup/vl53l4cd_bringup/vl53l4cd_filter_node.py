#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from ament_index_python.packages import get_package_share_directory


from vl53l4cd_msgs.msg import Vl53l4cd, Vl53l4cdStamped
from tof_msgs.msg import TofStamped

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from collections import deque
import numpy as np
import scipy.signal as ssi


class VL53L4CDFilterNode(Node):
    RANGING_ERR = -1
    RANGING_MAX = 1.600
    tof_model_type = "VL53L4CD"

    def __init__(self, node_name="vl53l4cd_filtered_node") -> None:
        super().__init__(node_name=node_name)

        # Loggers
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.err = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

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
        self._param_depth_near_plane = (
            self.declare_parameter(name="depth.near_plane", value=Parameter.Type.DOUBLE)
            .get_parameter_value()
            .double_value
        )
        self._param_depth_far_plane = (
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
        self._sub_vl53l4cd_distance_raw = self.create_subscription(
            msg_type=Vl53l4cdStamped,
            # msg_type=ToFData,
            topic="/microROS/vl53l4cd/data",
            callback=self._sub_cb_vl53l4cd_distance_raw,
            qos_profile=50,
        )

        # Publishers
        self._pub_tof_filtered = self.create_publisher(msg_type=TofStamped, topic="/vl53l4cd/filtered", qos_profile=50)

        # Messages
        self._msg_tof_stamped = TofStamped()
        self._msg_tof_stamped.config.row.size = 1
        self._msg_tof_stamped.config.row.stride = 1
        self._msg_tof_stamped.config.column.size = 1
        self._msg_tof_stamped.config.column.stride = 1
        self._msg_tof_stamped.type = "vl53l4cd"
        self._msg_tof_stamped.dfov = self.depth_dfov
        self._msg_tof_stamped.near_plane = self._param_depth_near_plane
        self._msg_tof_stamped.far_plane = self._param_depth_far_plane

        # Initialize variables
        self.deque_size = 10
        self.deques = [deque([self.RANGING_MAX] * self.deque_size), deque([self.RANGING_MAX] * self.deque_size)]

        fs = 22.0
        cutoff = 5.0
        order = 5
        self.sos_filt = ssi.butter(N=order, Wn=cutoff, fs=fs, btype='low', output='sos')
        self.zi = ssi.sosfilt_zi(self.sos_filt)
        self.state_filt = self.zi * self._param_depth_far_plane

        # self.info(self.kalmans)
        return

    def _sub_cb_vl53l4cd_distance_raw(self, msg: Vl53l4cdStamped):
        """
        Callback to filter raw distance data from the VL53L4CD sensor
        """
        # Moving average filter
        try:
            if msg.status != 0:
                msg.distance = 1600 # Set to far plane so that alignment check doesn't get stale value TODO: change to far plane var
            self.deques[msg.dev_id].popleft()
            self.deques[msg.dev_id].append(msg.distance / 1000)

            self._msg_tof_stamped.dev_id = msg.dev_id
            self._msg_tof_stamped.data = [np.mean(self.deques[msg.dev_id])]
            self._msg_tof_stamped.status = msg.status

            self._msg_tof_stamped.header.frame_id = f"tof{msg.dev_id}"  # TODO: put prefix names?
            self._msg_tof_stamped.header.stamp = self.get_clock().now().to_msg()

            self._pub_tof_filtered.publish(msg=self._msg_tof_stamped)

        except IndexError:
            self.fatal("Sensor ID value exceeded the number of moving average buffers. Please adjust.")

        ################################################################################################################
        # SavGol filter
        # try:
        #     if msg.status != 0:
        #         msg.distance = int(self._param_depth_far_plane * 1000) # put into mm for consistency
            
        #     self.deques[msg.dev_id].popleft()
        #     self.deques[msg.dev_id].append(msg.distance / 1000)

        #     filtered_buf = ssi.savgol_filter(x=self.deques[msg.dev_id], window_length=self.deque_size, polyorder=1)

        #     self._msg_tof_stamped.dev_id = msg.dev_id
        #     self._msg_tof_stamped.data = [filtered_buf[-1]]
        #     self._msg_tof_stamped.status = msg.status

        #     self._msg_tof_stamped.header.frame_id = f"tof{msg.dev_id}"  # TODO: put prefix names?
        #     # self._msg_tof_stamped.header.stamp = self.get_clock().now().to_msg()
        #     self._msg_tof_stamped.header.stamp = msg.header.stamp
        #     self._pub_tof_filtered.publish(msg=self._msg_tof_stamped)
        # except IndexError:
        #     self.fatal("Sensor ID value exceeded the number of moving average buffers. Please adjust.")

        ################################################################################################################
        # Butterworth low-pass filter
    #     try:
    #         if msg.status != 0:
    #             msg.distance = 1600 # Set to far plane so that alignment check doesn't get stale value TODO: change to far plane var
            
    #         filt_dist, self.state_filt = self.process_sample(x=msg.distance / 1000, state=self.state_filt)
    #         self.warn(type(filt_dist))
    #         self._msg_tof_stamped.dev_id = msg.dev_id
    #         self._msg_tof_stamped.data = float(filt_dist)
    #         self._msg_tof_stamped.status = msg.status

    #         self._msg_tof_stamped.header.frame_id = f"tof{msg.dev_id}"  # TODO: put prefix names?
    #         self._msg_tof_stamped.header.stamp = self.get_clock().now().to_msg()

    #         self._pub_tof_filtered.publish(msg=self._msg_tof_stamped)
            
    #     except IndexError:
    #         self.fatal("Sensor ID value exceeded the number of moving average buffers. Please adjust.")

    #     return
    
    # def process_sample(self, x, state):
    #     y, state = ssi.sosfilt(self.sos_filt, [x], zi=state)
    #     return y[0], state


def main():
    rclpy.init()
    tof_node = VL53L4CDFilterNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node=tof_node, executor=executor)
    tof_node.destroy_node()
    rclpy.shutdown()

    return


if __name__ == "__main__":
    main()
