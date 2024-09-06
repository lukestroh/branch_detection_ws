#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from teensy32_tof_msgs.msg import ToFData, ToFDataArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA


from teensy32_tof_bringup.utils.kalman import Kalman

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import numpy as np

"""
TODO: would be nice to be able to dynamically configure the number and placement of ToF sensors

"""


class ToFNode(Node):
    RANGING_ERR = -1
    def __init__(self) -> None:
        super().__init__(node_name="tof_node")
        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")

        self.use_mock_hardware = self.declare_parameter("use_mock_hardware", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        self.demo_mode = self.declare_parameter("demo_mode", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        self.num_sensors = self.declare_parameter("num_sensors", value=Parameter.Type.INTEGER).get_parameter_value().integer_value


        """TODO: Clean up parameters section"""
        self.tof_model_type = self.declare_parameter("tof_model", value=Parameter.Type.STRING).get_parameter_value().string_value
        # self.sensor_types = [] # TODO: Ability to use multiple ToF types?
        self.info(f"\nTime-of-flight sensor configuration:\n\tSensor type: {self.tof_model_type}\n\tNumber of sensors: {self.num_sensors}")



        # Subscriptions
        self._sub_tof_raw = self.create_subscription(
            msg_type=ToFDataArray,
            topic="/microROS/tof_data_array",
            callback=self._sub_cb_tof_raw,
            qos_profile=1,
        )
        self._sub_joint_states = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self._sub_cb_joint_states,
            qos_profile=1
        )

        # Publishers
        self._pub_tof_filtered = self.create_publisher(
            msg_type=ToFDataArray,
            topic="tof_filtered",
            qos_profile=1,
        )
        self._pub_tof_markers = self.create_publisher(
            msg_type=MarkerArray,
            topic="tof_markers",
            qos_profile=1
        )

        # Timers
        self._timer_pub_tof_filtered = self.create_timer(
            timer_period_sec=1/30,
            callback=self._timer_cb_pub_tof_filtered
        )

        """ Set up the ToF Kalman filters """
        # ToF model parameters
        # tof_model_param = self.declare_parameter(name="tof_model", value=Parameter.Type.STRING)
        tof_fov_x_param = self.declare_parameter(name="tof_fov_x", value=Parameter.Type.DOUBLE)
        tof_fov_y_param = self.declare_parameter(name="tof_fov_y", value=Parameter.Type.DOUBLE)
        range_z_min_param = self.declare_parameter(name="tof_range_z_min", value=Parameter.Type.DOUBLE)
        range_z_max_param = self.declare_parameter(name="tof_range_z_max", value=Parameter.Type.DOUBLE)
        # self.tof_model = tof_model_param.get_parameter_value().string_value
        self.sensor_fov = np.array([
            tof_fov_x_param.get_parameter_value().double_value,
            tof_fov_y_param.get_parameter_value().double_value
        ])
        self.sensor_z_range = np.array([
            range_z_min_param.get_parameter_value().double_value,
            range_z_max_param.get_parameter_value().double_value,
        ])
        # self.warn(f"{self.sensor_z_range}")

        # Values
        self.tof_vals = np.zeros(self.num_sensors)
        self.tof_filtered_vals = np.zeros(self.num_sensors)
        self.tof_kalmans = np.empty(shape=2, dtype=KalmanFilter)
        self.tof_covs = np.array([1.6504520533467566**2, 1.6593793309147946**2]) # measured values
        self.initial_err = 1000.0
        self.measurement_err = 15.0
        for i, val in np.ndenumerate(self.tof_vals):
            kalman = KalmanFilter(dim_x=2, dim_z=1)
            kalman.x = np.array([[val, 0]]).transpose()
            kalman.F = np.array([[1,1],[0,1]])
            kalman.H = np.array([[1, 0]])
            kalman.P = np.identity(kalman.dim_x) * self.initial_err
            kalman.R = self.measurement_err
            kalman.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=self.tof_covs[i])
            self.tof_kalmans[i] = kalman
        return

    def _sub_cb_tof_raw(self, msg: ToFDataArray) -> None:
        """Callback method for ToF raw data subscriber"""
        self.tof_vals = msg.data
        try:
            for i, dist in enumerate(self.tof_vals):
                if dist != self.RANGING_ERR:
                    self.tof_kalmans[i].predict()
                    self.tof_kalmans[i].update([dist])
                    self.tof_filtered_vals[i] = self.tof_kalmans[i].x[0,0]
        except IndexError as e:
            self.get_logger().error(f"{e}: Please check ToF setup and confirm that the number of sensors connected to the microROS agent aligns with the specified configuration.")
        return

    def _sub_cb_joint_states(self, msg: JointState) -> None:
        """Callback method for joint states"""
        self.joint_state = list(zip(msg.name, msg.position))
        return

    def _timer_cb_pub_tof_filtered(self) -> None:
        """Callback timer for filtered ToF data publisher"""
        msg_tof_filtered = ToFDataArray()
        msg_tof_filtered.data = list(self.tof_filtered_vals / 1000) # convert from mm to m.
        self._pub_tof_filtered.publish(msg=msg_tof_filtered)

        msg_tof_markers = self._generate_point_markers()
        self._pub_tof_markers.publish(msg=msg_tof_markers)
        return

    def _generate_cylinder_msg(self) -> Marker:
        """Generate a cylinder marker message"""
        marker = Marker()
        if self.demo_mode:
            marker.header.frame_id = "map" # TODO: Get rid of this...
        else:
            marker.header.frame_id = "tool0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        x = sum([self.tof0_filtered, self.tof1_filtered]) / 2 / 100
        marker.pose.position = Point(x=0.0, y=0.0, z=x)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0

        return marker

    def _generate_point_markers(self) -> MarkerArray:
        """Generate the laser markers"""
        markers = MarkerArray()
        for i, filtered_val in np.ndenumerate(self.tof_filtered_vals):
            i = i[0]
            # self.get_logger().error(f"HELLO WORLD: {i}")
            marker = Marker()
            marker.type = marker.POINTS
            marker.ns = "tof"
            marker.id = int(i)
            marker.header.frame_id = f"tof{i}"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.action = marker.ADD

            marker.scale.x = 0.01  # Size of the points in the x direction
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            marker.colors = [ColorRGBA(a=1.0, r=1.0, g=0.8, b=0.25)]
            marker.points = [Point(x=0.0, y=0.0, z=(filtered_val / 100))]
            markers.markers.append(marker)
        return markers


def main():
    rclpy.init()
    tof_node = ToFNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node=tof_node, executor=executor)
    tof_node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    main()
