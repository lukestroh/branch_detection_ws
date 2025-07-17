#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TwistStamped
import py_trees
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from collections import deque
import numpy as np


class TeleopNode(Node):
    def __init__(self):
        super().__init__(node_name="teleop_node")
        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        # PyTrees
        self.blackboard = py_trees.blackboard.Blackboard()

        # Parameters
        # self.max_vel = 0.08333333333333334  # TODO: get max vel from param file
        self.eef_max_linear_speed = 0.08333333333333334
        self.eef_max_rot_speed = 0.5
        self.ur_robot_prefix = "ur5e__"

        # Callback groups
        self.cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self._sub_joy_state = self.create_subscription(
            msg_type=Joy,
            topic="/joy",
            callback=self._sub_cb_joy_state,
            qos_profile=1,
            callback_group=self.cb_group,
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            qos_profile=QoSProfile(
                depth=10, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST
            ),
            callback_group=self.cb_group,
        )

        # Timers
        self._timer_pub_servo = self.create_timer(
            timer_period_sec=1 / 250, callback=self._timer_cb_pub_servo, callback_group=self.cb_group
        )

        # Messages
        self.servo_msg = TwistStamped()
        self.servo_msg.header.frame_id = f"{self.ur_robot_prefix}base_link"

        # Joystick mappings (XBox 1 controller)
        self.buttons = {
            "LB": 4,
            "RB": 5,
        }
        self.axes = {
            "LT": 2,
            "RT": 5,
            "left_joy_x": 0,
            "left_joy_y": 1,
            "right_joy_x": 3,
            "right_joy_y": 4,
        }
        self.remapped_velocities = {
            "LT": 0.0,
            "RT": 0.0,
            "left_joy_x": 0.0,
            "left_joy_y": 0.0,
            "right_joy_x": 0.0,
            "right_joy_y": 0.0,
            "T_sum": {},
        }
        self.scaled_velocities = np.zeros(3, dtype=float)  # Array for keeping track of joystick -> world scaled values

        # self.cmd_vel_values = deque(np.zeros(5))
        return

    def _sub_cb_joy_state(self, msg: Joy):

        axes_states = msg.axes

        # Remap the trigger values to positive and negative velocities. Unpressed triggers should have a velocity of 0.
        for i, (key, value) in enumerate(self.axes.items()):
            _scaled = self._scale(axes_states[value], 1, -1, 0, self.max_vel)  # joy input range [1,-1]
            if key == "LT":
                self.remapped_vals[i] = -_scaled
            elif key == "RT":
                self.remapped_vals[i] = _scaled

        # sum both trigger values
        vel_cmd = sum(self.remapped_vals)

        # self.cmd_vel_values.popleft()
        # self.cmd_vel_values.append(vel_cmd)

        # publish twist message
        self.servo_msg.header.stamp = self.get_clock().now().to_msg()
        self.servo_msg.twist.linear.x = np.mean(vel_cmd)

        return

    def _scale(self, val, amin, amax, bmin, bmax):
        """Scale a value from one range to a another"""
        if amin < amax:
            if val > amax or val < amin:
                raise ValueError(f"Input value {val} is out of bounds of input range {amin} to {amax}")
        else:
            if val < amax or val > amin:
                raise ValueError(f"Input value {val} is out of bounds of input range {amin} to {amax}")

        arange = amax - amin
        brange = bmax - bmin
        new_val = (((val - amin) * brange) / arange) + bmin
        return new_val

    def _remap_joystick_values(self, msg: Joy) -> None:
        """Remap joystick/trigger/button values to positive and negative velocities. Unpressed should have a velocity of 0.
        Joy input range for XBox controller: [1,-1]. Saves the remapped values to the class variable remapped_velocities.

        @param msg: Joy message containing the joystick state
        @return: None
        """
        # Map axes values to velocities
        for key, value in self.axes.items():
            # Map trigger values
            if key in {"LT", "RT"}:
                if key == "LT":
                    self.remapped_velocities[key] = self._scale(
                        msg.axes[value], 1, -1, 0, -1 * self.eef_max_linear_speed
                    )  # +z
                else:
                    self.remapped_velocities[key] = self._scale(
                        msg.axes[value], 1, -1, 0, self.eef_max_linear_speed
                    )  # -z
            # Map joystick values
            elif key in {"left_joy_x", "left_joy_y"}:
                self.remapped_velocities[key] = self._scale(
                    msg.axes[value], 1, -1, -self.eef_max_linear_speed, self.eef_max_linear_speed
                )  # +/- x,y
            elif key in {"right_joy_x", "right_joy_y"}:
                self.remapped_velocities[key] = self._scale(
                    msg.axes[value], 1, -1, -self.eef_max_rot_speed, self.eef_max_rot_speed
                )

        # Map button values to max velocities
        for key, value in self.buttons.items():
            if key == "LB":
                self.remapped_velocities[key] = self._scale(
                    msg.buttons[value], 0, 1, 0, -self.eef_max_rot_speed
                )  # Rotate z-axis ccw
            else:
                self.remapped_velocities[key] = self._scale(
                    msg.buttons[value], 0, 1, 0, self.eef_max_rot_speed
                )  # Rotate z-axis cw
        return

    def _sum_trigger_values(self) -> None:
        """Sum the trigger values to get the raw twist commands"""
        self.remapped_velocities["z_sum"] = {
            "linear": self.remapped_velocities["LT"] + self.remapped_velocities["RT"],
            "angular": self.remapped_velocities["LB"] + self.remapped_velocities["RB"],
        }
        return

    def _scale_to_max_speed(self) -> None:
        self.scaled_velocities[0] = self.remapped_velocities["left_joy_x"]
        self.scaled_velocities[1] = self.remapped_velocities["left_joy_y"]
        self.scaled_velocities[2] = self.remapped_velocities["z_sum"]["linear"]
        speed = np.linalg.norm(self.scaled_velocities)
        if speed == 0:
            return
        if speed > self.eef_max_linear_speed:
            self.scaled_velocities = self.scaled_velocities / speed * self.eef_max_linear_speed
        return

    def _assign_twist_values(self) -> None:
        self.servo_msg.twist.linear.x = self.scaled_velocities[0]
        self.servo_msg.twist.linear.y = self.scaled_velocities[1]
        self.servo_msg.twist.linear.z = self.scaled_velocities[2]
        self.servo_msg.twist.angular.x = self.remapped_velocities["right_joy_x"]
        self.servo_msg.twist.angular.y = self.remapped_velocities["right_joy_y"]
        self.servo_msg.twist.angular.z = self.remapped_velocities["z_sum"]["angular"]
        return

    def _sub_cb_joy_state(self, msg: Joy):
        """Callback for the joystick state message. Remaps joystick values to linear and angular velocities for servo control.
        Does not publish the message, only updates the class variables.

        @param msg: Joy message containing the joystick state
        @return: None
        """
        self._remap_joystick_values(msg)
        self._sum_trigger_values()
        self._scale_to_max_speed()
        self._assign_twist_values()

        self.servo_msg.header.stamp = self.get_clock().now().to_msg()
        return

    def _timer_cb_pub_servo(self):
        self._pub_servo.publish(self.servo_msg)
        return


def main():
    rclpy.init()
    teleop_node = TeleopNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(teleop_node, executor=executor)
    teleop_node.destroy_node()
    rclpy.shutdown()
    return
