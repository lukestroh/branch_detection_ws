#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from process_io_msgs.srv import SetPoint
from geometry_msgs.msg import Point

import py_trees

from ament_index_python.packages import get_package_share_directory
import os


class SetPointServiceNode(Node):
    def __init__(self):
        super().__init__(node_name="set_point_service_node")
        # Loggers
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.err = lambda x: self.get_logger().error(f"\n{x}")

        self.blackboard =  py_trees.blackboard.Blackboard()

        self.cb_group = ReentrantCallbackGroup()

        self._srv_server_set_point = self.create_service(
            srv_name="set_point",
            srv_type=SetPoint,
            callback=self._srv_server_cb_set_point,
            callback_group=self.cb_group
        )

        self.csv_path = os.path.join(os.path.expanduser('~'),'branch_detection_ws', 'src', 'behavior_trees_python', 'points', 'points.csv')

        return
    
    def _srv_server_cb_set_point(self, request: SetPoint.Request, response: SetPoint.Response):
        """Handle incoming requests to set a goal"""
        try:
            # Set blackboard
            self.blackboard.set("setpoint", (request.position.x, request.position.y, request.position.z))

            # Append point to CSV
            with open("points/points.csv", 'a') as f:
                f.write(f"{request.position.x}, {request.position.y}, {request.position.z}")

            response.success = True
            response.message = f"Goal saved to CSV"

        except Exception as e:
            self.err(f"Failed to write goal: {e}")
            response.success = False
            response.message = f"Error: {e}"

        return response
    

def main():
    rclpy.init()
    set_point_service_node = SetPointServiceNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(set_point_service_node, executor=executor)
    set_point_service_node.destroy_node()
    rclpy.shutdown()

    return

