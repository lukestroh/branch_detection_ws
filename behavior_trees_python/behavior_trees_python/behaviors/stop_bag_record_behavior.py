#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from rclpy.node import Node
from rclpy.parameter import Parameter

# from rclpy.service import S

from ros2bag_msgs.srv import StopRecord


class StopBagRecordBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str, record_bag: bool):
        super(StopBagRecordBehavior, self).__init__(name)
        self.name = name
        self.record_bag = record_bag
        return

    def setup(self, node: Node):
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n[{self.name}] {x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n[{self.name}] {x}")
        self.error = lambda x: self.node.get_logger().error(f"\n[{self.name}] {x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n[{self.name}] {x}")
        self.info(f"Setting up {self.name}")

        # Service clients
        self._srv_client_stop_bag_record = self.node.create_client(srv_name="/stop_bag_record", srv_type=StopRecord)
        self._srv_client_stop_bag_record.wait_for_service()

        # Behaviour attributes
        self.goal_status = None

        self.blackboard = pt.blackboard.Client(name=self.name)
        # self.blackboard.register_key(key="current_pose_index", access=pt.common.Access.WRITE)
        # self.blackboard.register_key(key="current_pose", access=pt.common.Access.WRITE)

        return

    def initialise(self):
        """Call the stop bag record service"""
        if self.record_bag == False:
            self.goal_status = True
        else:
            self.goal_status = None
        stop_record_req = StopRecord.Request()
        self._send_goal_future: Future = self._srv_client_stop_bag_record.call_async(request=stop_record_req)
        self._send_goal_future.add_done_callback(callback=self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        result: StopRecord.Response = future.result()
        self.goal_status = result.success
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
