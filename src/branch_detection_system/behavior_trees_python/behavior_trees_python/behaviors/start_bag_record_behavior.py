#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from rclpy.node import Node
from rclpy.parameter import Parameter

# from rclpy.service import S

from ros2bag_msgs.srv import StartRecord


class StartBagRecordBehavior(pt.behaviour.Behaviour):
    def __init__(self, name):
        super(StartBagRecordBehavior, self).__init__(name)
        self.blackboard = pt.blackboard.Blackboard()
        return

    def setup(self, node: Node):
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.node.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n{x}")
        self.info("Setting up StartBagRecordBehavior")

        # Parameters
        self._param_record_loc = (
            self.node.declare_parameter(name="record_loc", value=Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )

        # Service clients
        self._srv_client_start_bag_record = self.node.create_client(srv_name="/start_bag_record", srv_type=StartRecord)
        self._srv_client_start_bag_record.wait_for_service()

        # Behaviour attributes
        self.goal_status = None

        return

    def initialise(self):
        """Call the start bag record service"""
        start_record_req = StartRecord.Request()
        start_record_req.record_bag = True
        start_record_req.record_loc = self._param_record_loc

        self._send_goal_future: Future = self._srv_client_start_bag_record.call_async(request=start_record_req)
        self._send_goal_future.add_done_callback(callback=self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        result: StartRecord.Response = future.result()
        self.goal_status = result.success
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
