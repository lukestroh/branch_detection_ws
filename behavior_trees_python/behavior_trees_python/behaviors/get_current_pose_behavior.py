#!/usr/bin/env python3

import py_trees as pt
import numpy as np

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import GetCurrentPose


class GetCurrentPoseBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for getting the current pose from a TFNode"""

    def __init__(self, name: str, blackboard_pose_name: str) -> None:
        super(GetCurrentPoseBehavior, self).__init__(name)
        self.name = name
        self.blackboard_pose_name = blackboard_pose_name
        return

    def setup(self, node: Node) -> None:
        """Sends the inital RunFinalApproach goal"""
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n[{self.name}] {x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n[{self.name}] {x}")
        self.error = lambda x: self.node.get_logger().error(f"\n[{self.name}] {x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n[{self.name}] {x}")

        self.info(f"Setting up {self.name}")

        self.client = ActionClient(node=self.node, action_type=GetCurrentPose, action_name="get_current_pose")
        self.client.wait_for_server()

        self.goal_status = None
        self._goal_handle = None
        self._result_future = None

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key=f"{self.blackboard_pose_name}_pose", access=pt.common.Access.WRITE)
        return

    def initialise(self) -> None:
        """Send a goal to the RunFinalApproach action server"""
        self.goal_status = None
        self.goal = GetCurrentPose.Goal()
        self._send_goal_future: Future = self.client.send_goal_async(
            goal=self.goal,
        )
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status == True:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING

    def _send_goal_cb(self, future: Future) -> None:
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.warn(f"{self.name}: Action server not available.")
        else:
            self._result_future: Future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(callback=self._on_result_cb)
        return

    def _on_result_cb(self, future: Future) -> None:
        result: GetCurrentPose.Result = future.result().result
        self.goal_status = result.success
        self.blackboard.rpy_target_pose = result.pose
        return
