#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import RunTestReset

from geometry_msgs.msg import Pose



class ResetTestBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for the final approach controller action client"""

    def __init__(self, name):
        super(ResetTestBehavior, self).__init__(name)


        return

    def setup(self, node):
        """Sends the inital RunTestReset goal"""
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.node.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n{x}")

        self.info("Setting up ResetTestBehavior")
        self.client = ActionClient(node=self.node, action_type=RunTestReset, action_name="run_test_reset")
        self.client.wait_for_server()

        self.goal_status = None
        self._goal_handle = None
        self._result_future = None

        return
    

    def initialise(self):
        """Send a goal to the RunFinalApproach action server"""
        self.goal_status = None
        self.goal = RunTestReset.Goal()
        self.goal.pose = Pose()
        self.goal.pose.position = ...

        """
        position:
            x: -0.7238641982640622
            y: 0.6336055303079968
            z: 1.5642332165941444
        orientation:
            x: -0.2705980500992775
            y: -0.6532814825059136
            z: 0.653281482371788
            w: 0.2705980500437208
        """
        self._send_goal_future: Future = self.client.send_goal_async(
            goal=self.goal,
        )
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return
    
    def update(self):
        if self.goal_status is not None:
            if self.goal_status == True:
                self.warn("GOAL STATUS SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING