#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import RunFinalApproach


class FinalApproachControllerBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for the final approach controller action client"""

    def __init__(self, name):
        super(FinalApproachControllerBehavior, self).__init__(name)
        self.name = name

        return

    def setup(self, node):
        """Sends the inital RunFinalApproach goal"""
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n[{self.name}] {x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n[{self.name}] {x}")
        self.error = lambda x: self.node.get_logger().error(f"\n[{self.name}] {x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n[{self.name}] {x}")

        self.info(f"Setting up {self.name}")
        self.client = ActionClient(node=self.node, action_type=RunFinalApproach, action_name="run_final_approach")
        self.client.wait_for_server()

        self.goal_status = None
        self._goal_handle = None
        self._result_future = None

        self.blackboard = pt.blackboard.Client(name=self.name)

        return

    def initialise(self):
        """Send a goal to the RunFinalApproach action server"""
        self.goal_status = None
        self.goal = RunFinalApproach.Goal()
        self._send_goal_future: Future = self.client.send_goal_async(
            goal=self.goal,
        )
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    # def goal_callback(self, future):
    #     res = future.result()
    #     if res is None or not res.accepted():
    #         return
    #     future = res.get_result_async()
    #     future.add_done_callback(self.goal_result_callback)
    #     return

    def _send_goal_cb(self, future: Future):
        # If there is a result, consider action complete and save result code to be checked in the `update()` method
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.warn(f"{self.name}: Action server not available.")
            # self.feedback_message = "Action server not available."
        else:
            self.info(f"{self.name}: Goal accepted.")
            self._result_future: Future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(callback=self._on_result_cb)
        # self.goal_status = goal_handle.status
        # self.info((f"{self.goal_status}"))
        return

    def _on_result_cb(self, future: Future):
        result: RunFinalApproach.Result = future.result().result
        self.info(f"{self.name}: Result: {result}")
        self.goal_status = result.success
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status == True:
                self.warn("GOAL STATUS SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        if self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
            _goal_canceled_future: Future = self._goal_handle.cancel_goal_async()
            _goal_canceled_future.add_done_callback(self._on_cancel_cb)

        self.logger.info(f"Terminated with status {new_status}")
        # self.client = None
        return

    def _on_cancel_cb(self, future: Future):
        _cancel_result = future.result().result
        if _cancel_result:
            self.info("Action successfully canceled.")
        else:
            self.error("ACTION NOT CANCELED. ROBOT MAY STILL BE IN OPERATION.")
        return
