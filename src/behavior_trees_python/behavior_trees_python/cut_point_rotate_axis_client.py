#!/usr/bin/env python3

import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import RunCutPointRotateAxis


class CutPointRotateAxisControllerBehavior(pt.behaviour.Behaviour):
    """Behaviour wrapper for the cut point rotate axis action client"""
    def __init__(self, name, node):
        super(CutPointRotateAxisControllerBehavior, self).__init__(name)

        self.node = node
        # self.bb = pt.blackboard.Blackboard()
        
        self.info = lambda x: self.node.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.node.get_logger().error(f"\n{x}")
        return
    
    def initialise(self):
        self.client = ActionClient(
            node=self.node,
            action_type=RunCutPointRotateAxis,
            action_name="run_cut_point_rotate_axis"
        )
        self.client.wait_for_server()

        self.goal_status = None
        self._result_future = None

        self.goal = RunCutPointRotateAxis.Goal()
        self._send_goal_future: Future = self.client.send_goal_async(
            goal=self.goal,
        )
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        # If there is a result, consider action complete and save result code to be checked in the `update()` method
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.warn(f"{self.name}: Action server not available.")
        else:
            self.info(f"{self.name}: Goal accepted.")
            self._result_future: Future = goal_handle.get_result_async()
            self._result_future.add_done_callback(callback=self._on_result_cb)
        return

    def _on_result_cb(self, future: Future):
        result: RunCutPointRotateAxis.Result = future.result().result
        self.info(f"{self.name}: Result: {result}")
        self.goal_status = result.success
        return
    
    def update(self):
        if self.goal_status is not None:
            if self.goal_status == True:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
    
    def terminate(self, new_status: pt.common.Status):
        if self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
            _goal_canceled_future: Future = self._goal_handle.cancel_goal_async()
            _goal_canceled_future.add_done_callback(self._on_cancel_cb)

        self.logger.info(f"Terminated with status {new_status}")
        self.client = None
        return

    def _on_cancel_cb(self, future: Future):
        _cancel_result  = future.result().result
        if _cancel_result:
            self.info("Action successfully canceled.")
        else:
            self.error("ACTION NOT CANCELED. ROBOT MAY STILL BE IN OPERATION.")
        return