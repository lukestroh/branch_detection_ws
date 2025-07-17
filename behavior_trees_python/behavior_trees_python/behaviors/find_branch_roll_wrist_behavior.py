#!/usr/bin/env python3

import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import RunFindBranchRollWrist


class FindBranchRollWristControllerBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for the find branch rotate wrist action client"""

    def __init__(self, name):
        super(FindBranchRollWristControllerBehavior, self).__init__(name)
        self.name = name
        return

    def setup(self, node: Node):
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n[{self.name}] {x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n[{self.name}] {x}")
        self.error = lambda x: self.node.get_logger().error(f"\n[{self.name}] {x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n[{self.name}] {x}")

        self.info(f"Setting up {self.name}")

        self.client = ActionClient(
            node=self.node, action_type=RunFindBranchRollWrist, action_name="run_find_branch_roll_wrist"
        )

        while not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn("FindBranchRollWristController server not available.")

        self.goal_status = None
        self._result_future = None
        self._goal_handle = None

        self.node.get_logger().info("Finished FindBranchRollWristControllerBehavior setup.")

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_pose_index", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key='initial_joint_position', access=pt.common.Access.WRITE)
        # self.node.get_clock().sleep_for(Duration(seconds=0.1))
        return

    def initialise(self):
        """Send the action server a goal at the first tick."""
        self.goal_status = None
        self.goal = RunFindBranchRollWrist.Goal()
        try:
            self.goal.initial_joint_position = self.blackboard.initial_joint_position
        except KeyError as e:
            self.node.get_logger().info(f"{e}, initial joint position not yet on blackboard.")
            self.node.get_clock().sleep_for(Duration(seconds=1.0))
            # self.initialise()
            
        self._send_goal_future: Future = self.client.send_goal_async(
            goal=self.goal,
        )
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        # If there is a result, consider action complete and save result code to be checked in the `update()` method
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.warn(f"{self.name}: Action server not available.")
        else:
            # self.info(f"{self.name}: Goal accepted.")
            self._result_future: Future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(callback=self._on_result_cb)
        return

    def _on_result_cb(self, future: Future):
        result: RunFindBranchRollWrist.Result = future.result().result
        self.goal_status = result.success
        return

    def update(self):
        if self.goal_status is not None:
            # self.blackboard.current_pose_index += 1 # TODO: move to different behavior
            if self.goal_status == True:
                # self.warn(f"GOAL STATUS: {self.goal_status}")
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        if self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
            _goal_canceled_future: Future = self._goal_handle.cancel_goal_async()
            _goal_canceled_future.add_done_callback(self._on_cancel_cb)

        self.logger.info(f"Terminated with status {new_status}")
        return

    def _on_cancel_cb(self, future: Future):
        _cancel_result = future.result().result
        if _cancel_result:
            self.info("Action successfully canceled.")
        else:
            self.error("ACTION NOT CANCELED. ROBOT MAY STILL BE IN OPERATION.")
        return
