#!/usr/bin/env python3
import py_trees as pt
import numpy as np

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import GenerateCylindricalPoses


class GenerateUniformCylindricalPosesBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for the final approach controller action client"""

    def __init__(self, name):
        super(GenerateUniformCylindricalPosesBehavior, self).__init__(name)
        self.name = name
        return

    def setup(self, node: Node):
        """Sends the inital RunFinalApproach goal"""
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n[{self.name}] {x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n[{self.name}] {x}")
        self.error = lambda x: self.node.get_logger().error(f"\n[{self.name}] {x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n[{self.name}] {x}")

        self.info(f"Setting up {self.name}")
        self.client = ActionClient(
            node=self.node, action_type=GenerateCylindricalPoses, action_name="generate_uniform_cylindrical_poses"
        )
        self.client.wait_for_server()

        self.goal_status = None
        self._goal_handle = None
        self._result_future = None

        self.poses = []

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="poses", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose_index", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose", access=pt.common.Access.WRITE)

        return

    def initialise(self):
        """Send a goal to the RunFinalApproach action server"""
        self.goal_status = None
        self.goal = GenerateCylindricalPoses.Goal()

        debug = False
        if debug:
            self.goal.num_radius_poses = 5
            self.goal.num_theta_poses = 18
            self.goal.num_z_poses = 2
            self.goal.radius_range = [0.0, 0.0]
            self.goal.theta_range = [0.0, 0.0]
            self.goal.z_range = [0.0, 0.0]
        else:
            # Edge case settings
            self.goal.num_radius_poses = 5
            self.goal.num_theta_poses = 18
            self.goal.num_z_poses = 5
            self.goal.radius_range = [0.01, 0.07]
            self.goal.theta_range = [0.0, 2 * np.pi]
            self.goal.z_range = [0.0, -0.15]

        self._send_goal_future: Future = self.client.send_goal_async(
            goal=self.goal,
        )
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status == True:
                # self.warn(f"GOAL STATUS: {self.goal_status}")
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING

    def _send_goal_cb(self, future: Future):
        # If there is a result, consider action complete and save result code to be checked in the `update()` method
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.warn(f"{self.name}: Action server not available.")
            # self.feedback_message = "Action server not available."
        else:
            # self.info(f"{self.name}: Goal accepted.")
            self._result_future: Future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(callback=self._on_result_cb)

        return

    def _on_result_cb(self, future: Future):
        result: GenerateCylindricalPoses.Result = future.result().result
        self.goal_status = result.success
        self.poses = result.poses
        self.blackboard.poses = self.poses
        self.blackboard.current_pose = self.blackboard.poses[self.blackboard.current_pose_index]
        return

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
