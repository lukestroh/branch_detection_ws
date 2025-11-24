#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import RunTestReset
from final_approach_controller_msgs.msg import GeneratedPoses

from geometry_msgs.msg import Pose
from std_msgs.msg import Int64


class ResetTestBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for the final approach controller action client"""

    def __init__(self, name):
        super(ResetTestBehavior, self).__init__(name)
        self.name = name
        return

    def setup(self, node: Node):
        self.node = node
        self.info = lambda x: self.node.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.node.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.node.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.node.get_logger().fatal(f"\n{x}")

        self.info(f"Setting up {self.name}")
        self._action_client_run_test_reset = ActionClient(
            node=self.node, action_type=RunTestReset, action_name="run_test_reset"
        )
        self._action_client_run_test_reset.wait_for_server()

        self.goal_status = None
        self._goal_handle = None
        self._result_future = None

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="poses", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose_index", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose", access=pt.common.Access.WRITE)
        # self.blackboard.register_key(key="rpy_target_pose", access=pt.common.Access.WRITE)

        # Publishers for pose information
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._pub_current_pose = node.create_publisher(
            msg_type=Pose, topic="trial_start_pose", qos_profile=self._qos_profile
        )
        self._pub_current_pose_idx = node.create_publisher(
            msg_type=Int64, topic="pose_index", qos_profile=self._qos_profile
        )
        self._pub_generated_poses = node.create_publisher(
            msg_type=GeneratedPoses, topic="generated_start_poses", qos_profile=self._qos_profile
        )
        # self._pub_rpy_target_pose = node.create_publisher(
        #     msg_type=Pose, topic="rpy_target_pose", qos_profile=self._qos_profile
        # )
        return

    def initialise(self):
        """Send a goal to the RunFinalApproach action server"""
        self.goal_status = None
        self.goal = RunTestReset.Goal()
        poses = self.blackboard.poses
        self.goal.pose_idx = self.blackboard.current_pose_index
        self.goal.pose = self.blackboard.current_pose

        self._pub_current_pose.publish(msg=self.blackboard.current_pose)
        self._pub_current_pose_idx.publish(msg=Int64(data=self.blackboard.current_pose_index))
        self._pub_generated_poses.publish(msg=GeneratedPoses(poses=self.blackboard.poses))
        # if self.blackboard.rpy_target_pose:
        #     self._pub_rpy_target_pose.publish(msg=self.blackboard.rpy_target_pose)

        self._send_goal_future: Future = self._action_client_run_test_reset.send_goal_async(goal=self.goal)
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        # If there is a result, consider action complete and save result code to be checked in the `update()` method
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.warn(f"{self.name}: Action server not available.")
        else:
            self._result_future: Future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(callback=self._on_result_cb)
        return

    def _on_result_cb(self, future: Future):
        result: RunTestReset.Result = future.result().result
        # self.info(f"{self.name}: Result: {result}")
        self.goal_status = result.success
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status == True:
                # self.warn("GOAL STATUS SUCCESS")
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
