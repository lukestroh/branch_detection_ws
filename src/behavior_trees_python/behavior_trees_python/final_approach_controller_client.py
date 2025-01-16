#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from final_approach_controller_msgs.action import RunFinalApproach


class FinalApproachControllerBehavior(pt.behaviour.Behaviour):
    """Behavior wrapper for the final approach controller action client"""
    def __init__(self, name, node):
        super(FinalApproachControllerBehavior, self).__init__(name)

        self.node = node
        self.bb = pt.blackboard.Blackboard()

        return
    
    def initialise(self):
        """Sends the inital RunFinalApproach goal"""
        self.client = ActionClient(
            node=self.node,
            action_type=RunFinalApproach,
            action_name="run_final_approach"
        )
        self.client.wait_for_server()

        self.goal_status = None

        self.send_goal_future = self.client.send_goal_async(
            goal=self.goal,
        )
        self.send_goal_future.add_done_callback(self.goal_result_callback)
        return
    
    def goal_callback(self, future):
        res = future.result()
        if res is None or not res.accepted():
            return
        future = res.get_result_async()
        future.add_done_callback(self.goal_result_callback)
        return
    
    def goal_result_callback(self, future):
        # If there is a result, consider action complete and save result code to be checked in the `update()` method
        self.goal_status = future.result().status
        return
    
    def update(self):
        if self.goal_status is not None:
            if self.goal_status == GoalStatus.STATUS_SUCCEEDED:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
    
    def terminate(self, status):
        self.logger.info(f"Terminated with status {status}")
        self.client = None
        # self.bb.set
        return