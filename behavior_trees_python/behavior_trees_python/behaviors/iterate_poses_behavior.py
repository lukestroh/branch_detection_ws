#!/usr/bin/env python3

import py_trees as pt
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64


class IteratePosesBehavior(pt.behaviour.Behaviour):
    def __init__(self, name):
        super(IteratePosesBehavior, self).__init__(name)
        self.name = name
        return

    def setup(self, node: Node):
        self.node = node

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="current_pose_index", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="current_pose", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="poses", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="trials_done", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="poses_in_queue", access=pt.common.Access.WRITE)

        return

    def initialise(self):
        self.total_poses = len(self.blackboard.poses)
        self.blackboard.poses_in_queue = self.total_poses - (self.blackboard.current_pose_index + 1)
        return

    def update(self):
        self.blackboard.current_pose_index += 1
        self.blackboard.poses_in_queue = self.total_poses - (self.blackboard.current_pose_index + 1)

        if self.blackboard.current_pose_index >= len(self.blackboard.poses):
            self.blackboard.trials_done = True
        else:
            self.blackboard.current_pose = self.blackboard.poses[self.blackboard.current_pose_index]
        return pt.common.Status.SUCCESS
