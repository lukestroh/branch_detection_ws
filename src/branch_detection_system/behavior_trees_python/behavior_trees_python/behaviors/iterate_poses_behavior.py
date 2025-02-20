#!/usr/bin/env python3

import py_trees


class IteratePosesBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(IteratePosesBehavior, self).__init__(name)
        return
    
    def setup(self, node):
        self.node = node

        self.blackboard = py_trees.blackboard.Blackboard()

        return
    
    def initialise(self):
        return
    
    def update(self):
        poses = self.blackboard.get('poses')
        index = self.blackboard.get('current_pose_index')

        if index >= len(poses):
            return py_trees.common.Status.SUCCESS
        
        self.blackboard.set('current_pose_index', index + 1)
        self.blackboard.set('current_pose', poses[index])
        

        return py_trees.common.Status.RUNNING