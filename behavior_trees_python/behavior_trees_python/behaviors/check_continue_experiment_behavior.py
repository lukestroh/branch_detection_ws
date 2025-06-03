#!/usr/bin/env python3
import py_trees as pt


class CheckContinueExperimentBehavior(pt.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckContinueExperimentBehavior, self).__init__(name)
        self.name = name
        return

    def setup(self, node):
        self.node = node

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="trials_done", access=pt.common.Access.WRITE)
        return

    def initialise(self):
        return

    def update(self):
        if self.blackboard.trials_done:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.SUCCESS
