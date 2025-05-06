#!/usr/bin/env python3
"""

"""
import argparse
import asyncio
import functools as ft
import operator
import py_trees
import py_trees_ros
import sys
from threading import Lock

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node

from vl6180_msgs.msg import Vl6180FilteredStamped

from behavior_trees_python.behaviors.process_io_behavior import ProcessIOBehavior


class IOTreeNode(Node):
    def __init__(self, asyncio_loop):
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Thread locks
        # self._bb_tof_data_lock = Lock()
        self.asyncio_loop = asyncio_loop

        super().__init__(node_name="io_tree_node")

        self.bb = py_trees.blackboard.Client(name="IOTreeNode")

        # Behavior tree setup
        self.tree = self.create_behavior_tree_ros()
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(ft.partial(self.post_tick_handler, self.snapshot_visitor))
        self.post_tick_last_print_time = self.get_clock().now()
        return

    def create_behavior_tree_ros(self) -> py_trees.trees.BehaviourTree:
        """Construct the behavior tree."""
        root = py_trees.composites.Sequence("Root", memory=False)
        process_io = ProcessIOBehavior(name="Process IO", asyncio_loop=self.asyncio_loop)
        root.add_children([process_io])
        tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
        tree.setup(node=self)
        return tree

    def description(self):
        """Print description about the program"""
        # content =
        # self.get_logger().info(f"{py_trees.console.colours}")
        # self.get_logger().info(f'{py_trees.console.has_colours}')
        # py_trees.console.banner("FinalApproachControllerTree\n\n")
        msg = "ProcessIOTree"
        self.info(
            py_trees.console.green
            + 80 * "*"
            + py_trees.console.reset
            + "\n"
            + py_trees.console.green
            + "* "
            + py_trees.console.bold_white
            + msg.center(80)
            + py_trees.console.reset
            + "\n"
            + py_trees.console.green
            + 80 * "*"
            + py_trees.console.reset
        )
        return

    def post_tick_handler(
        self, snapshot_visitor: py_trees.visitors.SnapshotVisitor, behavior_tree: py_trees.trees.BehaviourTree
    ):
        """Write the tree snapshot to the console."""
        if self.get_clock().now() - self.post_tick_last_print_time > Duration(seconds=1.0):
            self.info("\n" + py_trees.display.unicode_blackboard())
            self.post_tick_last_print_time = self.get_clock().now()
        return


def main():
    rclpy.init()
    async_loop = asyncio.new_event_loop()
    io_tree_node = IOTreeNode(asyncio_loop=async_loop)
    executor = MultiThreadedExecutor()
    io_tree_node.tree.tick_tock(
        period_ms=5.0,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
    )
    try:
        rclpy.spin(io_tree_node, executor=executor)
    except Exception as e:
        io_tree_node.error(f"{e}")
    finally:
        io_tree_node.destroy_node()
        rclpy.shutdown()

    return
