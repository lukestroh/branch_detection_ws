#!/usr/bin/env python3
"""The FinalApproachTree should be dependent on the starting information of the ToF sensors. If the sensors read a branch, we should be able to run CutPointRotateAxis and FinalApproach.

If we cannot see the branch at the beginning of execution, we need to find the branch. This should start with the low-cost controller, FindBranchRotateWrist

"""
import argparse
import functools as ft
import operator
import py_trees
import py_trees_ros
from threading import Lock

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from vl6180_msgs.msg import Vl6180FilteredStamped

from behavior_trees_python.final_approach_controller_client import FinalApproachControllerBehavior

class FinalApproachTreeNode(Node):
    def __init__(self):
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Thread locks
        # self._bb_tof_data_lock = Lock()

        super().__init__(node_name="final_approach_tree_node")

        # Sensor params # TODO: Get from param file
        self.vl6180_far_plane = 0.200  # 0.19 based on testing, but give it small window
        self.vl6180_precision = 0.001

        # Blackboard setup
        self.bb = py_trees.blackboard.Client(name="FinalApproachTreeNode")
        self.bb.register_key(key="d_tof0", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="d_tof1", access=py_trees.common.Access.WRITE)

        # Behavior tree setup
        self.tree = self.create_behavior_tree_ros()
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            ft.partial(self.post_tick_handler, self.snapshot_visitor)
        )

        # Subscribers
        self._sub_tof_filtered = self.create_subscription(
            Vl6180FilteredStamped,
            "/vl6180/filtered",
            self._sub_cb_tof_filtered,
            10
        )
        
        self._last_log_time = self.get_clock().now()

        

        return

    def _sub_cb_tof_filtered(self, msg: Vl6180FilteredStamped):
        # self.d_tof0 = msg.data[0]
        # self.d_tof1 = msg.data[1]
        self.bb.d_tof0 = msg.data[0]
        self.bb.d_tof1 = msg.data[1]
        return

    def description(self):
        """Print description about the program"""
        # content =
        # self.get_logger().info(f"{py_trees.console.colours}")
        self.get_logger().info(f'{py_trees.console.has_colours}')
        # py_trees.console.banner("FinalApproachControllerTree\n\n")
        msg = "FinalApproachControllerTree"
        self.info(py_trees.console.green + 80 * "*" + py_trees.console.reset + '\n' + py_trees.console.green + "* " + py_trees.console.bold_white + msg.center(80) + py_trees.console.reset + "\n" + py_trees.console.green + 80 * "*" + py_trees.console.reset)
        return
        
    def create_behavior_tree_ros(self):
        """
        Root: Either_or deciding status of tof readings.
        find_branch_selector: Set of find branch controllers, quits at first success
        align_and_approach_sequence: Runs both to SUCCESS/FAILURE

        A selector executes each of its child behaviours in turn until one of them succeeds (at which point it itself returns ~py_trees.common.Status.RUNNING or ~py_trees.common.Status.SUCCESS"""

        # Behaviors
        final_approach_behavior = FinalApproachControllerBehavior(name="fac_client", node=self)

        find_branch_selector = py_trees.composites.Selector( 
            name="find_branch_controller_selector",
            memory=True,
        )

        find_branch_selector.add_children([
            
        ])

        align_and_approach_sequence = py_trees.composites.Sequence(
            name="align_and_approach_sequence",
            memory=True,
            children=[final_approach_behavior]
        )

        # root = py_trees.decorators.OneShot(
        #     name="root",
        #     child=find_branch_selector,
        #     policy=py_trees.common.OneShotPolicy.ON_COMPLETION # Allow some controllers to fail. If the FindBranch controllers fail, then the whole system fails.
        # )
        root = py_trees.composites.Sequence(
            name="root_sequence",
            memory=True,
            children=[find_branch_selector, align_and_approach_sequence]
        )
        tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
        tree.setup(node=self)

        # _task_no_sensor_readings = py_trees.behaviours.Success(name=f"Task1: Both tofs >= {self.vl6180_far_plane}")
        # _task_one_sensor_reading = py_trees.behaviours.Success(name=f"Task2: One tof < {self.vl6180_far_plane}")
        # _task_both_sensor_readings = py_trees.behaviours.Success(name=f"Task3: Both tof < {self.vl6180_far_plane}")

        conditions = [
            py_trees.common.ComparisonExpression(variable="d_tof0", value=self.vl6180_far_plane, operator=operator.gt)
        ]

        

        self.description()
        return tree

    def post_tick_handler(
        self,
        snapshot_visitor: py_trees.visitors.SnapshotVisitor,
        behavior_tree: py_trees.trees.BehaviourTree
    ):
        """Write the tree snapshot to the console."""
        if self.get_clock().now() - self._last_log_time > Duration(seconds=3):
            self.info("\n")
            self.info(py_trees.display.unicode_tree(
                root=behavior_tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.previously_visited
            ) + "\n" + py_trees.display.unicode_blackboard())
            self._last_log_time = self.get_clock().now()
        
        # self.info(py_trees.display.unicode_blackboard())
        return
        
    def cli_arg_parser(self) -> argparse.ArgumentParser:
        """Process CLI args"""
        parser = argparse.ArgumentParser(
            description=self.description(),
            formatter_class=argparse.RawDescriptionHelpFormatter
        )
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '-i',
            '--interactive',
            action="store_true",
            help="pause and wait for keypress at each tick"
        )
        return parser


def main():
    rclpy.init()
    fa_tree_node = FinalApproachTreeNode()
    # args = fa_tree_node.cli_arg_parser().parse_args()
    # if args.interactive:
    #     ...
    #     py_trees.console.read_single_keypress()
    # fa_tree_node.tree.tick_tock(period_ms=5.0)
    # rclpy.spin(fa_tree_node)
    while rclpy.ok():
        fa_tree_node.tree.tick()
        rclpy.spin_once(node=fa_tree_node)
        
        fa_tree_node.get_clock().sleep_for(Duration(nanoseconds=int(0.05 * 1e9)))
        
    rclpy.shutdown()
    return
