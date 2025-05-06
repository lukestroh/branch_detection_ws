#!/usr/bin/env python3

import argparse
import functools as ft
import operator
import py_trees
import py_trees_ros
import sys
from threading import Lock

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from tof_msgs.msg import TofStamped
from geometry_msgs.msg import Pose
from vl6180_msgs.msg import Vl6180FilteredStamped

from behavior_trees_python.behaviors.cut_point_rotate_axis_client import CutPointRotateAxisControllerBehavior
from behavior_trees_python.behaviors.final_approach_controller_client import FinalApproachControllerBehavior
from behavior_trees_python.behaviors.find_branch_roll_wrist_client import FindBranchRollWristControllerBehavior
from behavior_trees_python.behaviors.generate_poses_behavior import GeneratePosesBehavior
from behavior_trees_python.behaviors.iterate_poses_behavior import IteratePosesBehavior
from behavior_trees_python.behaviors.reset_test_behavior import ResetTestBehavior
from behavior_trees_python.behaviors.start_bag_record_behavior import StartBagRecordBehavior
from behavior_trees_python.behaviors.stop_bag_record_behavior import StopBagRecordBehavior


class ResetTestTreeNode(Node):
    def __init__(self):
        super().__init__(node_name="reset_tests_tree_node")
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Thread locks
        # self._bb_tof_data_lock = Lock()

        # Parameters
        self._param_record_loc = (
            self.node.declare_parameter(name="record_loc", value=Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )

        # Blackboard setup
        self.bb = py_trees.blackboard.Client(name="ResetTestTreeNode")
        self.bb.register_key(key="d_tof0", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="d_tof1", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="current_pose_index", access=py_trees.common.Access.WRITE)
        self.bb.current_pose_index = 0
        self.bb.register_key(key="current_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="poses", access=py_trees.common.Access.WRITE)
        self.bb.current_pose = Pose()

        # Behavior tree setup
        self.tree: py_trees_ros.trees.BehaviourTree = self.create_behavior_tree_ros()
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(ft.partial(self.post_tick_handler, self.snapshot_visitor))

        # Subscribers
        self._sub_tof_filtered = self.create_subscription(
            TofStamped, "/vl53l4cd/filtered", self._sub_cb_tof_filtered, 10
        )

        self._last_log_time = self.get_clock().now()

        return

    def _sub_cb_tof_filtered(self, msg: TofStamped):
        if msg.dev_id == 0:
            self.bb.d_tof0 = msg.data[0]
        elif msg.dev_id == 1:
            self.bb.d_tof1 = msg.data[0]
        return

    def description(self):
        """Print description about the program"""
        # content =
        # self.get_logger().info(f"{py_trees.console.colours}")
        # self.get_logger().info(f'{py_trees.console.has_colours}')
        # py_trees.console.banner("FinalApproachControllerTree\n\n")
        msg = "ResetTestTree"
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

    def create_behavior_tree_ros(self):
        """
        Root: Either_or deciding status of tof readings.
        find_branch_selector: Set of find branch controllers, quits at first success
        align_and_approach_sequence: Runs both to SUCCESS/FAILURE

        A selector executes each of its child behaviours in turn until one of them succeeds (at which point it itself returns ~py_trees.common.Status.RUNNING or ~py_trees.common.Status.SUCCESS
        """

        # Behaviors
        generate_poses_behavior = GeneratePosesBehavior(name="generate_poses_behavior")
        # iterate_poses_behavior = IteratePosesBehavior(name='iterate_poses_behavior')
        reset_test_behavior = ResetTestBehavior(name="reset_test_behavior")
        cut_point_rotate_axis_behavior = CutPointRotateAxisControllerBehavior(name="cut_point_rotate_axis_client")
        final_approach_behavior = FinalApproachControllerBehavior(name="final_approach_behavior_client")

        start_bag_record_behavior = StartBagRecordBehavior(name="start_bag_record_client")
        stop_bag_record_behavior = StopBagRecordBehavior(name="stop_bag_record_client")

        # Find branch roll wrist
        find_branch_roll_wrist_behavior = FindBranchRollWristControllerBehavior(
            name="find_branch_roll_wrist_client",
        )
        find_branch_roll_wrist_retry = py_trees.decorators.Retry(
            name="find_branch_roll_wrist_retry",
            child=find_branch_roll_wrist_behavior,
            num_failures=1,
        )
        # find_branch_roll_wrist_blackboard = py_trees.decorators.StatusToBlackboard(
        #     name='find_branch_roll_wrist_blackboard',
        #     child=find_branch_roll_wrist_retry,
        #     variable_name='find_branch_roll_wrist'
        # )

        find_branch_selector = py_trees.composites.Selector(
            name="find_branch_controller_selector",
            memory=True,
        )

        # Find branch selector
        find_branch_selector.add_children([find_branch_roll_wrist_retry])
        # Align and approach sequence
        align_and_approach_sequence = py_trees.composites.Sequence(
            name="align_and_approach_sequence",
            memory=True,
            children=[cut_point_rotate_axis_behavior, final_approach_behavior],
        )

        grouped_controller_sequence = py_trees.composites.Sequence(
            name="grouped_controller_sequence",
            memory=True,
            children=[find_branch_selector, align_and_approach_sequence],
        )

        # grouped_controller_failure_is_running = py_trees.decorators.FailureIsRunning(
        #     name='grouped_controller_failure_is_running',
        #     child=grouped_controller_sequence
        # )

        # grouped_controller_everything_is_running = py_trees.decorators.SuccessIsRunning(
        #     name='grouped_controller_everything_is_running',
        #     child=grouped_controller_failure_is_running
        # )

        # iterate_poses_success_is_running = py_trees.decorators.SuccessIsRunning(
        #     name='iterate_poses_success_is_running',
        #     child=iterate_poses_behavior
        # )

        iterate_poses_sequence = py_trees.composites.Sequence(
            name="iterate_poses_sequence",
            memory=True,
            children=[
                start_bag_record_behavior,
                reset_test_behavior,
                grouped_controller_sequence,
                stop_bag_record_behavior,
            ],
        )
        iterate_poses_failure_is_running = py_trees.decorators.FailureIsRunning(
            name="iterate_poses_failure_is_running", child=iterate_poses_sequence
        )
        iterate_poses_everything_is_running = py_trees.decorators.SuccessIsRunning(
            name="iterate_poses_success_is_running", child=iterate_poses_failure_is_running
        )
        # iterate_poses_retry = py_trees.decorators.Retry(
        #     name='iterate_poses_retry',
        #     child=iterate_poses_success_is_running,
        #     num_failures=31
        # )

        #####################
        # Root sequence
        #####################
        # Run find_branch_selector until success, then run align_and_approach_sequence
        root_sequence = py_trees.composites.Sequence(
            name="root_sequence",
            memory=True,
            # children=[align_and_approach_sequence]
            children=[generate_poses_behavior, iterate_poses_everything_is_running],
        )
        root = py_trees.decorators.OneShot(
            name="root", child=root_sequence, policy=py_trees.common.OneShotPolicy.ON_COMPLETION
        )
        # root.setup_with_descendants()
        tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
        tree.setup(node=self)

        self.description()
        return tree

    def post_tick_handler(
        self, snapshot_visitor: py_trees.visitors.SnapshotVisitor, behavior_tree: py_trees.trees.BehaviourTree
    ):
        """Write the tree snapshot to the console."""
        # snapshot_visitor.
        # if self.get_clock().now() - self._last_log_time > Duration(seconds=0.005):
        #     self.info("\n")
        if self.get_clock().now() - self._last_log_time > Duration(seconds=1.0):
            self.info(
                py_trees.display.unicode_tree(
                    root=behavior_tree.root,
                    visited=snapshot_visitor.visited,
                    previously_visited=snapshot_visitor.previously_visited,
                    show_status=True,
                )
                + "\n"
                + py_trees.display.unicode_blackboard()
            )
            self._last_log_time = self.get_clock().now()

        if behavior_tree.root.status == py_trees.common.Status.SUCCESS:
            self.info(f"Exiting with status {behavior_tree.root.status}")
            behavior_tree.shutdown()
            sys.exit(0)
        elif behavior_tree.root.status == py_trees.common.Status.FAILURE:
            self.error(f"Exiting with status {behavior_tree.root.status}")
            behavior_tree.shutdown()
            sys.exit(0)

        return


def main():
    rclpy.init()
    reset_test_tree_node = ResetTestTreeNode()
    # args = fa_tree_node.cli_arg_parser().parse_args()
    # if args.interactive:
    #     ...
    #     py_trees.console.read_single_keypress()
    reset_test_tree_node.tree.tick_tock(period_ms=5.0)
    try:
        rclpy.spin(reset_test_tree_node)
    except KeyboardInterrupt:
        # TODO: Need to find a way to send cancel goal to running action from here.
        # fa_tree_node.
        reset_test_tree_node.info("Shutting down")
        # fa_tree_node.info(f"{fa_tree_node.tree.visitors[0].}")
        # fa_tree_node.tree.visitors[0][1].terminate(new_status=py_trees.common.Status.FAILURE)
    except ExternalShutdownException:
        sys.exit(0)
    finally:
        reset_test_tree_node.destroy_node()

    return
