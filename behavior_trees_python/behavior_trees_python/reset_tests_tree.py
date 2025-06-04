#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tof_msgs.msg import TofStamped
from visualization_msgs.msg import Marker

import argparse
import functools as ft
import operator
import py_trees
import py_trees_ros
import sys
from threading import Lock
import pprint as pp

from behavior_trees_python.behaviors.check_continue_experiment_behavior import CheckContinueExperimentBehavior
from behavior_trees_python.behaviors.cut_point_rotate_axis_behavior import CutPointRotateAxisControllerBehavior
from behavior_trees_python.behaviors.final_approach_controller_behavior import FinalApproachControllerBehavior
from behavior_trees_python.behaviors.find_branch_roll_wrist_behavior import FindBranchRollWristControllerBehavior
from behavior_trees_python.behaviors.generate_poses_behavior import GeneratePosesBehavior
from behavior_trees_python.behaviors.generate_rpy_projected_poses import GenerateRpyProjectedPosesBehavior
from behavior_trees_python.behaviors.generate_uniform_cylindrical_poses_behavior import (
    GenerateUniformCylindricalPosesBehavior,
)
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

        # Blackboard setup
        self.bb = py_trees.blackboard.Client(name="ResetTreeBlackboard")
        self.bb.register_key(key="trials_done", access=py_trees.common.Access.WRITE)
        self.bb.trials_done = False
        self.bb.register_key(key="d_tof0", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="d_tof1", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="current_pose_index", access=py_trees.common.Access.WRITE)
        self.bb.current_pose_index = 0
        self.bb.register_key(key="current_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='initial_joint_position', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="poses", access=py_trees.common.Access.WRITE)
        self.bb.current_pose = Pose()
        self.bb.register_key(key="poses_in_queue", access=py_trees.common.Access.WRITE)

        # Behavior tree setup
        self.tree: py_trees_ros.trees.BehaviourTree = self.create_behavior_tree_ros()
        self.snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(ft.partial(self.post_tick_handler, self.snapshot_visitor))
        self.tree.add_visitor(self.snapshot_visitor)
        self.last_tree_snapshot = None

        # Callback groups
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Subscribers
        self._sub_tof_filtered = self.create_subscription(
            TofStamped, "/vl53l4cd/filtered", self._sub_cb_tof_filtered, 10
        )
        self._sub_joint_states = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self._sub_cb_joint_states,
            callback_group=self._reentrant_cb_group,
            qos_profile=1,
        )

        # Class attributes
        self._initial_joint_position = None
        self._initial_pose = None
        return

    def _sub_cb_tof_filtered(self, msg: TofStamped):
        if msg.dev_id == 0:
            self.bb.d_tof0 = msg.data[0]
        elif msg.dev_id == 1:
            self.bb.d_tof1 = msg.data[0]
        return
    
    def _sub_cb_joint_states(self, msg: JointState):
        if self._initial_joint_position is None:
            self._initial_joint_position = msg.position
            self.bb.initial_joint_position = self._initial_joint_position
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
        generate_uniform_cylindrical_poses_behavior = GenerateUniformCylindricalPosesBehavior(
            name="generate_uniform_cylindrical_poses_behavior"
        )
        generate_rpy_projected_poses_behavior = GenerateRpyProjectedPosesBehavior(name="generate_rpy_projected_poses_behavior")
        iterate_poses_behavior = IteratePosesBehavior(name="iterate_poses_behavior")
        check_continue_experiment_behavior = CheckContinueExperimentBehavior(name="check_continue_experiment_behavior")
        reset_test_behavior = ResetTestBehavior(name="reset_test_behavior")
        cut_point_rotate_axis_behavior = CutPointRotateAxisControllerBehavior(name="cut_point_rotate_axis_behavior")
        final_approach_behavior = FinalApproachControllerBehavior(name="final_approach_behavior_behavior")
        start_bag_record_behavior = StartBagRecordBehavior(name="start_bag_record_behavior")
        stop_bag_record_behavior = StopBagRecordBehavior(name="stop_bag_record_behavior")

        
        find_branch_roll_wrist_behavior = FindBranchRollWristControllerBehavior(
            name="find_branch_roll_wrist_behavior",
        )
        find_branch_roll_wrist_retry = py_trees.decorators.Retry(
            name="find_branch_roll_wrist_retry",
            child=find_branch_roll_wrist_behavior,
            num_failures=1,
        )
        find_branch_selector = py_trees.composites.Selector(
            name="find_branch_controller_selector",
            memory=True,
        )
        find_branch_selector.add_children([find_branch_roll_wrist_retry])
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
        grouped_controller_failure_is_success = py_trees.decorators.FailureIsSuccess(
            name="grouped_controller_failure_is_success", child=grouped_controller_sequence
        )
        iterate_trials_sequence = py_trees.composites.Sequence(
            name="iterate_trials_sequence",
            memory=True,
            children=[
                check_continue_experiment_behavior,
                start_bag_record_behavior,
                reset_test_behavior,
                grouped_controller_failure_is_success,
                stop_bag_record_behavior,
                iterate_poses_behavior,
            ],
        )
        iterate_trials_success_is_running = py_trees.decorators.SuccessIsRunning(
            name="iterate_trials_success_is_running", child=iterate_trials_sequence
        )

        #####################
        # Root sequence
        #####################
        # Run find_branch_selector until success, then run align_and_approach_sequence
        root_sequence = py_trees.composites.Sequence(
            name="root_sequence",
            memory=True,
            # children=[align_and_approach_sequence]
            children=[generate_uniform_cylindrical_poses_behavior, iterate_trials_success_is_running],
        )
        root_sequence_failure_is_success = py_trees.decorators.FailureIsSuccess(
            name="root_sequence_failure_is_success", child=root_sequence
        )
        root = py_trees.decorators.OneShot(
            name="root", child=root_sequence_failure_is_success, policy=py_trees.common.OneShotPolicy.ON_COMPLETION
        )
        # root.setup_with_descendants()
        tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
        tree.setup(node=self)

        self.description()
        return tree

    def filtered_blackboard_display(self, exclude_keys: list[str] = []):
        bb = py_trees.blackboard.Blackboard()
        lines = [py_trees.console.bold_blue + "Blackboard:" + py_trees.console.reset]
        for key, value in sorted(bb.storage.items()):
            if key in exclude_keys:
                continue
            lines.append(
                py_trees.console.bold_green
                + f"\t{key}: "
                + py_trees.console.reset
                + py_trees.console.yellow
                + pp.pformat(value)
                + py_trees.console.reset
            )
        lines.append("\n")
        return "\n".join(lines)

    def post_tick_handler(
        self, snapshot_visitor: py_trees.visitors.SnapshotVisitor, behavior_tree: py_trees.trees.BehaviourTree
    ):
        """Write the tree snapshot to the console."""
        # snapshot_visitor.
        # if self.get_clock().now() - self._last_log_time > Duration(seconds=0.005):
        #     self.info("\n")
        current_snapshot = {_id: status for _id, status in snapshot_visitor.visited.items()}

        # self.info(current_snapshot)

        # if self.get_clock().now() - self._last_log_time > Duration(seconds=1.0):
        if current_snapshot != self.last_tree_snapshot:
            self.info(
                py_trees.display.unicode_tree(
                    root=behavior_tree.root,
                    visited=snapshot_visitor.visited,
                    previously_visited=snapshot_visitor.previously_visited,
                    show_status=True,
                )
                + "\n"
                # + py_trees.display.unicode_blackboard()
                + self.filtered_blackboard_display(exclude_keys=["/poses"])
            )
            # self._last_log_time = self.get_clock().now()
            self.last_tree_snapshot = current_snapshot

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
