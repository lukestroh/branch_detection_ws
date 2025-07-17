#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from final_approach_controller.tf_node import TFNode

from final_approach_controller_msgs.action import GeneratePoses, GenerateCylindricalPoses, GenerateRpyProjectedPoses
from final_approach_controller_msgs.msg import GeneratedPoses
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from std_srvs.srv import Trigger

from copy import deepcopy
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
import secrets


class GeneratePosesServiceNode(TFNode):
    def __init__(self):
        super().__init__(node_name="generate_poses_action_server_node")
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        self.fatal = lambda x: self.get_logger().fatal(f"\n{x}")

        # Parameters
        self._param_robot_eef_part = (
            self.declare_parameter("robot_eef_part", value=Parameter.Type.STRING).get_parameter_value().string_value
        )
        self._param_robot_base_part = (
            self.declare_parameter("robot_base_part", value=Parameter.Type.STRING).get_parameter_value().string_value
        )
        # self.warn(self._param_robot_eef_part.get_parameter_value().string_value)

        # Callback groups
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Action servers
        self._action_server_generate_uniform_poses = ActionServer(
            node=self,
            action_name="/generate_uniform_poses",
            action_type=GeneratePoses,
            goal_callback=self._action_goal_cb_generate_uniform_poses,
            cancel_callback=self._action_cancel_cb_generate_uniform_poses,
            execute_callback=self._action_execute_cb_generate_uniform_poses,
            callback_group=self._reentrant_cb_group,
        )

        self._action_server_generate_uniform_cylindrical_poses = ActionServer(
            node=self,
            action_name="/generate_uniform_cylindrical_poses",
            action_type=GenerateCylindricalPoses,
            goal_callback=self._action_goal_cb_generate_uniform_cylindrical_poses,
            cancel_callback=self._action_cancel_cb_generate_uniform_cylindrical_poses,
            execute_callback=self._action_execute_cb_generate_uniform_cylindrical_poses,
            callback_group=self._reentrant_cb_group,
        )

        self._action_server_generate_uniform_rpy_projected_poses = ActionServer(
            node=self,
            action_name="/generate_rpy_projected_poses",
            action_type=GenerateRpyProjectedPoses,
            goal_callback=self._action_goal_cb_generate_rpy_projected_poses,
            cancel_callback=self._action_cancel_cb_generate_rpy_projected_poses,
            execute_callback=self._action_execute_cb_generate_rpy_projected_poses,
            callback_group=self._reentrant_cb_group,
        )

        # Publishers
        self._pub_generated_poses = self.create_publisher(
            msg_type=GeneratedPoses, topic="generated_start_poses", qos_profile=5
        )

        # Class attrb
        self.tf_start__tool0_to_base: TransformStamped
        self.start_pose: Pose | PoseStamped

        # Class vars
        self.num_poses_per_dof = 2
        self.x_range = 0.1
        self.y_range = 0.1
        self.z_range = 0.1
        self.roll_range = 1 * np.pi / 3
        self.pitch_range = 1 * np.pi / 3
        self.yaw_range = np.pi
        self.pose_list = []
        self.generator = np.random.default_rng(seed=secrets.randbits(128))
        return

    def transform_to_pose(self, tf_msg: TransformStamped, stamped: bool = True) -> Pose | PoseStamped:
        """Convert a transform message to a pose message"""
        if stamped:
            pose = Pose()
        else:
            pose = PoseStamped()
            pose.header.frame_id = ("amiga_base",)
            pose.header.stamp = self.get_clock().now().to_msg()

        # pose.position = tf_msg.transform.translation
        pose.position.x = tf_msg.transform.translation.x
        pose.position.y = tf_msg.transform.translation.y
        pose.position.z = tf_msg.transform.translation.z

        pose.orientation.x = tf_msg.transform.rotation.x
        pose.orientation.y = tf_msg.transform.rotation.y
        pose.orientation.z = tf_msg.transform.rotation.z
        pose.orientation.w = tf_msg.transform.rotation.w

        return pose

    # ===============================
    #        Action callbacks
    # ===============================

    def _action_goal_cb_generate_uniform_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT

    def _action_cancel_cb_generate_uniform_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        goal_handle.canceled()
        return CancelResponse.ACCEPT

    def _action_execute_cb_generate_uniform_poses(self, goal_handle: ServerGoalHandle):
        generate_poses_result = GeneratePoses.Result()

        self.tf_start__tool0_to_base = self.lookup_transform(
            source_frame=f"{self._param_robot_eef_part}__tool0",
            target_frame=f"{self._param_robot_base_part}__base",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=True,
        )

        # Start frame is zeroed as it will be transformed with all other poses at the end of this function
        self.start_pose = Pose()

        # RPY as demonstrated around mock_pruner__tool0 frame values... This means roll is different than "roll wrist". TODO: Standardize.

        position_poses = self.generate_uniform_position_poses(
            start_pose=self.start_pose,
            ranges=(self.x_range, self.y_range, self.z_range),
            num_poses=self.num_poses_per_dof,
        )

        # orientation_poses = self.generate_orientation_poses(
        #     start_pose=self.start_pose, ranges=(self.roll_range, self.pitch_range, self.yaw_range), num_poses=self.num_poses_per_dof
        # )

        all_poses = position_poses  # + orientation_poses

        all_poses.insert(0, self.start_pose)

        # Rotate the pose to the base frame for planning
        for i, pose in enumerate(all_poses):
            pose_xyz = [pose.position.x, pose.position.y, pose.position.z, 1]
            pose_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            pose_mat = Rotation.from_quat(pose_quat).as_matrix()

            world_pose_xyz = self.tf_start__tool0_to_base @ pose_xyz
            world_pose_orientation_mat = self.tf_start__tool0_to_base[:3, :3] @ pose_mat
            world_pose_quat = Rotation.from_matrix(world_pose_orientation_mat).as_quat()

            all_poses[i].position.x = world_pose_xyz[0]
            all_poses[i].position.y = world_pose_xyz[1]
            all_poses[i].position.z = world_pose_xyz[2]
            all_poses[i].orientation.x = world_pose_quat[0]
            all_poses[i].orientation.y = world_pose_quat[1]
            all_poses[i].orientation.z = world_pose_quat[2]
            all_poses[i].orientation.w = world_pose_quat[3]
            # generate_poses_result.poses[i] =

        # Save poses to action-result/message, publish message for later analysis
        generate_poses_result.poses = all_poses
        generated_poses_msg = GeneratedPoses()
        generated_poses_msg.poses = all_poses
        self._pub_generated_poses.publish(generated_poses_msg)

        goal_handle.succeed()
        generate_poses_result.success = True
        return generate_poses_result

    def _action_goal_cb_generate_uniform_cylindrical_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT

    def _action_cancel_cb_generate_uniform_cylindrical_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        goal_handle.canceled()
        return CancelResponse.ACCEPT

    def _action_execute_cb_generate_uniform_cylindrical_poses(self, goal_handle: ServerGoalHandle):
        self.start_pose = Pose()

        generate_poses_result = GenerateCylindricalPoses.Result()

        generate_poses_goal: GenerateCylindricalPoses.Goal = goal_handle.request

        self.tf_start__tool0_to_base = self.lookup_transform(
            source_frame=f"{self._param_robot_eef_part}__tool0",
            target_frame=f"{self._param_robot_base_part}__base",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=True,
        )

        position_poses = self.generate_uniform_cylindrical_position_poses(generate_poses_goal=generate_poses_goal)

        all_poses = position_poses

        all_poses.insert(0, self.start_pose)

        # Rotate the pose to the base frame for planning
        for i, pose in enumerate(all_poses):
            pose_xyz = [pose.position.x, pose.position.y, pose.position.z, 1]
            pose_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            pose_mat = Rotation.from_quat(pose_quat).as_matrix()

            world_pose_xyz = self.tf_start__tool0_to_base @ pose_xyz
            world_pose_orientation_mat = self.tf_start__tool0_to_base[:3, :3] @ pose_mat
            world_pose_quat = Rotation.from_matrix(world_pose_orientation_mat).as_quat()

            all_poses[i].position.x = world_pose_xyz[0]
            all_poses[i].position.y = world_pose_xyz[1]
            all_poses[i].position.z = world_pose_xyz[2]
            all_poses[i].orientation.x = world_pose_quat[0]
            all_poses[i].orientation.y = world_pose_quat[1]
            all_poses[i].orientation.z = world_pose_quat[2]
            all_poses[i].orientation.w = world_pose_quat[3]

        generate_poses_result.poses = all_poses
        generated_poses_msg = GeneratedPoses()
        generated_poses_msg.poses = all_poses
        self._pub_generated_poses.publish(generated_poses_msg)

        goal_handle.succeed()
        generate_poses_result.success = True
        return generate_poses_result

    def _action_goal_cb_generate_rpy_projected_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT

    def _action_cancel_cb_generate_rpy_projected_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        goal_handle.canceled()
        return CancelResponse.ACCEPT

    def _action_execute_cb_generate_rpy_projected_poses(self, goal_handle: ServerGoalHandle):
        self.start_pose = Pose()
        generate_poses_goal: GenerateRpyProjectedPoses.Goal = goal_handle.request
        generate_poses_result = GenerateRpyProjectedPoses.Result()

        self.tf_start__tool0_to_base = self.lookup_transform(
            source_frame=f"{self._param_robot_eef_part}__tool0",
            target_frame=f"{self._param_robot_base_part}__base",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=True,
        )

        poses = self.generate_rpy_projected_poses(generate_poses_goal=generate_poses_goal)

        for i, pose in enumerate(poses):
            pose_xyz = [pose.position.x, pose.position.y, pose.position.z, 1]
            pose_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            pose_mat = Rotation.from_quat(pose_quat).as_matrix()

            world_pose_xyz = self.tf_start__tool0_to_base @ pose_xyz
            world_pose_orientation_mat = self.tf_start__tool0_to_base[:3, :3] @ pose_mat
            world_pose_quat = Rotation.from_matrix(world_pose_orientation_mat).as_quat()

            poses[i].position.x = world_pose_xyz[0]
            poses[i].position.y = world_pose_xyz[1]
            poses[i].position.z = world_pose_xyz[2]
            poses[i].orientation.x = world_pose_quat[0]
            poses[i].orientation.y = world_pose_quat[1]
            poses[i].orientation.z = world_pose_quat[2]
            poses[i].orientation.w = world_pose_quat[3]

        generate_poses_result.poses = poses
        generated_poses_msg = GeneratedPoses()
        generated_poses_msg.poses = poses
        self._pub_generated_poses.publish(generated_poses_msg)

        goal_handle.succeed()
        generate_poses_result.success = True
        return generate_poses_result

    # ===============================
    #        Helper functions
    # ===============================
    def generate_uniform_cylindrical_position_poses(self, generate_poses_goal: GenerateCylindricalPoses.Goal):
        poses = []

        r = np.linspace(
            start=generate_poses_goal.radius_range[0],
            stop=generate_poses_goal.radius_range[1],
            num=generate_poses_goal.num_radius_poses,
        )
        theta = np.linspace(
            start=generate_poses_goal.theta_range[0],
            stop=generate_poses_goal.theta_range[1],
            num=generate_poses_goal.num_theta_poses,
        )
        x = np.outer(r, np.cos(theta)).flatten()
        y = np.outer(r, np.sin(theta)).flatten()

        z = np.linspace(
            start=generate_poses_goal.z_range[0],
            stop=generate_poses_goal.z_range[1],
            num=generate_poses_goal.num_z_poses,
        )

        xy = np.stack((x, y), axis=1)
        xy_repeated = np.tile(xy, reps=(len(z), 1))
        z_repeated = np.repeat(z, len(x))[:, np.newaxis]

        xyz = np.hstack((xy_repeated, z_repeated))

        for p in xyz:
            poses.append(Pose(position=Point(x=p[0], y=p[1], z=p[2])))

        return poses

    def generate_uniform_position_poses(
        self, start_pose: Pose | PoseStamped, ranges: tuple[float], num_poses: int
    ) -> list[Pose]:
        poses = []

        start_position = [start_pose.position.x, start_pose.position.x, start_pose.position.z]

        _start_mgrid = np.mgrid[
            (start_position[0] - ranges[0] / 2) : (start_position[0] + ranges[0] / 2) : (num_poses * 1j),
            (start_position[1] - ranges[1] / 2) : (start_position[1] + ranges[1] / 2) : (num_poses * 1j),
            (start_position[2] - ranges[2] / 2) : (start_position[2] + ranges[2] / 2) : (num_poses * 1j),
        ]

        position_grid = _start_mgrid.reshape(3, -1).T

        for p in position_grid:
            poses.append(Pose(position=Point(x=p[0], y=p[1], z=p[2])))

        return poses

    def generate_rpy_projected_poses(self, generate_poses_goal: GenerateRpyProjectedPoses.Goal):
        poses = []
        roll_range = generate_poses_goal.roll_range
        pitch_range = generate_poses_goal.pitch_range
        yaw_range = generate_poses_goal.yaw_range
        z_range = generate_poses_goal.z_range

        rolls = np.linspace(*roll_range, generate_poses_goal.num_roll_poses)
        pitches = np.linspace(*pitch_range, generate_poses_goal.num_pitch_poses)
        yaws = np.linspace(*yaw_range, generate_poses_goal.num_yaw_poses)
        zs = np.linspace(*zs, generate_poses_goal.num_z_poses)

        rpy_grid = list(itertools.product(rolls, pitches, yaws))
        rotations = Rotation.from_euler("zyx", [(y, p, r) for r, p, y in rpy_grid])
        rot_quats = rotations.as_quat()

        start_points = np.zeros(shape=(len(zs), 3))
        start_points[:, 2] = zs

        rotated_points = []
        for point in start_points:
            rotated_points.extend(rotations.apply(point))

        rot_quats = rot_quats.repeat(zs.shape, axis=0)

        for i in range(rotated_points.shape[0]):
            poses.append(
                Pose(
                    position=Point(x=rotated_points[i][0], y=rotated_points[i][2], z=rotated_points[i][3]),
                    orientation=Quaternion(x=rot_quats[i][0], y=rot_quats[i][1], z=rot_quats[i][2], w=rot_quats[i][3]),
                )
            )

        return rotated_points


"""
    def generate_uniform_orientation_poses(self, start_pose: Pose | PoseStamped, ranges: tuple[float], num_poses: int):
        poses = []

        start_position = [start_pose.position.x, start_pose.position.x, start_pose.position.z]
        start_orientation = [
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w,
        ]

        rot = Rotation.from_quat(
            [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
        )

        roll, pitch, yaw = rot.as_euler("xyz", degrees=False)

        roll_linspace = np.linspace(start=roll - self.roll_range / 2, stop=roll + self.roll_range / 2, num=num_poses)
        pitch_linspace = np.linspace(
            start=pitch - self.pitch_range / 2, stop=pitch + self.pitch_range / 2, num=num_poses
        )
        yaw_linspace = np.linspace(start=yaw - self.yaw_range / 2, stop=yaw + self.yaw_range / 2, num=num_poses)

        for r in roll_linspace:
            pose = deepcopy(start_pose)
            rot = Rotation.from_euler("xyz", [r, pitch, yaw])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poses.append(pose)

        for p in pitch_linspace:
            pose = deepcopy(start_pose)
            rot = Rotation.from_euler("xyz", [roll, p, yaw])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poses.append(pose)

        for y in yaw_linspace:
            pose = deepcopy(start_pose)
            rot = Rotation.from_euler("xyz", [roll, pitch, y])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poses.append(pose)

        return poses
"""


def main():
    rclpy.init()
    generate_poses_service_node = GeneratePosesServiceNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(generate_poses_service_node, executor=executor)
    generate_poses_service_node.destroy_node()
    rclpy.shutdown()

    return
