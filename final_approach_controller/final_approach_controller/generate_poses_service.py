#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

from final_approach_controller.tf_node import TFNode

from branch_detection_system_moveit_msgs.srv import MoveToPose
from controller_manager_msgs.srv import SwitchController
from final_approach_controller_msgs.action import GeneratePoses, GenerateCylindricalPoses, GenerateRpyProjectedPoses, GetCurrentPose
from final_approach_controller_msgs.msg import GeneratedPoses
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from std_srvs.srv import Trigger

from copy import deepcopy
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
import secrets

import plotly.graph_objects as go
from branch_detection_system_analysis.plot import plotly_helpers as ph

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
        _param_use_mock_hardware: bool = (
            self.declare_parameter(name="use_mock_hardware", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        )
        if _param_use_mock_hardware:
            self._move_group_controller = "joint_trajectory_controller"
        else:
            self._move_group_controller = "scaled_joint_trajectory_controller"
        self._servo_controller = "forward_position_controller"
        # self.warn(self._param_robot_eef_part.get_parameter_value().string_value)

        # Callback groups
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Service clients
        self._srv_cartesian_move_to_pose = self.create_client(
            srv_type=MoveToPose, srv_name="/cartesian_move_to_pose", callback_group=self._reentrant_cb_group
        )
        self._srv_switch_ctrls = self.create_client(
            srv_type=SwitchController,
            srv_name="/controller_manager/switch_controller",
            callback_group=self._reentrant_cb_group,
        )

        while not self._srv_cartesian_move_to_pose.wait_for_service(timeout_sec=1.0):
            self.warn("Waiting for Cartesian move to pose service...")

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

        self._action_server_get_current_pose = ActionServer(
            node=self,
            action_name="/get_current_pose",
            action_type=GetCurrentPose,
            goal_callback=self._action_goal_cb_get_current_pose,
            cancel_callback=self._action_cancel_cb_get_current_pose,
            execute_callback=self._action_execute_cb_get_current_pose,
            callback_group=self._reentrant_cb_group
        )

        # Publishers
        self._pub_generated_poses = self.create_publisher(
            msg_type=GeneratedPoses, topic="generated_start_poses", qos_profile=5
        )
        self._pub_rpy_target_pose = self.create_publisher(
            msg_type=Pose, topic="rpy_target_pose", qos_profile=5
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
            pose.header.frame_id = f"{self._param_robot_base_part}_base"
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

    async def _action_execute_cb_generate_uniform_cylindrical_poses(self, goal_handle: ServerGoalHandle):

        # ##############################################################################################
        # # TODO: Make this its own behavior
        # await self.switch_controllers(
        #     activate_controllers=self._move_group_controller,
        #     deactivate_controllers=self._servo_controller,
        # )
        # start_pose = Pose(
        #     position=Point(x=-0.336062743489267, y=1.070126103681655, z=1.7842673493199608),
        #     orientation=Quaternion(
        #         x=0.7446764785111847, y=-0.03594632354223663, z=0.09850742246533545, w=-0.6591366261217881
        #     ),
        # )
        # _move_to_pose_req = MoveToPose.Request()
        # _move_to_pose_req.goal.position = start_pose.position
        # _move_to_pose_req.goal.orientation = start_pose.orientation

        # self.info("Sending goal")

        # move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(request=_move_to_pose_req)
        # move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
        # await move_group_future

        # await self.switch_controllers(
        #     activate_controllers=self._servo_controller,
        #     deactivate_controllers=self._move_group_controller,
        # )
        # ################################################################################################

        start_pose = Pose()

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

        all_poses.insert(0, start_pose)

        # all_poses.insert(0, self.start_pose)

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

    async def _action_execute_cb_generate_rpy_projected_poses(
        self, goal_handle: ServerGoalHandle
    ):
        self.start_pose = Pose()
        generate_poses_goal: GenerateRpyProjectedPoses.Goal = goal_handle.request
        _generate_poses_result = GenerateRpyProjectedPoses.Result()

        ##############################################################################################
        # # TODO: Make this its own behavior
        # await self.switch_controllers(
        #     activate_controllers=self._move_group_controller,
        #     deactivate_controllers=self._servo_controller,
        # )
        # start_pose = Pose(
        #     position=Point(x=-0.336062743489267, y=1.070126103681655, z=1.7842673493199608),
        #     orientation=Quaternion(
        #         x=0.7446764785111847, y=-0.03594632354223663, z=0.09850742246533545, w=-0.6591366261217881
        #     ),
        # )
        # _move_to_pose_req = MoveToPose.Request()
        # _move_to_pose_req.goal.position = start_pose.position
        # _move_to_pose_req.goal.orientation = start_pose.orientation

        # self.info("Sending goal")

        # move_group_future: Future = self._srv_cartesian_move_to_pose.call_async(request=self._move_to_pose_req)
        # move_group_future.add_done_callback(callback=self._done_cb_srv_cartesian_move_to_pose)
        # await move_group_future

        # await self.switch_controllers(
        #     activate_controllers=self._servo_controller,
        #     deactivate_controllers=self._move_group_controller,
        # )
        ################################################################################################

        self.tf_start__tool0_to_base = self.lookup_transform(
            source_frame=f"{self._param_robot_eef_part}__tool0",
            target_frame=f"{self._param_robot_base_part}__base",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=True,
        )

        tf_start__tool0_to_base = self.lookup_transform(
            source_frame=f"{self._param_robot_eef_part}__tool0",
            target_frame=f"{self._param_robot_base_part}__base",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=False,
        )

        # target_pose = self.transform_to_pose(tf_msg=tf_start__tool0_to_base, stamped=False)

        poses = self.generate_rpy_projected_poses(generate_poses_goal=generate_poses_goal)

        # poses.insert(0, self.start_pose)

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

        _generate_poses_result.poses = poses
        _generated_poses_msg = GeneratedPoses()
        _generated_poses_msg.poses = poses
        self._pub_generated_poses.publish(_generated_poses_msg)

        # mats = [self.pose_to_tf_mat(pose) for pose in poses]

        # fig = go.Figure()
        # for i in range(len(mats)):
        #     fig = ph.plot_3d_coordinate_frame(fig=fig, position=mats[i][:3, 3], orientation=mats[i][:3, :3])

        # fig.update_layout(scene=dict(aspectmode='data'))
        # fig.show()

        goal_handle.succeed()
        _generate_poses_result.success = True
        return _generate_poses_result
    
    def _action_goal_cb_get_current_pose(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT
    
    def _action_cancel_cb_get_current_pose(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        goal_handle.canceled()
        return CancelResponse.ACCEPT
    
    def _action_execute_cb_get_current_pose(self, goal_handle: ServerGoalHandle):
        _get_current_pose_result = GetCurrentPose.Result()

        tf_mat = self.lookup_transform(
            target_frame=f"{self._param_robot_base_part}__base",
            source_frame=f"{self._param_robot_eef_part}__tool0",
            time=self.get_clock().now(),
            sync=True,
            as_matrix=True
        )

        _get_current_pose_result.header.stamp = self.get_clock().now().to_msg()
        _get_current_pose_result.header.frame_id = f"{self._param_robot_base_part}__base"
        _get_current_pose_result.pose = self.tf_mat_to_pose(tf_mat=tf_mat)
        _get_current_pose_result.success = True
        goal_handle.succeed()

        return _get_current_pose_result
    
    def pose_to_tf_mat(self, pose: Pose) -> np.ndarray:
        mat = np.identity(4)
        p = pose.position
        q = pose.orientation
        mat[:3, 3] = [p.x, p.y, p.z]
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return mat
    
    def tf_mat_to_pose(self, tf_mat: np.ndarray) -> Pose:
        p = Pose()
        p.position.x = tf_mat[0, 3]
        p.position.y = tf_mat[1, 3]
        p.position.z = tf_mat[2, 3]
        q = Rotation.from_matrix(tf_mat[:3, :3]).as_quat()
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        return p


    # ===============================
    #        Future callbacks
    # ===============================
    def _done_cb_srv_cartesian_move_to_pose(self, future: Future) -> None:
        self.info("Move plan/execute finished.")
        return

    # ===============================
    #        Helper functions
    # ===============================
    def generate_uniform_cylindrical_position_poses(
        self, generate_poses_goal: GenerateCylindricalPoses.Goal
    ) -> list[Pose]:
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

    def generate_rpy_projected_poses(self, generate_poses_goal, target_point = np.array([0,0,0])) -> list[Pose]:
        # tp = target_pose.position
        # target_point = np.array([tp.x, tp.y, tp.z])

        poses = []

        # unpack ranges and counts
        roll_range, pitch_range, yaw_range, z_range = (
            generate_poses_goal.roll_range,
            generate_poses_goal.pitch_range,
            generate_poses_goal.yaw_range,
            generate_poses_goal.z_range,
        )
        n_r, n_p, n_y, n_z = (
            generate_poses_goal.num_roll_poses,
            generate_poses_goal.num_pitch_poses,
            generate_poses_goal.num_yaw_poses,
            generate_poses_goal.num_z_poses,
        )

        # sample each axis
        rolls = np.linspace(*roll_range, n_r)
        pitches = np.linspace(*pitch_range, n_p)
        yaws = np.linspace(*yaw_range, n_y)
        zs = np.linspace(*z_range, n_z)

        # for each RPY, rotate the local -Z vector, then translate from target
        for r, p, y in itertools.product(rolls, pitches, yaws):
            # build the rotation at the target
            R = Rotation.from_euler("zyx", [y, p, r])
            quat = R.as_quat()  # [x, y, z, w]

            for z in zs:
                if np.isclose(z, 0.0, atol=1e-9):
                    raise ValueError("z-offset must be non-zero")

                # local offset along -Z in the rotated frame
                offset = R.apply([0, 0, -z])

                # translate that offset from the target point
                pos = target_point - offset

                poses.append(
                    Pose(
                        position=Point(x=pos[0], y=pos[1], z=pos[2]),
                        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
                    )
                )

        return poses

    async def switch_controllers(self, activate_controllers, deactivate_controllers) -> None:
        try:
            switch_ctrlr_req = SwitchController.Request(
                activate_controllers=[activate_controllers],
                deactivate_controllers=[deactivate_controllers],
                strictness=SwitchController.Request.STRICT,
            )
            SwitchController.Response()
            switch_ctrlr_future: Future = self._srv_switch_ctrls.call_async(request=switch_ctrlr_req)
            await switch_ctrlr_future
            if switch_ctrlr_future.result().ok:
                self.info(f"Successfully deactivated {deactivate_controllers}, activated {activate_controllers}")
            else:
                self.error("Failed to switch controllers,")
        except Exception as e:
            self.error(f"{e}")
            pass
        return


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
