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

from final_approach_controller_msgs.action import GeneratePoses
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from std_srvs.srv import Trigger

from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation


class GeneratePosesServiceNode(TFNode):
    def __init__(self):
        super().__init__(node_name="generate_poses_action_server_node")
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.error = lambda x: self.get_logger().error(f"\n{x}")
        
        # Parameters
        self._param_robot_eef_part = self.declare_parameter('robot_eef_part', value=Parameter.Type.STRING)
        self.warn(self._param_robot_eef_part.get_parameter_value().string_value)

        # Callback groups
        self._reentrant_cb_group = ReentrantCallbackGroup()

        # Action servers
        self._action_srv_generate_poses_from_current_pose = ActionServer(
            node=self,
            action_name='/generate_poses',
            action_type=GeneratePoses,
            goal_callback=self._action_goal_cb_generate_poses,
            cancel_callback=self._action_cancel_cb_generate_poses,
            execute_callback=self._action_execute_cb_generate_poses,
            callback_group=self._reentrant_cb_group
        )

        # Class attrb
        self.tf_start__tool0_to_base: TransformStamped
        self.start_pose: Pose | PoseStamped

        # Class vars
        self.num_poses_per_dof = 5
        self.x_range = 0.1
        self.y_range = 0.1
        self.z_range = 0.1
        self.roll_range = 3 * np.pi / 4
        self.pitch_range = 3 * np.pi / 4
        self.yaw_range = np.pi
        self.pose_list = []
        return
    
    def transform_to_pose(self, tf_msg: TransformStamped, stamped: bool = True) -> Pose | PoseStamped:
        """Convert a transform message to a pose message"""
        if stamped:
            pose = Pose()
        else:
            pose = PoseStamped()
            pose.header.frame_id = 'amiga_base',
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
    
    def _action_goal_cb_generate_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received goal request")
        return GoalResponse.ACCEPT
    
    def _action_cancel_cb_generate_poses(self, goal_handle: ServerGoalHandle):
        self.info("Received cancel request")
        goal_handle.canceled()
        return CancelResponse.ACCEPT
    
    def _action_execute_cb_generate_poses(self, goal_handle: ServerGoalHandle):
        generate_poses_result = GeneratePoses.Result()

        self.tf_start__tool0_to_base = self.lookup_transform(
            source_frame='mock_pruner__tool0',
            target_frame="amiga__base",
            time=self.get_clock().now(),
            sync=True
        )

        self.start_pose = self.transform_to_pose(tf_msg=self.tf_start__tool0_to_base)

        x_poses = self.generate_position_poses('x', self.start_pose, self.x_range, num_poses=self.num_poses_per_dof)
        for pose in x_poses:
            generate_poses_result.poses.append(pose) 
        
        y_poses = self.generate_position_poses('y', self.start_pose, self.y_range, num_poses=self.num_poses_per_dof)
        for pose in y_poses:
            generate_poses_result.poses.append(pose) 

        z_poses = self.generate_position_poses('z', self.start_pose, self.z_range, num_poses=self.num_poses_per_dof)
        for pose in z_poses:
            generate_poses_result.poses.append(pose) 

        # RPY as demonstrated around mock_pruner__tool0 frame values... This means roll is different than "roll wrist". TODO: Standardize.

        orientation_poses = self.generate_orientation_poses(self.start_pose, self.roll_range, num_poses=self.num_poses_per_dof)
        for pose in orientation_poses:
            generate_poses_result.poses.append(pose)

        goal_handle.succeed()
        generate_poses_result.success = True
        return generate_poses_result
    
    def generate_position_poses(self, direction: str, start_pose: Pose | PoseStamped, _range: float, num_poses: int):
        poses = []

        if direction == 'x':
            pos = start_pose.position.x
        elif direction == 'y':
            pos = start_pose.position.y
        elif direction == 'z':
            pos = start_pose.position.z
        else:
            raise ValueError
        
        linspace = np.linspace(start=pos - _range/2, stop=pos + _range/2, num=num_poses)
        for _x in linspace:
            pose = deepcopy(start_pose)
            pose.position.x = _x
            poses.append(pose)
        return poses
    
    def generate_orientation_poses(self, start_pose: Pose | PoseStamped, _range: float, num_poses: int):
        poses = []

        rot = Rotation.from_quat([
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w
        ])

        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        roll_linspace = np.linspace(start=roll - self.roll_range/2, stop=roll + self.roll_range/2, num=num_poses)
        pitch_linspace = np.linspace(start=pitch - self.pitch_range/2, stop=pitch + self.pitch_range/2, num=num_poses)
        yaw_linspace = np.linspace(start=yaw - self.yaw_range/2, stop=yaw + self.yaw_range/2, num=num_poses)

        for r in roll_linspace:
            pose = deepcopy(start_pose)
            rot = Rotation.from_euler('xyz', [r, pitch, yaw])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poses.append(pose)

        for p in pitch_linspace:
            pose = deepcopy(start_pose)
            rot = Rotation.from_euler('xyz', [roll, p, yaw])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poses.append(pose)

        for y in yaw_linspace:
            pose = deepcopy(start_pose)
            rot = Rotation.from_euler('xyz', [roll, pitch, y])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poses.append(pose)

        return poses
    

def main():
    rclpy.init()
    generate_poses_service_node = GeneratePosesServiceNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(generate_poses_service_node, executor=executor)
    generate_poses_service_node.destroy_node()
    rclpy.shutdown()

    return

