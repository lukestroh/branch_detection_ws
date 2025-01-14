start_pose = Pose()
start_pose.position.x = tf_cut_point_to_base[3, 0]
start_pose.position.y = tf_cut_point_to_base[3, 1]
start_pose.position.z = tf_cut_point_to_base[3, 2]
start_pose.orientation.x = cut_point_world_orientation_quat[0]
start_pose.orientation.y = cut_point_world_orientation_quat[1]
start_pose.orientation.z = cut_point_world_orientation_quat[2]
start_pose.orientation.w = cut_point_world_orientation_quat[3]

end_pose = Pose()
end_pose.position.x = desired_eef_xyz[0]
end_pose.position.y = desired_eef_xyz[1]
end_pose.position.z = desired_eef_xyz[2]
end_pose.orientation.x = cut_point_world_orientation_quat[0]
end_pose.orientation.y = cut_point_world_orientation_quat[1]
end_pose.orientation.z = cut_point_world_orientation_quat[2]
end_pose.orientation.w = cut_point_world_orientation_quat[3]

cartesian_path_request = GetCartesianPath.Request()
self.arm_prefix = "ur5e__"  # TODO: Get prefix params from launch
self.robot_name = "pruning_robot"  # TODO: fix SRDF name structure as well
cartesian_path_request.group_name = f"{self.arm_prefix}{self.robot_name}_manipulator"
cartesian_path_request.start_state.is_diff = True
cartesian_path_request.header.frame_id = "cart__base"
cartesian_path_request.max_step = 0.01
cartesian_path_request.jump_threshold = 0.0
cartesian_path_request.avoid_collisions = True
cartesian_path_request.waypoints = [start_pose, end_pose]

future = self._srv_client_get_cartesian_path.call_async(request=cartesian_path_request)
future.add_done_callback(callback=self._srv_client_get_cartesian_path_done_cb)
