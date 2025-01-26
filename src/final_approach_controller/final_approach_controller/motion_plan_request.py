_motion_plan_request = MotionPlanRequest()
_goal_pose_constraint = Constraints()
_position_constraint = PositionConstraint()
_orientation_constraint = OrientationConstraint()

_position_constraint.header.frame_id = "cart__base"
_position_constraint.link_name = "mock_pruner__tool0"
_position_constraint.weight = self.eef_weight
_position_constraint.target_point_offset.x = 0.0
_position_constraint.target_point_offset.y = 0.0
_position_constraint.target_point_offset.z = 0.0
_target_pose = Pose(
    position=Point(x=desired_eef_xyz[0], y=desired_eef_xyz[1], z=desired_eef_xyz[2]),
    orientation=Quaternion(
        x=desired_orientation_quat[0],
        y=desired_orientation_quat[1],
        z=desired_orientation_quat[2],
        w=desired_orientation_quat[3],
    ),
)
_position_constraint.constraint_region.primitive_poses.append(_target_pose)
_goal_pose_constraint.position_constraints.append(_position_constraint)

_orientation_constraint.header.frame_id = "cart__base"
_orientation_constraint.link_name = "mock_pruner__tool0"
_orientation_constraint.weight = self.eef_weight
_orientation_constraint.absolute_x_axis_tolerance = 0.01
_orientation_constraint.absolute_y_axis_tolerance = 0.01
_orientation_constraint.absolute_z_axis_tolerance = 0.01
# _orientation_constraint.orientation.x = desired_orientation_quat[0]
# _orientation_constraint.orientation.y = desired_orientation_quat[1]
# _orientation_constraint.orientation.z = desired_orientation_quat[2]
# _orientation_constraint.orientation.w = desired_orientation_quat[3]
_goal_pose_constraint.orientation_constraints.append(_orientation_constraint)

_motion_plan_request.workspace_parameters.header.frame_id = "cart__base"
_motion_plan_request.goal_constraints.append(_goal_pose_constraint)
_motion_plan_request.allowed_planning_time = 5.0
_motion_plan_request.num_planning_attempts = 10

self.arm_prefix = "ur5e__"  # TODO: Get prefix params from launch
self.robot_name = "pruning_robot"  # TODO: fix SRDF name structure as well
_motion_plan_request.group_name = f"{self.arm_prefix}{self.robot_name}_manipulator"
_motion_plan_request.start_state.is_diff = True


_motion_plan_request.planner_id = "RRTstarkConfigDefault"
# _motion_plan_request.pipeline_id = "ompl"

_move_group_goal = MoveGroup.Goal()
_move_group_goal.request = _motion_plan_request
_move_group_goal.planning_options = PlanningOptions(plan_only=False)

future = self._action_client_move_group.send_goal_async(goal=_move_group_goal)
