#include <branch_detection_system_moveit_interface/move_arm.hpp>
#include <rclcpp/rclcpp.hpp>

MoveArmNode::MoveArmNode() : Node(
        "move_arm_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    ),
    move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur5e__pruning_robot_manipulator")
    
{
    // callback groups
    this->move_to_pose_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->cartesian_move_to_pose_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Services
    this->move_to_pose_service_ = this->create_service<branch_detection_system_moveit_msgs::srv::MoveToPose>(
        "move_to_pose",
        std::bind(&MoveArmNode::move_to_pose, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        this->move_to_pose_cb_group_
    );
    this->cartesian_move_to_pose_service_ = this->create_service<branch_detection_system_moveit_msgs::srv::MoveToPose>(
        "cartesian_move_to_pose",
        std::bind(&MoveArmNode::cartesian_move_to_pose, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        this->cartesian_move_to_pose_cb_group_
    );
}

void MoveArmNode::move_to_pose(
    const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Request> request,
    const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Response> response
) {
    geometry_msgs::msg::PoseStamped _pose_msg;
    
    _pose_msg.pose.position.x = request->goal.position.x;
    _pose_msg.pose.position.y = request->goal.position.y;
    _pose_msg.pose.position.z = request->goal.position.z;
    _pose_msg.pose.orientation.x = request->goal.orientation.x;
    _pose_msg.pose.orientation.y = request->goal.orientation.y;
    _pose_msg.pose.orientation.z = request->goal.orientation.z;
    _pose_msg.pose.orientation.w = request->goal.orientation.w;
    _pose_msg.header.frame_id = "cart__base"; // TODO: Get from launch parameters in the future
    _pose_msg.header.stamp = this->now();

    this->move_group_.setPoseTarget(_pose_msg, "mock_pruner__tool0");
    this->move_group_.setGoalOrientationTolerance(0.01);
    this->move_group_.setPlannerId("RRTconnectkConfigDefault");
    this->move_group_.setPlanningTime(20.0);
    this->move_group_.setNumPlanningAttempts(10);
    // this->move_group_.setGoalJointTolerance(0.001);

    // Attempt to move to pose goal
    moveit::planning_interface::MoveGroupInterface::Plan goal;
    auto const ok = static_cast<bool>(this->move_group_.plan(goal));
    response->result = true;
    if (ok) {
        this->move_group_.execute(goal);
    }
    else {
        RCLCPP_ERROR(
            this->get_logger(),
            "Planning failed!"
        );
        response->result = false;
    }
}


void MoveArmNode::cartesian_move_to_pose(
    const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Request> request,
    const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Response> response
) {
    // Waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(this->move_group_.getCurrentPose().pose);
    waypoints.push_back(request->goal);

    // Plan cartesian path
    double jump_threshold = 0.0; //disable jump threshold
    double eef_step = 0.01; //step size (m)
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction_planned = this->move_group_.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory
    );

    RCLCPP_INFO(this->get_logger(), "Cartesian path plan percentage: %.2f", fraction_planned);

    if (fraction_planned > 0.95) {
        moveit::planning_interface::MoveGroupInterface::Plan goal;
        goal.trajectory_ = trajectory;
        this->move_group_.execute(goal);
        response->result = true;
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Failed to plan a Cartesian path.");
        response->result = false;
    }

}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);


    auto move_arm_node = std::make_shared<MoveArmNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_arm_node);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}