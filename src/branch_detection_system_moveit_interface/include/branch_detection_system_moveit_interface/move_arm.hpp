#include <rclcpp/rclcpp.hpp>
// Custom messages
#include <branch_detection_system_moveit_msgs/srv/move_to_pose.hpp>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class MoveArmNode : public rclcpp::Node {
    public:
        MoveArmNode();
        // move_group interface
        moveit::planning_interface::MoveGroupInterface move_group_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        

    private:
        rclcpp::Service<branch_detection_system_moveit_msgs::srv::MoveToPose>::SharedPtr move_to_pose_service_;
        rclcpp::Service<branch_detection_system_moveit_msgs::srv::MoveToPose>::SharedPtr cartesian_move_to_pose_service_;
        rclcpp::CallbackGroup::SharedPtr move_to_pose_cb_group_;
        rclcpp::CallbackGroup::SharedPtr cartesian_move_to_pose_cb_group_;

        void move_to_pose(
            const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Request> request,
            const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Response> response
        );

        void cartesian_move_to_pose(
            const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Request> request,
            const std::shared_ptr<branch_detection_system_moveit_msgs::srv::MoveToPose::Response> response
        );
};

