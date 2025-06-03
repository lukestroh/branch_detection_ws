#include <behavior_trees_cpp/final_approach_controller_client.hpp>
#include <final_approach_controller_msgs/action/run_final_approach.hpp>
#include <behaviortree_ros2/plugins.hpp>

FinalApproachControllerClient::FinalApproachControllerClient(const std::string& name,
                                                                               const BT::NodeConfig& conf,
                                                                               const BT::RosNodeParams& params)
    : RosActionNode<final_approach_controller_msgs::action::RunFinalApproach>(name, conf, params) {}

bool FinalApproachControllerClient::setGoal(Goal& goal) { return true; }


BT::NodeStatus FinalApproachControllerClient::onResultReceived(const WrappedResult& wr) {
    RCLCPP_INFO(
        logger(),
        "%s: onResultReceived. Done = %s",
        name().c_str(),
        wr.result->success ? "true" : "false"
    );
    return wr.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus FinalApproachControllerClient::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FinalApproachControllerClient::onFailure(BT::ActionNodeErrorCode error) {
    RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
    return BT::NodeStatus::FAILURE;
}

void FinalApproachControllerClient::onHalt() { RCLCPP_INFO(logger(), "%s: onHalt", name().c_str()); }

CreateRosNodePlugin(FinalApproachControllerClient, "FinalApproachControllerClient");