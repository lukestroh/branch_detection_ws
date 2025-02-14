#include <behavior_trees_cpp/final_approach_controller_action.hpp>
#include <behaviortree_ros2/plugins.hpp>


RunFinalApproachControllerAction::RunFinalApproachControllerAction(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params) : RosActionNode<final_approach_controller_msgs::action::RunFinalApproach>(
                                                                                                    name, conf, params)
{

}

bool RunFinalApproachControllerAction::setGoal(Goal& goal) {
    return true;
}

BT::NodeStatus RunFinalApproachControllerAction::onResultReceived(const WrappedResult& wr) {
    RCLCPP_INFO(
        logger(),
        "%s: onResultReceived. Done = %s", name().c_str(),
        wr.result->success ? "true" : "false"
    );
    return wr.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RunFinalApproachControllerAction::onFailure(BT::ActionNodeErrorCode error) {
    RCLCPP_ERROR(
        logger(),
        "%s: onFailure with error: %s",
        name().c_str(),
        toStr(error)
    );
    return BT::NodeStatus::FAILURE;
}

void RunFinalApproachControllerAction::onHalt() {
    RCLCPP_INFO(
        logger(),
        "%s: onHalt",
        name().c_str()
    );
}

CreateRosNodePlugin(RunFinalApproachControllerAction, "RunFinalApproachControllerAction");