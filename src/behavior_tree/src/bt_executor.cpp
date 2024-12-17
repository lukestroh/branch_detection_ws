#include <behavior_tree/bt_executor.hpp>

#include <rclcpp/rclcpp.hpp>
// BranchDetectionActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options) {

// }

BranchDetectionActionServer::BranchDetectionActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options) {
    
}

void BranchDetectionActionServer::onTreeCreated(BT::Tree& tree) {
    bt_logger_ = std::make_shared<BT::StdCoutLogger>(tree);
}

std::optional<std::string> BranchDetectionActionServer::onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) {

}




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto branch_detection_action_server = std::make_shared<BT::TreeExecutionServer>(options);

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));

    exec.add_node(branch_detection_action_server->node());
    exec.spin();
    exec.remove_node(branch_detection_action_server->node());
    
    rclcpp::shutdown();
    return 0;
}