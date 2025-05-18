#ifndef __BT_EXECUTOR_HPP__
#define __BT_EXECUTOR_HPP__

#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>



class BranchDetectionActionServer : public BT::TreeExecutionServer{
    public:
        BranchDetectionActionServer(const rclcpp::NodeOptions& options);
        ~BranchDetectionActionServer() override;

    protected:
        bool onGoalReceived(const std::string& tree_name, const std::string& payload) override;

        void onTreeCreated(BT::Tree& tree) override;

        void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override;

        std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

        std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_canceled) override;

    private:
        std::shared_ptr<BT::StdCoutLogger> bt_logger_;

};


#endif // __BT_EXECUTOR_HPP