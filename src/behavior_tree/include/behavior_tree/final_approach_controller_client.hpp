#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#ifndef USE_SLEEP_PLUGIN
#include <behavior_tree/final_approach_controller_action.hpp>
#endif


class FinalApproachControllerClient : public BT::CoroActionNode {
    public:
        FinalApproachControllerClient(const std::string& name, const BT::NodeConfig& config);

        BT::NodeStatus tick() override;
        void halt() override;
        static BT::PortsList providedPorts();

};

