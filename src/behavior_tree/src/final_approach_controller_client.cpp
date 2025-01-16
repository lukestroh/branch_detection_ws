#include <behavior_tree/final_approach_controller_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#ifndef USE_SLEEP_PLUGIN
#include <behavior_tree/final_approach_controller_action.hpp>
#endif

FinalApproachControllerClient::FinalApproachControllerClient(const std::string& name, const BT::NodeConfig& config) : BT::CoroActionNode(name, config) {
    // action_client_ = rclcpp_action::create_client<final_approach_controller_msgs::action::RunFinalApproach>()
}

BT::NodeStatus FinalApproachControllerClient::tick() {
    std::string msg;
    if (getInput("message", msg)) {
        std::cout << "FinalApproachControllerClient: " << msg <<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else {
        std::cout << "FinalApproachControllerClient failed" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::PortsList FinalApproachControllerClient::providedPorts() {
    return {
        BT::InputPort<bool>("success"),
        BT::OutputPort<double>("tof0"),
        BT::OutputPort<double>("tof1"),
        BT::OutputPort<double>("dist"),
        BT::OutputPort<double>("theta")
    };
}

void FinalApproachControllerClient::halt() {
    BT::CoroActionNode::halt();
}


static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree>
        <Sequence>
            <FinalApproachControllerClient message="start"/>
            <FinalApproachControllerAction name="fapc_action" />
            <FinalApproachControllerClient message="FAPC action completed" />
        </Sequence>
    </BehaviorTree>
</root>
)";


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto final_approach_controller_client_node = std::make_shared<rclcpp::Node>("final_approach_controller_client_node");

    BT::BehaviorTreeFactory bt_factory;

    bt_factory.registerNodeType<FinalApproachControllerClient>("FinalApproachControllerClient");

    BT::RosNodeParams params;
    params.nh = final_approach_controller_client_node;
    params.default_port_value = "final_approach_controller_service";

#ifdef USE_SLEEP_PLUGIN
    RegisterRosNode(bt_factory, "/home/luke/branch_detection_ws/install/behavior_tree/lib/behavior_tree/libfinal_approach_controller_plugin.so", params);
#else  
    bt_factory.registerNodeType<RunFinalApproachControllerAction>("RunFinalApproachControllerAction", params);
#endif

    auto tree = bt_factory.createTreeFromText(xml_text);
    // auto tree = bt_factory.createTreeFromFile()

    tree.tickWhileRunning();

    return 0;
}