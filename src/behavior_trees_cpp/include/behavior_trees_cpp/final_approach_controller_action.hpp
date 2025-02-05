#ifndef __BT_RUN_FINAL_APPROACH_CONTROLLER__
#define __BT_RUN_FINAL_APPROACH_CONTROLLER__

#include <behaviortree_ros2/bt_action_node.hpp>
#include <final_approach_controller_msgs/action/run_final_approach.hpp>



class RunFinalApproachControllerAction : public BT::RosActionNode<final_approach_controller_msgs::action::RunFinalApproach> {
    public:
        RunFinalApproachControllerAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

        static BT::PortsList providedPorts() {
            return providedBasicPorts({ BT::InputPort<unsigned>("") });
        }

        bool setGoal(Goal& goal) override;
        void onHalt() override;
        BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

        virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;


};



#endif // __BT_RUN_FINAL_APPROACH_CONTROLLER__