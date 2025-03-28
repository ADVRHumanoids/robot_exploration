#include <exploration_manager/define_inspection_goals.h>

DefineInspectionGoals::DefineInspectionGoals(const std::string& name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   

}

BT::NodeStatus DefineInspectionGoals::tick(){
    RCLCPP_INFO(node_->get_logger(), "DefineInspectionGoals");
   
    return BT::NodeStatus::SUCCESS;
}