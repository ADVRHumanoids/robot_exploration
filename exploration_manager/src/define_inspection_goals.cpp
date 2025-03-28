#include <exploration_manager/define_inspection_goals.h>

DefineInspectionGoals::DefineInspectionGoals(const std::string& name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   
    //TODO: From params
    min_distance_to_object_ = 0.8;
    max_distance_to_object_ = 1.5;
}

BT::NodeStatus DefineInspectionGoals::tick(){
    // RCLCPP_INFO(node_->get_logger(), "DefineInspectionGoals");
            
    if(bt_data_->tasks[bt_data_->current_task].getNavTargetsNumber() < INSPECTION_IMAGES){
        bt_data_->tasks[bt_data_->current_task].clearNavTargets();

        for(int i = 0; i < INSPECTION_IMAGES; i++){
            geometry_msgs::msg::Pose new_pose;
            
            // Select nav target in the line, at X distance from object
            angle_ = atan2(bt_data_->object_pose.transform.translation.y - bt_data_->last_robot_pose.transform.translation.y,
                           bt_data_->object_pose.transform.translation.x - bt_data_->last_robot_pose.transform.translation.x);
            
            // Transform into [-3.14; 3.14]
            if(angle_ > 3.14)
                angle_ = angle_ - 6.28*(1.0 + std::floor(angle_/6.28));
            else if(angle_ < -3.14)
                angle_ = angle_ + 6.28*(1.0 + std::floor(-angle_/6.28));
            


            new_pose.position.x = bt_data_->object_pose.transform.translation.x - (1.2-i*0.35)*cos(angle_);
            new_pose.position.y = bt_data_->object_pose.transform.translation.y - (1.2-i*0.35)*sin(angle_);

            //Define orientation to face the object
            new_pose.orientation.x = 0;
            new_pose.orientation.y = 0;
            new_pose.orientation.z = sin(angle_/2.0);
            new_pose.orientation.w = cos(angle_/2.0);

            bt_data_->tasks[bt_data_->current_task].addNavTarget(new_pose);        
        }
    }
    
    return BT::NodeStatus::SUCCESS;
}