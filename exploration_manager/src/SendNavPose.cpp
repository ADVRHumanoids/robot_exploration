#include <exploration_manager/SendNavPose.h>

SendNavPose::SendNavPose(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    send_nav_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
}

BT::NodeStatus SendNavPose::tick(){

    nav_target_.header.frame_id = bt_data_->world_frame;

    //If the previous target is almost the same as the new one, do not send again (< 10cm)
    if(bt_data_->is_driving && !bt_data_->force_frontier_update && 
       pow(nav_target_.pose.position.x - bt_data_->locomotion_target.position.x, 2) +
       pow(nav_target_.pose.position.x - bt_data_->locomotion_target.position.x, 2) < 0.01f){
        return BT::NodeStatus::SUCCESS;
    }

    nav_target_.pose = bt_data_->locomotion_target;

    bt_data_->is_driving = true;
    // bt_data_->force_frontier_update = true; //Force frontiers updates evertime you change nav target

    ROS_WARN("Send robot to: %f %f", nav_target_.pose.position.x, nav_target_.pose.position.y);

    send_nav_target_.publish(nav_target_);

    return BT::NodeStatus::SUCCESS;
}

void SendNavPose::halt(){

}
