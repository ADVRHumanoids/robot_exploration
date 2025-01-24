#include <exploration_manager/SendNavPose.h>

SendNavPose::SendNavPose(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    send_nav_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
}

BT::NodeStatus SendNavPose::tick(){

    nav_target_.header.frame_id = "map";

    //If the previous target is almost the same as the new one, do not send again (< 10cm)
    if(bt_data.is_driving && 
       pow(nav_target_.pose.position.x - bt_data.locomotion_target.position.x, 2) +
       pow(nav_target_.pose.position.x - bt_data.locomotion_target.position.x, 2) < 0.01f){
        return BT::NodeStatus::SUCCESS;
    }

    nav_target_.pose = bt_data.locomotion_target;

    bt_data.is_driving = true;

    ROS_WARN("Send robot to: %f %f", nav_target_.pose.position.x, nav_target_.pose.position.y);

    send_nav_target_.publish(nav_target_);

    return BT::NodeStatus::SUCCESS;
}

void SendNavPose::halt(){

}
