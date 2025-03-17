#include <exploration_manager/SendNavPose.h>

SendNavPose::SendNavPose(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    // send_nav_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    send_candidate_nav_target_ = nh_.serviceClient<centauro_ros_nav::SendCandidateNavTarget>("/set_candidate_nav_target");
}

BT::NodeStatus SendNavPose::tick(){

    candidate_nav_target_.request.target_pose.header.frame_id = bt_data_->world_frame;

    //If the previous target is almost the same as the new one, do not send again (< 10cm)
    if(bt_data_->is_driving && !bt_data_->force_frontier_update && 
       pow(candidate_nav_target_.request.target_pose.pose.position.x - bt_data_->locomotion_target.position.x, 2) +
       pow(candidate_nav_target_.request.target_pose.pose.position.x - bt_data_->locomotion_target.position.x, 2) < 0.01f){
        return BT::NodeStatus::SUCCESS;
    }

    candidate_nav_target_.request.target_pose.pose = bt_data_->locomotion_target;
    // bt_data_->force_frontier_update = true; //Force frontiers updates evertime you change nav target

    ROS_WARN("Send robot to: %f %f", candidate_nav_target_.request.target_pose.pose.position.x,
                                     candidate_nav_target_.request.target_pose.pose.position.y);

    // send_nav_target_.publish(nav_target_);
    
    //Set robot pose
    candidate_nav_target_.request.robot_pose.position.x = bt_data_->last_robot_pose.getOrigin().getX();
    candidate_nav_target_.request.robot_pose.position.y = bt_data_->last_robot_pose.getOrigin().getY();
    candidate_nav_target_.request.robot_pose.position.z = bt_data_->last_robot_pose.getOrigin().getZ();

    candidate_nav_target_.request.robot_pose.orientation.x = bt_data_->last_robot_pose.getRotation().getX();
    candidate_nav_target_.request.robot_pose.orientation.y = bt_data_->last_robot_pose.getRotation().getY();
    candidate_nav_target_.request.robot_pose.orientation.z = bt_data_->last_robot_pose.getRotation().getZ();
    candidate_nav_target_.request.robot_pose.orientation.w = bt_data_->last_robot_pose.getRotation().getW();

    if(send_candidate_nav_target_.call(candidate_nav_target_)){
        bt_data_->is_driving = true;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

void SendNavPose::halt(){

}
