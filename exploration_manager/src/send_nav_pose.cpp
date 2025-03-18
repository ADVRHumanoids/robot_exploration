#include <exploration_manager/send_nav_pose.h>

SendNavPose::SendNavPose(const std::string& name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   
    //Service Client
    send_candidate_nav_target_ = node_->create_client<centauro_ros_nav_srvs::srv::SendCandidateNavTarget>("/set_candidate_nav_target");
}

BT::NodeStatus SendNavPose::tick(){

    candidate_nav_target_req_->target_pose.header.frame_id = bt_data_->world_frame;

    //If the previous target is almost the same as the new one, do not send again (< 10cm)
    if(bt_data_->is_driving && !bt_data_->force_frontier_update && 
       pow(candidate_nav_target_req_->target_pose.pose.position.x - bt_data_->locomotion_target.position.x, 2) +
       pow(candidate_nav_target_req_->target_pose.pose.position.x - bt_data_->locomotion_target.position.x, 2) < 0.01f){
        return BT::NodeStatus::SUCCESS;
    }

    candidate_nav_target_req_->target_pose.pose = bt_data_->locomotion_target;
    // bt_data_->force_frontier_update = true; //Force frontiers updates evertime you change nav target

     RCLCPP_INFO(node_->get_logger(), "Send robot to: %f %f", 
                 candidate_nav_target_req_->target_pose.pose.position.x,
                 candidate_nav_target_req_->target_pose.pose.position.y);
    
    //Set robot pose
    candidate_nav_target_req_->robot_pose.position.x = bt_data_->last_robot_pose.transform.translation.x;
    candidate_nav_target_req_->robot_pose.position.y = bt_data_->last_robot_pose.transform.translation.y;
    candidate_nav_target_req_->robot_pose.position.z = bt_data_->last_robot_pose.transform.translation.z;

    candidate_nav_target_req_->robot_pose.orientation.x = bt_data_->last_robot_pose.transform.rotation.x;
    candidate_nav_target_req_->robot_pose.orientation.y = bt_data_->last_robot_pose.transform.rotation.y;
    candidate_nav_target_req_->robot_pose.orientation.z = bt_data_->last_robot_pose.transform.rotation.z;
    candidate_nav_target_req_->robot_pose.orientation.w = bt_data_->last_robot_pose.transform.rotation.w;

    candidate_nav_target_res_ = send_candidate_nav_target_->async_send_request(candidate_nav_target_req_);
    if (rclcpp::spin_until_future_complete(node_, candidate_nav_target_res_) == rclcpp::FutureReturnCode::SUCCESS)
    {
        bt_data_->is_driving = true;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}