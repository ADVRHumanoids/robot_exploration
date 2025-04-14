#include <exploration_manager/send_nav_pose.h>

SendNavPose::SendNavPose(const std::string& name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   
    //Service Client
    send_candidate_nav_target_ = node_->create_client<centauro_ros_nav_srvs::srv::SendCandidateNavTarget>("/set_candidate_nav_target");
    bt_data_->ros_status.send_cand_target_srv = send_candidate_nav_target_->wait_for_service(10s);

    candidate_nav_target_req_ = std::make_shared<centauro_ros_nav_srvs::srv::SendCandidateNavTarget::Request>();

    node_->declare_parameter("robot_exploration.distance_to_object_pose", 0.85);
    distance_to_object_pose_ = node_->get_parameter("robot_exploration.distance_to_object_pose").as_double();

    updated_nav_ = false;
}

BT::NodeStatus SendNavPose::tick(){

    RCLCPP_DEBUG(node_->get_logger(), "SendNavPose");
    candidate_nav_target_req_->reference_frame = bt_data_->world_frame;
    candidate_nav_target_req_->rotate_to_point = false;

    if(!bt_data_->ros_status.send_cand_target_srv){
        //Try again
        bt_data_->ros_status.send_cand_target_srv = send_candidate_nav_target_->wait_for_service(1s);
        RCLCPP_WARN(node_->get_logger(), "SendCandidateTarget Server Unavailable"); 
        return BT::NodeStatus::FAILURE;
    }

    //1. Update the intermediate target (based on the task)

    //If exploration phase or task is "reach obj"
    if(bt_data_->need_exploration || bt_data_->tasks[bt_data_->current_task].id == 1){

        //If Task is "Reach Target" and you know the object --> Nav Target is based on the Object
        if(bt_data_->known_object_pose){
            // Select nav target in the line, at X distance from object
            angle_ = atan2(bt_data_->object_pose.transform.translation.y - bt_data_->last_robot_pose.transform.translation.y,
                           bt_data_->object_pose.transform.translation.x - bt_data_->last_robot_pose.transform.translation.x);
            
            // Transform into [-3.14; 3.14]
            if(angle_ > 3.14)
                angle_ = angle_ - 6.28*(1.0 + std::floor(angle_/6.28));
            else if(angle_ < -3.14)
                angle_ = angle_ + 6.28*(1.0 + std::floor(-angle_/6.28));
            
            bt_data_->locomotion_target.position.x = bt_data_->object_pose.transform.translation.x - distance_to_object_pose_*cos(angle_);
            bt_data_->locomotion_target.position.y = bt_data_->object_pose.transform.translation.y - distance_to_object_pose_*sin(angle_);

            //TODO: Improve
            //Define orientation to face the object
            bt_data_->locomotion_target.orientation.x = 0;
            bt_data_->locomotion_target.orientation.y = 0;
            bt_data_->locomotion_target.orientation.z = sin(angle_/2.0);
            bt_data_->locomotion_target.orientation.w = cos(angle_/2.0);
        }
        //Else Nav Target if provided by "Explore" action
    }
    else if(bt_data_->tasks[bt_data_->current_task].id == 2){ //Inspection Task

        // If not already defined the targets --> Return failure
        if(bt_data_->tasks[bt_data_->current_task].getNavTargetsNumber() == 0)
            return BT::NodeStatus::FAILURE;
        
        //Update locomotion_target if new goal is acquired
        temp_nav_pose_ = bt_data_->tasks[bt_data_->current_task].getLastNavTarget();

        if(temp_nav_pose_ != candidate_nav_target_req_->target_pose){
            RCLCPP_INFO(node_->get_logger(), "New goal: %f %f --> %f %f", candidate_nav_target_req_->target_pose.position.x,
                                                                          candidate_nav_target_req_->target_pose.position.y,
                                                                          temp_nav_pose_.position.x,
                                                                          temp_nav_pose_.position.y);
            bt_data_->locomotion_target = temp_nav_pose_;
            updated_nav_ = true;
        }
        
        candidate_nav_target_req_->point_to_face.x = bt_data_->object_pose.transform.translation.x;
        candidate_nav_target_req_->point_to_face.y = bt_data_->object_pose.transform.translation.y;
        candidate_nav_target_req_->point_to_face.z = bt_data_->object_pose.transform.translation.z;

        //Compute distance to previous nav target
        distance_to_nav_target_ = pow(bt_data_->last_robot_pose.transform.translation.x - bt_data_->locomotion_target.position.x, 2) +
                                  pow(bt_data_->last_robot_pose.transform.translation.y - bt_data_->locomotion_target.position.y, 2);

        //yaw_error        
        rob_yaw_ = atan2(2.0*(bt_data_->last_robot_pose.transform.rotation.x*bt_data_->last_robot_pose.transform.rotation.y +
                              bt_data_->last_robot_pose.transform.rotation.w*bt_data_->last_robot_pose.transform.rotation.z),
                         1.0 - 2.0*(bt_data_->last_robot_pose.transform.rotation.y*bt_data_->last_robot_pose.transform.rotation.y +
                                    bt_data_->last_robot_pose.transform.rotation.z*bt_data_->last_robot_pose.transform.rotation.z));

        nav_yaw_ = atan2(2.0*(bt_data_->locomotion_target.orientation.x*bt_data_->locomotion_target.orientation.y + 
                              bt_data_->locomotion_target.orientation.w*bt_data_->locomotion_target.orientation.z),
                         1.0 - 2.0*(bt_data_->locomotion_target.orientation.y*bt_data_->locomotion_target.orientation.y + 
                                    bt_data_->locomotion_target.orientation.z*bt_data_->locomotion_target.orientation.z));

        angle_ = nav_yaw_ - rob_yaw_;
        
        angle_ = fabs(angle_);
        if(angle_ > 6.28)
            angle_ -= 6.28;

        if(angle_ > 3.14)
            angle_ = 6.28 - angle_;

        //If not driving and close to object, facing it --> acquire image
        if(!bt_data_->is_driving && distance_to_nav_target_ < 0.25*0.25 && angle_ < 0.20){
            RCLCPP_INFO(node_->get_logger(), "Inspection Target Reached");
            bt_data_->acquire_image = true;
            return BT::NodeStatus::FAILURE;
        }

        //else move to inspection target
        if(!bt_data_->is_driving)
            RCLCPP_INFO(node_->get_logger(), "Inspection Target (%d/%d) to better define (sqr distance: %f, AngR: %f, AngT: %f)", 
                                            bt_data_->tasks[bt_data_->current_task].inspection_steps, INSPECTION_IMAGES,
                                            distance_to_nav_target_, rob_yaw_, nav_yaw_);

        //If the previous target is almost the same as the new one, do not send again (< 10cm)
        if(//bt_data_->is_driving && //!bt_data_->force_frontier_update && 
            pow(candidate_nav_target_req_->target_pose.position.x - bt_data_->locomotion_target.position.x, 2) +
            pow(candidate_nav_target_req_->target_pose.position.y - bt_data_->locomotion_target.position.y, 2) < 0.01f){
                return BT::NodeStatus::FAILURE;
        }
    }

    // bt_data_->force_frontier_update = true; //Force frontiers updates evertime you change nav target
    
    //Set nav target and send to Nav2
    candidate_nav_target_req_->target_pose = bt_data_->locomotion_target;

    RCLCPP_INFO(node_->get_logger(), "Send robot to: %f %f", 
                candidate_nav_target_req_->target_pose.position.x,
                candidate_nav_target_req_->target_pose.position.y);
        
    //Set robot pose
    candidate_nav_target_req_->robot_pose.position.x = bt_data_->last_robot_pose.transform.translation.x;
    candidate_nav_target_req_->robot_pose.position.y = bt_data_->last_robot_pose.transform.translation.y;
    candidate_nav_target_req_->robot_pose.position.z = bt_data_->last_robot_pose.transform.translation.z;

    candidate_nav_target_req_->robot_pose.orientation.x = bt_data_->last_robot_pose.transform.rotation.x;
    candidate_nav_target_req_->robot_pose.orientation.y = bt_data_->last_robot_pose.transform.rotation.y;
    candidate_nav_target_req_->robot_pose.orientation.z = bt_data_->last_robot_pose.transform.rotation.z;
    candidate_nav_target_req_->robot_pose.orientation.w = bt_data_->last_robot_pose.transform.rotation.w;
    
    candidate_nav_target_fut_ = send_candidate_nav_target_->async_send_request(candidate_nav_target_req_).share();
    candidate_nav_target_res_ = candidate_nav_target_fut_.get(); // Blocking call

    if(candidate_nav_target_res_ != nullptr)
    {
        bt_data_->locomotion_target = candidate_nav_target_res_->new_target;
        // bt_data_->is_driving = true;
        updated_nav_ = false;

        RCLCPP_INFO(node_->get_logger(), "Update target to: %f %f", 
                    bt_data_->locomotion_target.position.x,
                    bt_data_->locomotion_target.position.y);
                    
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}