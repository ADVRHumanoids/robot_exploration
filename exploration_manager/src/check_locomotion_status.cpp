#include <exploration_manager/check_locomotion_status.h>

CheckLocomotionStatus::CheckLocomotionStatus(const std::string& name,
                                             const BT::NodeConfig &config,
                                             rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   
    // Distance from nav target
    node_->declare_parameter("robot_exploration.min_nav_target_distance", 0.85);
    min_nav_target_distance_ = node_->get_parameter("robot_exploration.min_nav_target_distance").as_double();
    min_nav_target_distance_ *= min_nav_target_distance_; // Consider squared

    // Distance to target object (when known pose)
    node_->declare_parameter("robot_exploration.distance_target_object", 0.85);
    distance_target_object_ = node_->get_parameter("robot_exploration.distance_target_object").as_double();
    distance_target_object_ *= distance_target_object_; // Consider squared

    //Distance to frontier
    node_->declare_parameter("robot_exploration.min_dist_frontier_robot", 0.85);
    min_frontier_distance_ = node_->get_parameter("robot_exploration.min_dist_frontier_robot").as_double();
    min_frontier_distance_ *= min_frontier_distance_; // Consider squared

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&CheckLocomotionStatus::getNavStatus, this, _1));

    prev_status_msg_.status = GoalStatus::STATUS_UNKNOWN;
}

void CheckLocomotionStatus::getNavStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg){
    status_msg_ = msg;
}

//Check if Ros Nav succeed and also that the robot is "close enough" to desired location
BT::NodeStatus CheckLocomotionStatus::tick(){
    // RCLCPP_INFO(node_->get_logger(), "CheckLocomotionStatus");
        
    //Get robot's pose
     try {
        bt_data_->now = node_->get_clock()->now();
        
        bt_data_->last_robot_pose = tf_buffer_->lookupTransform(
                                        bt_data_->world_frame, bt_data_->base_frame,
                                        bt_data_->now);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "CheckLocomotionStatus: Could not transform!");
        return BT::NodeStatus::FAILURE;
    }

    //Squared distance
    temp_distance_ = (bt_data_->last_robot_pose.transform.translation.x - bt_data_->locomotion_target.position.x)*
                     (bt_data_->last_robot_pose.transform.translation.x - bt_data_->locomotion_target.position.x) +
                     (bt_data_->last_robot_pose.transform.translation.y - bt_data_->locomotion_target.position.y)*
                     (bt_data_->last_robot_pose.transform.translation.y - bt_data_->locomotion_target.position.y);

    //If no message --> presume not driving and start of execution
    if(status_msg_ == nullptr){
        bt_data_->is_driving = false;
        
        return BT::NodeStatus::FAILURE;
    }

    status_msg_id_ = status_msg_->status_list.size() - 1;

    // Consider the last in status_list
    if(status_msg_id_ >= 0){

        if(status_msg_->status_list[status_msg_id_].status == GoalStatus::STATUS_ACCEPTED){
            //NOTE: Accepted and waiting execution
        }
        // If Exploring and close to nav target (frontier)
        else if(!bt_data_->known_object_pose && status_msg_->status_list[status_msg_id_].status == GoalStatus::STATUS_EXECUTING &&
            (temp_distance_ < min_frontier_distance_))
        {
            bt_data_->is_driving = true;
            RCLCPP_INFO(node_->get_logger(), "Force Frontier Update");
            bt_data_->force_frontier_update = true;
        }
        else if(status_msg_->status_list[status_msg_id_].status == GoalStatus::STATUS_SUCCEEDED){         
            bt_data_->is_driving = false;
            // bt_data_->finished_exploration = true;
            
            //Check distance to target (to avoid also checking SUCCEED of prev exec.)
            if(bt_data_->known_object_pose && temp_distance_ < min_nav_target_distance_){

                RCLCPP_INFO(node_->get_logger(), "Nav Target Reached!");

                //If we arrived to the object --> Finish, else change frontier
                bt_data_->need_exploration = !(bt_data_->known_object_pose);
                bt_data_->finished_exploration = bt_data_->known_object_pose;

                if(!bt_data_->known_object_pose)
                    bt_data_->force_frontier_update = true;
            }
            // else{
            //     RCLCPP_INFO(node_->get_logger(), "Locomotion Ended...");                
            // }
        }
        else if(status_msg_->status_list[status_msg_id_].status == GoalStatus::STATUS_ABORTED){
            RCLCPP_INFO(node_->get_logger(), "Locomotion Aborted...");  
            bt_data_->is_driving = false;
            bt_data_->need_exploration = true;
            bt_data_->finished_exploration = false;
        }
        else if(status_msg_->status_list[status_msg_id_].status == GoalStatus::STATUS_EXECUTING){
            bt_data_->is_driving = true;
        }
        else{
            RCLCPP_INFO(node_->get_logger(), "Not previous cases in locomotion...");
        }
    }
    // else{
    //     prev_status_msg_.status = GoalStatus::STATUS_UNKNOWN;
    // }

    return BT::NodeStatus::FAILURE;
}