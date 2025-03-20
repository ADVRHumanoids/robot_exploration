#include <exploration_manager/check_locomotion_status.h>

CheckLocomotionStatus::CheckLocomotionStatus(const std::string& name,
                                             const BT::NodeConfig &config,
                                             rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    node_->declare_parameter("robot_exploration.min_nav_target_distance", 0.85);
    min_nav_target_distance_ = node_->get_parameter("robot_exploration.min_nav_target_distance").as_double();
    min_nav_target_distance_ = min_nav_target_distance_*min_nav_target_distance_; // Consider squared

    node_->declare_parameter("robot_exploration.min_dist_frontier_robot", 0.85);
    min_frontier_distance_ = node_->get_parameter("robot_exploration.min_dist_frontier_robot").as_double();
    min_frontier_distance_ = min_frontier_distance_*min_frontier_distance_; // Consider squared

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      nav_status_sub_ = node_->create_subscription<actionlib_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&CheckLocomotionStatus::getNavStatus, this, _1));
}

void CheckLocomotionStatus::getNavStatus(const actionlib_msgs::msg::GoalStatusArray::SharedPtr msg){
    msg_ = msg;
}

//Check if Ros Nav succeed and also that the robot is "close enough" to desired location
BT::NodeStatus CheckLocomotionStatus::tick(){
    // RCLCPP_INFO(node_->get_logger(), "CheckLocomotionStatus");
        
    //Get robot's pose
     try {
        // std::cout << "Look for " << bt_data_->object_name << std::endl;
        bt_data_->now = node_->get_clock()->now();
        
        bt_data_->last_robot_pose = tf_buffer_->lookupTransform(
                                        bt_data_->world_frame, bt_data_->base_frame,
                                        bt_data_->now);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "CheckLocomotionStatus: Could not transform!");
        return BT::NodeStatus::FAILURE;
    }

    if(msg_ == nullptr)
        return BT::NodeStatus::FAILURE;

    status_msg_id_ = msg_->status_list.size() - 1;

    // Consider the last in status_list
    if(status_msg_id_ >= 0){
        
        // If Exploring and close to nav target
        if(!bt_data_->known_object_pose && msg_->status_list[status_msg_id_].status == GoalStatus::ACTIVE &&
            (pow(bt_data_->last_robot_pose.transform.translation.x - bt_data_->locomotion_target.position.x,2) +
             pow(bt_data_->last_robot_pose.transform.translation.y - bt_data_->locomotion_target.position.y,2) < min_frontier_distance_))
        {
            RCLCPP_INFO(node_->get_logger(), "Force Frontier Update");
            bt_data_->force_frontier_update = true;
        }
        else if(msg_->status_list[status_msg_id_].status == GoalStatus::SUCCEEDED){                
            bt_data_->is_driving = false;
            // bt_data_->finished_exploration = true;
            
            //Check distance to target (to avoid also checking SUCCEED of prev exec.)
            if(pow(bt_data_->last_robot_pose.transform.translation.x - bt_data_->locomotion_target.position.x,2) +
               pow(bt_data_->last_robot_pose.transform.translation.y - bt_data_->locomotion_target.position.y,2) < min_nav_target_distance_){

                RCLCPP_INFO(node_->get_logger(), "Nav Target Reached!");

                //If we arrived to the object --> Finish, else change frontier
                bt_data_->need_exploration = !(bt_data_->known_object_pose);
                bt_data_->finished_exploration = bt_data_->known_object_pose;

                if(!bt_data_->known_object_pose)
                    bt_data_->force_frontier_update = true;

                if(bt_data_->finished_exploration)
                    return BT::NodeStatus::FAILURE;
                else
                    return BT::NodeStatus::SUCCESS;
            }
            else{
                RCLCPP_INFO(node_->get_logger(), "Locomotion Ended...");                
            }
        }
        else if(msg_->status_list[status_msg_id_].status == GoalStatus::ABORTED){
            RCLCPP_INFO(node_->get_logger(), "Locomotion Aborted...");  
            bt_data_->is_driving = false;
            bt_data_->need_exploration = true;
            bt_data_->finished_exploration = false;
        }
        else{
            RCLCPP_INFO(node_->get_logger(), "Not previous cases in locomotion...");
        }
    }

    return BT::NodeStatus::FAILURE;
}