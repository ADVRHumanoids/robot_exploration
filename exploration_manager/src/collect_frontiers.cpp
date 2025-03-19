#include <exploration_manager/collect_frontiers.h>

CollectFrontiers::CollectFrontiers(const std::string& name,
                                   const BT::NodeConfig &config,
                                   rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   
    //Frontier Extraction Client
    frontier_extract_srv_ = node_->create_client<frontier_extraction_srvs::srv::GetFrontiers>("/get_frontiers");
    
    //Nav2 Action Client
    nav2_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_,
                                                                    "/navigate_to_pose");
    //Param get
    node_->declare_parameter("robot_exploration.exploration.timer_frontiers", 5.0);
    timer_frontiers_ = node_->get_parameter("robot_exploration.exploration.timer_frontiers").as_double();

    time_diff_ = timer_frontiers_;

    prev_time_ = node_->get_clock()->now();
    now_ = prev_time_;

    //TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    get_frontiers_req_ = std::make_shared<frontier_extraction_srvs::srv::GetFrontiers::Request>();
}

BT::NodeStatus CollectFrontiers::tick(){
    // RCLCPP_INFO(node_->get_logger(), "CollectFrontiers");

    //Get robot's pose
    try {
        now_ = node_->get_clock()->now();
        bt_data_->last_robot_pose = tf_buffer_->lookupTransform(
                                        bt_data_->base_frame, bt_data_->world_frame,
                                        now_);        
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(), "CollectFrontiers: Could not transform!");

        return BT::NodeStatus::FAILURE;
    }

    time_diff_ =  static_cast<float>(now_.seconds()) - static_cast<float>(prev_time_.seconds()) +
                 (static_cast<float>(now_.nanoseconds()) - static_cast<float>(prev_time_.nanoseconds()))*10e-9;

    // RCLCPP_INFO(node_->get_logger(), "Timer: %f", time_diff_);

    if(time_diff_ > timer_frontiers_ || bt_data_->force_frontier_update){
        
        RCLCPP_INFO(node_->get_logger(), "CollectFrontiers: Timer: %f - ForceUpdate: %d", time_diff_, bt_data_->force_frontier_update);
       
        prev_time_ = now_;

        bt_data_->force_frontier_update = false;
        //Ask for the frontiers found 
        get_frontiers_req_->robot_pose.x = bt_data_->last_robot_pose.transform.translation.x;
        get_frontiers_req_->robot_pose.y = bt_data_->last_robot_pose.transform.translation.y;
        get_frontiers_req_->robot_pose.z = bt_data_->last_robot_pose.transform.translation.z;
    
        get_frontiers_fut_ = frontier_extract_srv_->async_send_request(get_frontiers_req_);
        get_frontiers_res_ = get_frontiers_fut_.get(); // Blocking call

        if (get_frontiers_res_){
            bt_data_->frontiers =  get_frontiers_res_->frontiers;
        } else {
            return BT::NodeStatus::FAILURE;
        }

        if(bt_data_->frontiers.size() == 0){
            RCLCPP_INFO(node_->get_logger(), "No more frontiers! Exploration finished!");
            //Cancel Nav Pose
            nav2_client_ptr_->async_cancel_all_goals();

            bt_data_->need_exploration = false;
            bt_data_->is_driving = false;
            bt_data_->finished_exploration = true;

            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::SUCCESS;
}
