#include <exploration_manager/explore.h>

Explore::Explore(const std::string& name,
                 const BT::NodeConfig &config,
                 rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    node_->declare_parameter("robot_exploration.exploration.min_dist_frontier_robot", 0.25);
    node_->declare_parameter("robot_exploration.exploration.cost_function.n_points", 4.0);
    node_->declare_parameter("robot_exploration.exploration.cost_function.euclidean_distance", 12.0);
    node_->declare_parameter("robot_exploration.exploration.cost_function.rotation_distance", 25.0);
    node_->declare_parameter("robot_exploration.exploration.cost_function.distance_prev_target", 17.0);
    node_->declare_parameter("robot_exploration.exploration.cost_function.close_frontiers", 5.0);
    
    min_dist_frontier_robot_ = node_->get_parameter("robot_exploration.exploration.min_dist_frontier_robot").as_double();
    min_dist_frontier_robot_ = min_dist_frontier_robot_*min_dist_frontier_robot_;

    cost_n_points_ = node_->get_parameter("robot_exploration.exploration.cost_function.n_points").as_double();
    cost_euclidean_distance_ = node_->get_parameter("robot_exploration.exploration.cost_function.euclidean_distance").as_double();
    cost_rotation_distance_ = node_->get_parameter("robot_exploration.exploration.cost_function.rotation_distance").as_double();
    cost_distance_prev_target_ = node_->get_parameter("robot_exploration.exploration.cost_function.distance_prev_target").as_double();
    cost_neighbors_ = node_->get_parameter("robot_exploration.exploration.cost_function.close_frontiers").as_double();
    
    close_frontiers_ = {};

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus Explore::tick(){

    // RCLCPP_INFO(node_->get_logger(), "Explore!");

    //Get robot's pose
    try {
        // std::cout << "Look for " << bt_data_->object_name << std::endl;
        bt_data_->now = node_->get_clock()->now();
        
        bt_data_->last_robot_pose = tf_buffer_->lookupTransform(
                                        bt_data_->base_frame, bt_data_->world_frame,
                                        bt_data_->now);

        //TODO: Use fastatan
        robot_yaw_ = 2.0*atan2(bt_data_->last_robot_pose.transform.rotation.z, 
                               bt_data_->last_robot_pose.transform.rotation.w);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(node_->get_logger(), "Explore: Robot pose not found!");
        return BT::NodeStatus::FAILURE;
    }

    //Analyze frontiers and select point to explore
    //Compute cost/utility function
    cost_min_ = 100000.0f;
    max_frontier_idx_ = -1;
    angle_ = 6.28f;

    // ROS_WARN("Number of Clusters: %d", static_cast<int>(bt_data_->frontiers.size()));

    close_frontiers_.resize(static_cast<int>(bt_data_->frontiers.size()));
    for(int i = 0; i < static_cast<int>(bt_data_->frontiers.size()); i++){
        close_frontiers_[i] = 0;
    }

    for(int i = 0; i < static_cast<int>(bt_data_->frontiers.size()); i++){
        if(bt_data_->frontiers[i].centroid.x == 0.0f || 
           bt_data_->frontiers[i].centroid.y == 0.0f)
           continue;

        //Compute distance between this frontier and all the others
        for(int j = i+1; j < static_cast<int>(bt_data_->frontiers.size()); j++){
            temp_distance_ += (pow(bt_data_->frontiers[i].centroid.x - bt_data_->frontiers[j].centroid.x, 2) + 
                               pow(bt_data_->frontiers[i].centroid.y - bt_data_->frontiers[j].centroid.y, 2));
            
            // Check numbers of frontiers in a neighboorhood of 2m
            if(temp_distance_ < 4.0f){
                close_frontiers_[i] += 1;
                close_frontiers_[j] += 1;
            }
        }
        //Distance to robot
        squared_euclidean_distance_ = (pow(bt_data_->frontiers[i].centroid.x - bt_data_->last_robot_pose.transform.translation.x, 2) + 
                                       pow(bt_data_->frontiers[i].centroid.y - bt_data_->last_robot_pose.transform.translation.y, 2));

        //TODO get from param
        //Avoid sending targets close to the robot (within nav tolerance), otherwise: stuck!
        if(squared_euclidean_distance_ < 0.16f)
            continue;


        //Distance to prev nav target(frontier)
        squared_distance_to_prev_target_ = (pow(bt_data_->frontiers[i].centroid.x - bt_data_->locomotion_target.position.x, 2) + 
                                            pow(bt_data_->frontiers[i].centroid.y - bt_data_->locomotion_target.position.y, 2));
        
        //TODO: Use fastatan
        temp_ang_ = atan2(bt_data_->frontiers[i].centroid.y - bt_data_->last_robot_pose.transform.translation.y,
                          bt_data_->frontiers[i].centroid.x - bt_data_->last_robot_pose.transform.translation.x);
        
        temp_ang_diff_ = fabs(robot_yaw_ - temp_ang_);
        //Transform in 0-180° difference
        if(temp_ang_diff_ > 6.28f)
            temp_ang_diff_ -= 6.28f*floor(temp_ang_diff_/6.28f); 

        if(temp_ang_diff_ > 3.14f)
            temp_ang_diff_ = 6.28f - temp_ang_diff_; 
        

        //Frontier size - Distance to robot - Distance to prev nav target - frontiers close
        cost_ = cost_n_points_ * 1.0f/(static_cast<float>(1 + bt_data_->frontiers[i].number_of_points)) + 
                cost_euclidean_distance_ * squared_euclidean_distance_ + 
                cost_rotation_distance_ * temp_ang_diff_ +
                cost_distance_prev_target_ * squared_distance_to_prev_target_ +
                cost_neighbors_ * 1.0f/(static_cast<float>(1 + close_frontiers_[i]));
        
        if(cost_ < cost_min_){
            cost_min_ = cost_;
            max_frontier_idx_ = i;
            angle_ = temp_ang_;
        }
    }
    
    if(max_frontier_idx_ == -1)
        return BT::NodeStatus::FAILURE;

    bt_data_->locomotion_target.position.x = bt_data_->frontiers[max_frontier_idx_].centroid.x;
    bt_data_->locomotion_target.position.y = bt_data_->frontiers[max_frontier_idx_].centroid.y;
    
    //Direction to face the target
    bt_data_->locomotion_target.orientation.z = sin(angle_/2);
    bt_data_->locomotion_target.orientation.w = cos(angle_/2);

    return BT::NodeStatus::SUCCESS;
}