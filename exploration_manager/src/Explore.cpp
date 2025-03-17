#include <exploration_manager/Explore.h>

Explore::Explore(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    ros::param::get("robot_exploration/exploration/min_dist_frontier_robot", min_dist_frontier_robot_);
    min_dist_frontier_robot_ = min_dist_frontier_robot_*min_dist_frontier_robot_;

    ros::param::get("robot_exploration/exploration/cost_function/n_points", cost_n_points_);
    ros::param::get("robot_exploration/exploration/cost_function/euclidean_distance", cost_euclidean_distance_);
    ros::param::get("robot_exploration/exploration/cost_function/rotation_distance", cost_rotation_distance_);
    ros::param::get("robot_exploration/exploration/cost_function/distance_prev_target", cost_distance_prev_target_);
    ros::param::get("robot_exploration/exploration/cost_function/close_frontiers", cost_neighbors_);

    close_frontiers_ = {};
}

BT::NodeStatus Explore::tick(){

    //Get robot's pose
    try {
        bt_data_->now = ros::Time::now();
        listener_.waitForTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, ros::Duration(2.0));
        listener_.lookupTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, bt_data_->last_robot_pose);

        //TODO: Use fastatan
        robot_yaw_ = 2.0*atan2(bt_data_->last_robot_pose.getRotation().getZ(), 
                               bt_data_->last_robot_pose.getRotation().getW());

    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        return BT::NodeStatus::FAILURE;
    }

    //Analyze frontiers and select point to explore
    //Compute cost/utility function
    cost_min_ = 100000.0f;
    max_frontier_idx_ = -1;
    angle_ = 6.28f;

    // ROS_WARN("Number of Clusters: %d", bt_data_->frontiers.size());

    close_frontiers_.resize(bt_data_->frontiers.size());
    for(int i = 0; i < bt_data_->frontiers.size(); i++){
        close_frontiers_[i] = 0;
    }

    for(int i = 0; i < bt_data_->frontiers.size(); i++){
        if(bt_data_->frontiers[i].centroid.x == 0.0f || 
           bt_data_->frontiers[i].centroid.y == 0.0f)
           continue;

        //Compute distance between this frontier and all the others
        for(int j = i+1; j < bt_data_->frontiers.size(); j++){
            temp_distance_ += (pow(bt_data_->frontiers[i].centroid.x - bt_data_->frontiers[j].centroid.x, 2) + 
                               pow(bt_data_->frontiers[i].centroid.y - bt_data_->frontiers[j].centroid.y, 2));
            
            // Check numbers of frontiers in a neighboorhood of 2m
            if(temp_distance_ < 4.0f){
                close_frontiers_[i] += 1;
                close_frontiers_[j] += 1;
            }
        }
        //Distance to robot
        squared_euclidean_distance_ = (pow(bt_data_->frontiers[i].centroid.x - bt_data_->last_robot_pose.getOrigin().x(), 2) + 
                                       pow(bt_data_->frontiers[i].centroid.y - bt_data_->last_robot_pose.getOrigin().y(), 2));

        //TODO get from param
        //Avoid sending targets close to the robot (within nav tolerance), otherwise: stuck!
        if(squared_euclidean_distance_ < 0.16f)
            continue;


        //Distance to prev nav target(frontier)
        squared_distance_to_prev_target_ = (pow(bt_data_->frontiers[i].centroid.x - bt_data_->locomotion_target.position.x, 2) + 
                                            pow(bt_data_->frontiers[i].centroid.y - bt_data_->locomotion_target.position.y, 2));
        
        //TODO: Use fastatan
        temp_ang_ = atan2(bt_data_->frontiers[i].centroid.y - bt_data_->last_robot_pose.getOrigin().y(),
                          bt_data_->frontiers[i].centroid.x - bt_data_->last_robot_pose.getOrigin().x());
        
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

void Explore::halt(){

}
