#include <exploration_manager/Explore.h>

Explore::Explore(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    ros::param::get("robot_exploration/exploration/min_dist_frontier_robot", min_dist_frontier_robot_);
    min_dist_frontier_robot_ = min_dist_frontier_robot_*min_dist_frontier_robot_;

    ros::param::get("robot_exploration/exploration/cost_function/n_points", cost_n_points_);
    ros::param::get("robot_exploration/exploration/cost_function/euclidean_distance", cost_euclidean_distance_);
    ros::param::get("robot_exploration/exploration/cost_function/distance_prev_target", cost_distance_prev_target_);
}

BT::NodeStatus Explore::tick(){

    //Get robot's pose
    try {
        bt_data_->now = ros::Time::now();
        listener_.waitForTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, ros::Duration(2.0));
        listener_.lookupTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, bt_data_->last_robot_pose);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();

        return BT::NodeStatus::FAILURE;
    }

    //Analyze frontiers and select point to explore
    //Compute cost/utility function
    cost_max_ = -1.0f;
    max_frontier_idx_ = -1;

    // ROS_WARN("Number of Clusters: %d", bt_data_->frontiers.size());

    for(int i = 0; i < bt_data_->frontiers.size(); i++){
        if(bt_data_->frontiers[i].centroid.x == 0.0f || 
           bt_data_->frontiers[i].centroid.y == 0.0f)
           continue;
        
        squared_euclidean_distance_ = (pow(bt_data_->frontiers[i].centroid.x - bt_data_->last_robot_pose.getOrigin().x(), 2) + 
                                       pow(bt_data_->frontiers[i].centroid.y - bt_data_->last_robot_pose.getOrigin().y(), 2));

        squared_distance_to_prev_target_ = (pow(bt_data_->frontiers[i].centroid.x -  bt_data_->locomotion_target.position.x, 2) + 
                                            pow(bt_data_->frontiers[i].centroid.y - bt_data_->locomotion_target.position.y, 2));
        
        if(squared_euclidean_distance_ < min_dist_frontier_robot_)
            continue;

        cost_ = cost_n_points_ * bt_data_->frontiers[i].number_of_points + 
                cost_euclidean_distance_ * 1.0f/squared_euclidean_distance_ + 
                cost_distance_prev_target_ * 1.0f/squared_distance_to_prev_target_;

        if(cost_ > cost_max_){
            cost_max_ = cost_;
            max_frontier_idx_ = i;
        }
    }
    
    if(max_frontier_idx_ == -1)
        return BT::NodeStatus::FAILURE;

    bt_data_->locomotion_target.position.x = bt_data_->frontiers[max_frontier_idx_].centroid.x;
    bt_data_->locomotion_target.position.y = bt_data_->frontiers[max_frontier_idx_].centroid.y;
    
    bt_data_->locomotion_target.orientation.w = 1.0f;    

    return BT::NodeStatus::SUCCESS;
}

void Explore::halt(){

}
