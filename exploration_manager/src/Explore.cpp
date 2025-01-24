#include <exploration_manager/Explore.h>

Explore::Explore(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
}

BT::NodeStatus Explore::tick(){

    //Get robot's pose
    try {
        bt_data.now = ros::Time(0);
        listener_.waitForTransform("map", "pelvis", bt_data.now, ros::Duration(2.0));
        listener_.lookupTransform("map", "pelvis", bt_data.now, bt_data.last_robot_pose);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();

        return BT::NodeStatus::FAILURE;
    }

    //Analyze frontiers and select point to explore
    //Compute cost/utility function
    cost_max_ = -1.0f;
    max_frontier_idx_ = -1;

    // ROS_WARN("Number of Clusters: %d", bt_data.frontiers.size());

    for(int i = 0; i < bt_data.frontiers.size(); i++){
        if(bt_data.frontiers[i].centroid.x == 0.0f || 
           bt_data.frontiers[i].centroid.y == 0.0f)
           continue;
        
        squared_euclidean_distance_ = (pow(bt_data.frontiers[i].centroid.x - bt_data.last_robot_pose.getOrigin().x(), 2) + 
                                       pow(bt_data.frontiers[i].centroid.y - bt_data.last_robot_pose.getOrigin().y(), 2));

        squared_distance_to_prev_target_ = (pow(bt_data.frontiers[i].centroid.x -  bt_data.locomotion_target.position.x, 2) + 
                                            pow(bt_data.frontiers[i].centroid.y - bt_data.locomotion_target.position.y, 2));
        //0.25cm
        if(squared_euclidean_distance_ < 0.0625f)
            continue;

        cost_ =  1.0f * bt_data.frontiers[i].number_of_points + 
                25.0f * 1.0f/squared_euclidean_distance_ + 
                75.0f * 1.0f/squared_distance_to_prev_target_;

        if(cost_ > cost_max_){
            cost_max_ = cost_;
            max_frontier_idx_ = i;
        }
    }
    
    if(max_frontier_idx_ == -1)
        return BT::NodeStatus::FAILURE;

    bt_data.locomotion_target.position.x = bt_data.frontiers[max_frontier_idx_].centroid.x;
    bt_data.locomotion_target.position.y = bt_data.frontiers[max_frontier_idx_].centroid.y;
    
    bt_data.locomotion_target.orientation.w = 1.0f;    

    return BT::NodeStatus::SUCCESS;
}

void Explore::halt(){

}
