#include <exploration_manager/CheckLocomotionStatus.h>

CheckLocomotionStatus::CheckLocomotionStatus(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    ros::param::get("robot_exploration/min_nav_target_distance", min_nav_target_distance_);
    min_nav_target_distance_ = min_nav_target_distance_*min_nav_target_distance_; // Consider squared

    ros::param::get("robot_exploration/exploration/min_dist_frontier_robot", min_frontier_distance_);
    min_frontier_distance_ = min_frontier_distance_*min_frontier_distance_; // Consider squared

    nav_status_sub_ = nh_.subscribe("/move_base/status", 1, &CheckLocomotionStatus::getNavStatus, this);
}


void CheckLocomotionStatus::getNavStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    msg_ = msg;
}

//Check if Ros Nav succeed and also that the robot is "close enough" to desired location
BT::NodeStatus CheckLocomotionStatus::tick(){

    // msg_ = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", ros::Duration(0.5));
    
    //Get robot's pose
    try {
        bt_data_->now = ros::Time::now();
        listener_.waitForTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, ros::Duration(0.5));
        listener_.lookupTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, bt_data_->last_robot_pose);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        return BT::NodeStatus::FAILURE;
    }

    if(msg_ == nullptr)
        return BT::NodeStatus::FAILURE;

    // Consider the last in status_list
    if(msg_->status_list.size() > 0){
        
        // SUCCEEDED
        // If Exploring and close to nav target
        if(!bt_data_->known_object_pose && msg_->status_list[msg_->status_list.size()-1].status == 1 &&
            (pow(bt_data_->last_robot_pose.getOrigin().x() - bt_data_->locomotion_target.position.x,2) +
             pow(bt_data_->last_robot_pose.getOrigin().y() - bt_data_->locomotion_target.position.y,2) < min_frontier_distance_))
        {
            ROS_WARN("Force Frontier Update");
            bt_data_->force_frontier_update = true;
        }
        else if(msg_->status_list[msg_->status_list.size()-1].status == 3){                
            bt_data_->is_driving = false;
            // bt_data_->finished_exploration = true;
            
            //Check distance to target (to avoid also checking SUCCEED of prev exec.)
            if(pow(bt_data_->last_robot_pose.getOrigin().x() - bt_data_->locomotion_target.position.x,2) +
               pow(bt_data_->last_robot_pose.getOrigin().y() - bt_data_->locomotion_target.position.y,2) < min_nav_target_distance_){

                ROS_WARN("Nav Target Reached!");

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
        }
        else if(msg_->status_list[msg_->status_list.size()-1].status == 4){
            bt_data_->is_driving = false;
            bt_data_->need_exploration = true;
            bt_data_->finished_exploration = false;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void CheckLocomotionStatus::halt(){

}