#include <exploration_manager/CheckLocomotionStatus.h>

CheckLocomotionStatus::CheckLocomotionStatus(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    ros::param::get("robot_exploration/min_nav_target_distance", min_nav_target_distance_);
    min_nav_target_distance_ = min_nav_target_distance_*min_nav_target_distance_; // Consider squared
}
//Check if Ros Nav succeed and also that the robot is "close enough" to desired location
BT::NodeStatus CheckLocomotionStatus::tick(){

    msg_ = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", ros::Duration(0.5));

    //Get robot's pose
    try {
        bt_data_->now = ros::Time::now();
        listener_.waitForTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, ros::Duration(0.5f));
        listener_.lookupTransform(bt_data_->world_frame, bt_data_->base_frame, bt_data_->now, bt_data_->last_robot_pose);

    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();

        return BT::NodeStatus::FAILURE;
    }

    // Consider the last in status_list
    if(msg_->status_list.size() > 0){
        // SUCCEEDED
        if(msg_->status_list[msg_->status_list.size()-1].status == 3){
            bt_data_->is_driving = false;
            bt_data_->need_exploration = false;
            bt_data_->finished_exploration = true;
            
            //Check distance to target (to avoid also checking SUCCEED of prev exec.)
            if(pow(bt_data_->last_robot_pose.getOrigin().x() - bt_data_->locomotion_target.position.x,2) +
               pow(bt_data_->last_robot_pose.getOrigin().y() - bt_data_->locomotion_target.position.y,2) < min_nav_target_distance_){

                ROS_WARN("Nav Target Reached!");
                
                bt_data_->is_driving = false;
                bt_data_->need_exploration = false;
                bt_data_->finished_exploration = true;
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