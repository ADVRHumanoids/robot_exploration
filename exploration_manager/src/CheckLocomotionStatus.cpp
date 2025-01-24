#include <exploration_manager/CheckLocomotionStatus.h>

CheckLocomotionStatus::CheckLocomotionStatus(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
}
//Check if Ros Nav succeed and also that the robot is "close enough" to desired location
BT::NodeStatus CheckLocomotionStatus::tick(){

    msg_ = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", ros::Duration(0.5));

    //Get robot's pose
    try {
        bt_data.now = ros::Time(0);
        listener_.waitForTransform("map", "pelvis", bt_data.now, ros::Duration(0.5f));
        listener_.lookupTransform("map", "pelvis", bt_data.now, bt_data.last_robot_pose);

    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();

        return BT::NodeStatus::FAILURE;
    }

    // Consider the last in status_list
    if(msg_->status_list.size() > 0){
        // SUCCEEDED
        if(msg_->status_list[msg_->status_list.size()-1].status == 3){
            bt_data.is_driving = false;
            bt_data.need_exploration = false;
            bt_data.finished_exploration = true;
            
            //Check distance to target (to avoid also checking SUCCEED of prev exec.)
            if(pow(bt_data.last_robot_pose.getOrigin().x() - bt_data.locomotion_target.position.x,2) +
               pow(bt_data.last_robot_pose.getOrigin().y() - bt_data.locomotion_target.position.y,2) < 0.04f){ // 0.20m   

                ROS_WARN("Nav Target Reached!");
                
                bt_data.is_driving = false;
                bt_data.need_exploration = false;
                bt_data.finished_exploration = true;
                return BT::NodeStatus::SUCCESS;
            }
        }
        else if(msg_->status_list[msg_->status_list.size()-1].status == 4){
            bt_data.is_driving = false;
            bt_data.need_exploration = true;
            bt_data.finished_exploration = false;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void CheckLocomotionStatus::halt(){

}