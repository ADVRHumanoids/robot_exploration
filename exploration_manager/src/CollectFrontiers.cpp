#include <exploration_manager/CollectFrontiers.h>

CollectFrontiers::CollectFrontiers(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    frontier_extract_srv_ = nh_.serviceClient<frontier_extraction::GetFrontiers>("/get_frontiers");
    nav_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

    ros::param::get("robot_exploration/exploration/timer_frontiers", timer_frontiers_);
    time_diff_ = 2.0f*timer_frontiers_;

    prev_time_ = ros::Time::now();
}

BT::NodeStatus CollectFrontiers::tick(){

    //Get robot's pose
    try {
        now_ = ros::Time::now();
        listener_.waitForTransform(bt_data_->world_frame, bt_data_->base_frame, now_, ros::Duration(0.5));
        listener_.lookupTransform(bt_data_->world_frame, bt_data_->base_frame, now_, bt_data_->last_robot_pose);

    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();

        return BT::NodeStatus::FAILURE;
    }

    time_diff_ = static_cast<float>(now_.sec) - static_cast<float>(prev_time_.sec) +
                 (static_cast<float>(now_.nsec) - static_cast<float>(prev_time_.nsec))*10e-9;

    if(time_diff_ > timer_frontiers_){
        prev_time_ = now_;
        //Ask for the frontiers found 
        get_frontiers_.request.robot_pose.x = bt_data_->last_robot_pose.getOrigin().x();
        get_frontiers_.request.robot_pose.y = bt_data_->last_robot_pose.getOrigin().y();

        frontier_extract_srv_.call(get_frontiers_);

        bt_data_->frontiers =  get_frontiers_.response.frontiers;

        if(bt_data_->frontiers.size() == 0){
            ROS_WARN("No more frontiers! Exploration finished!");
            //Cancel Nav Pose
            nav_cancel_pub_.publish(cancel_nav_req_);

            bt_data_->need_exploration = false;
            bt_data_->is_driving = false;
            bt_data_->finished_exploration = true;

            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::SUCCESS;
}

void CollectFrontiers::halt(){

}
