#include <exploration_manager/CollectObjectPose.h>

CollectObjectPose::CollectObjectPose(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    ros::param::get("robot_exploration/distance_target_object", distance_target_object_);
}

BT::NodeStatus CollectObjectPose::tick(){
    //Get objects's pose
    try {
        // std::cout << "Look for " << bt_data_->object_name << std::endl;
        bt_data_->now = ros::Time::now();
        listener_.waitForTransform(bt_data_->world_frame, bt_data_->object_name, bt_data_->now, ros::Duration(0.5));
        listener_.lookupTransform(bt_data_->world_frame, bt_data_->object_name, bt_data_->now, bt_data_->object_pose);
    } catch ( tf::TransformException ex ) {
        bt_data_->need_exploration = true;
        bt_data_->known_object_pose = false;
        return BT::NodeStatus::FAILURE;
    }

    // Select nav target in the line, at a distance of 0.85m from object
    angle_ = atan2(bt_data_->object_pose.getOrigin().y() - bt_data_->last_robot_pose.getOrigin().y(),
                   bt_data_->object_pose.getOrigin().x() - bt_data_->last_robot_pose.getOrigin().x());
    
    bt_data_->locomotion_target.position.x = bt_data_->object_pose.getOrigin().x() - distance_target_object_*cos(angle_);
    bt_data_->locomotion_target.position.y = bt_data_->object_pose.getOrigin().y() - distance_target_object_*sin(angle_);

    //TODO: Improve
    bt_data_->locomotion_target.orientation.x = 0;
    bt_data_->locomotion_target.orientation.y = 0;
    bt_data_->locomotion_target.orientation.z = 0;
    bt_data_->locomotion_target.orientation.w = 1;
    
    return BT::NodeStatus::SUCCESS;
}

void CollectObjectPose::halt(){

}