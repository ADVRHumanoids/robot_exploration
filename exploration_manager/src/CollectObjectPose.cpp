#include <exploration_manager/CollectObjectPose.h>

CollectObjectPose::CollectObjectPose(const std::string& name) :
    BT::AsyncActionNode(name, {})
{

}

BT::NodeStatus CollectObjectPose::tick(){
    //Get objects's pose
    try {
        std::cout << "Look for " << bt_data.object_name << std::endl;
        bt_data.now = ros::Time::now();
        listener_.waitForTransform("map", bt_data.object_name, bt_data.now, ros::Duration(1.0));
        listener_.lookupTransform("map", bt_data.object_name, bt_data.now, bt_data.object_pose);
    } catch ( tf::TransformException ex ) {
        return BT::NodeStatus::FAILURE;
    }

    // Select nav target in the line, at a distance of 0.85m from object
    angle_ = atan2(bt_data.object_pose.getOrigin().y() - bt_data.last_robot_pose.getOrigin().y(),
                   bt_data.object_pose.getOrigin().x() - bt_data.last_robot_pose.getOrigin().x());
    
    bt_data.locomotion_target.position.x = bt_data.object_pose.getOrigin().x() - 0.85f*cos(angle_);
    bt_data.locomotion_target.position.y = bt_data.object_pose.getOrigin().y() - 0.85f*sin(angle_);

    //TODO: Improve
    bt_data.locomotion_target.orientation.x = 0;
    bt_data.locomotion_target.orientation.y = 0;
    bt_data.locomotion_target.orientation.z = 0;
    bt_data.locomotion_target.orientation.w = 1;
    
    return BT::NodeStatus::SUCCESS;
}

void CollectObjectPose::halt(){

}