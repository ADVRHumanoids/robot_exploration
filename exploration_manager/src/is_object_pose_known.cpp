#include <exploration_manager/is_object_pose_known.h>

BT::NodeStatus IsObjectPoseKnown(){
    //Check if location of object is known
    if(bt_data_->known_object_pose)
        return BT::NodeStatus::SUCCESS;
    
    return BT::NodeStatus::FAILURE;
}