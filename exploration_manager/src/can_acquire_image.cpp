#include <exploration_manager/can_acquire_image.h>

BT::NodeStatus CanAcquireImage(){

    /*
        TODO:
        If:
        - is not driving
        - target reached
        - object in FoV
    */

    if(bt_data_->acquire_image && !bt_data_->is_driving)
        return BT::NodeStatus::SUCCESS;
    
    return BT::NodeStatus::FAILURE;
}