#include <exploration_manager/is_request_active.h>

BT::NodeStatus IsRequestActive(){
    //Check if finished the task sequence
    if(bt_data_->finished_exploration){
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::SUCCESS;
}