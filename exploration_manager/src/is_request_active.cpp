#include <exploration_manager/is_request_active.h>

BT::NodeStatus IsRequestActive(){
    //Check if finished the task sequence
    if(bt_data_->current_task  < static_cast<int>(bt_data_->tasks.size()) && bt_data_->active_task){
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}