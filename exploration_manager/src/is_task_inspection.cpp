#include <exploration_manager/is_task_inspection.h>

BT::NodeStatus IsTaskInspection(){
    //Check if current task is inspection
    if(bt_data_->tasks[bt_data_->current_task].id == 1){
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}