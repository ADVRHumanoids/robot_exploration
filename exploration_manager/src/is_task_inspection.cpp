#include <exploration_manager/is_task_inspection.h>

BT::NodeStatus IsTaskInspection(){
    //Check if current task is inspection
    if(bt_data_->current_task >= 0 &&
       bt_data_->current_task < static_cast<int>(bt_data_->tasks.size()) &&
       bt_data_->tasks[bt_data_->current_task].id == 2){
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}