#include <exploration_manager/IsExplorationRequired.h>

BT::NodeStatus IsExplorationRequired(){
    //Check if finished the task sequence
    if(bt_data_->need_exploration)
        return BT::NodeStatus::SUCCESS;
    
    return BT::NodeStatus::FAILURE;
}