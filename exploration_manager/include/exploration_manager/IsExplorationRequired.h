#ifndef __IS_TASK_COLLECTED__
#define __IS_TASK_COLLECTED__

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <exploration_manager/SharedClass.h>

using namespace BT;

BT::NodeStatus IsExplorationRequired();

#endif
