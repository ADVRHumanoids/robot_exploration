#ifndef __IS_REQUEST_ACTIVE__
#define __IS_REQUEST_ACTIVE__

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>

using namespace BT;

BT::NodeStatus IsRequestActive();

#endif
