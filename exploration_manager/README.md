# exploration_manager - ROS2
This package containes all the nodes for the exploration BT.

# Dependencies

- ROS (jazzy)
- behaviortree_cpp
- tf2
- exploration_manager_actions
- frontier_extraction_msgs
- frontier_extraction_srvs
- centauro_ros_nav
- centauro_ros_nav_srvs
- action_msgs
- actionlib_msgs
- object_detection_msgs
- object_detection_srvs
- exploration_manager_msgs
- define_inspection_goals_srvs


### behavior_trees
This folder contains the _.xml_ with the BT definition.

### config
The config folder has a configuration file, _exploration_config_centauro.yaml_, that contains information related to world and base frames, as well as values for controlling the exploration in terms of time, distances, and costs.

## launch
The launch file available for this package runs the BT and the 2D/3D frontier extraction.
