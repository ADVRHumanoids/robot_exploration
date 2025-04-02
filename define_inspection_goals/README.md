# Define Inspection Goals

This ROS2 package contains a node that exposes the service '/get_inspection_goals'. This is used to define N inspection points around the interesting object, so that the robot can move to those points and acquire data of the object.


### Input:
- 2D Occupancy (global costmap)
- Object Location

### Output:
- Nav Waypoints (facing the object)


## define_inspection_srvs
It contains the custom service 'GetInspectionGoals.srv', with the following definition:
```
uint8 goals_number
geometry_msgs/Point object_pos
---
geometry_msgs/Pose[] poses
```