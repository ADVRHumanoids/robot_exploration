# Frontier Extraction - ROS 2
This ROS2 package is used to extract frontier points from 2D occupancy grids of 3D octomap representations.


#### Input:
- occupancy grid
- robot pose

#### Output:
- Frontiers (list of points, mean point)

## frontier_extraction_msgs
This contains the custom message 'Frontier.msg'.

## frontier_extraction_srvs
This contains the custom service 'GetFrontiers.srv'.