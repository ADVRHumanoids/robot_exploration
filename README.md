# robot_exploration
Set of packages for robot exploration on occupancy map.

- 2D Occupancy map with octomap from pointcloud (LiDAR, RGB-D,...)
- Simple frontier extraction
- Cost Function for frontier selection

Robot independent, provides as output the goal pose to reach, sent to the MoveBase of the Nav Stack (within the BT).

# Dependencies
- ROS (noetic)
- XBot
- CartesIO
- iit-centauro-ros-pkg
- realsense and velodyne packages
- centauro_ros_nav (branch: exploration)
- sudo apt install ros-$ROS_DISTRO-aruco-detect
- octomap
- hhcm_perception [Optional]


# How to launch [Only exploration]
- roslaunch gazebo_odom start_simulation.launch [Launch gazebo, rviz and gazebo odom]
- xbot2-core --simtime
- rosservice call /xbotcore/homing/switch 1
- rosservice call /xbotcore/omnisteering/switch 1
- roslaunch hhcm_perception filtering.launch [Optional: but change ros_nav according to point cloud]
- roslaunch centauro_ros_nav centauro_nav.launch
- roslaunch exploration_manager exploration_manager.launch
- 

# NOTE
The input of the Exploration is the *frame_name* of the object that you want to find. The exploration checks in "KnownTargetPose" if such frame exists. If it exists, then a Nav Target is sent to the object. Otherwise, the robot will start/continue the exploration.