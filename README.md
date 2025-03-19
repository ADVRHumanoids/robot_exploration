# robot_exploration - ROS2
Set of packages for robot exploration on occupancy map.

- 2D Occupancy map with octomap from pointcloud (LiDAR, RGB-D,...)
- Simple frontier extraction
- Cost Function for frontier selection

Robot independent, provides as output the goal pose to reach, sent to the MoveBase of the Nav Stack (within the BT).

# Dependencies
- ROS (jazzy)
- XBot
- CartesIO
- iit-centauro-ros-pkg
- realsense and velodyne packages
- octomap
- centauro_ros_nav (branch: ros2)
- perception_utils (branch: ros2)
- object_detection_manager (branch: ros2) [Optional]


# NOTE
The input of the Exploration is the *frame_name* of the object that you want to find. The exploration checks in "KnownTargetPose" if such frame exists. If it exists, then a Nav Target is sent to the object. Otherwise, the robot will start/continue the exploration.


Using frames, if the transform map-obj is no longer valid/published, then exploration resumes. 

# Exploration [with YOLO]
- ros2 launch gazebo_odom start_simulation.launch.py
- ros2 service call /xbotcore/omnisteering/switch std_srvs/srv/SetBool data:\ true\
- ros2 launch centauro_ros_nav centauro_nav.launch.py
- ros2 launch exploration_manager exploration_manager.launch.py
- ros2 run object_detection_manager object_detection_node [Optional]
- ros2 action send_goal /request_exploration exploration_manager_actions/action/RequestExploration object_name:\ \'obj1\'\