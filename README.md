# robot_exploration - ROS2
Set of packages to perform robot exploration on 2D/3D occupancy map.

- 2D/3D Occupancy map with octomap from pointcloud (LiDAR, RGB-D,...)
- Simple frontier extraction
- Cost Function for frontier selection

Inputs: 
- Sensors' reading [Pointcloud]
- global/local maps
- TFs
- Exploration Request

Output:
- Nav goal to reach --> Sent to Nav2

## NOTE

The input of the Exploration is the _class name_ of the object that you want to find. The exploration checks in "KnownTargetPose" if such object exists, asking to the EnvironmentKnowledgeManager. If it exists, then a Nav Target is sent to the object. Otherwise, the robot will start/continue the exploration.

Using frames, if the transform map-obj is no longer valid/published, then exploration resumes. 

# Dependencies

- ROS (jazzy)
- XBot
- CartesIO
- iit-centauro-ros-pkg
- realsense and velodyne packages
- octomap
- octomap_server
- centauro_ros_nav (branch: ros2)
- perception_utils (branch: ros2)
- object_detection_manager (branch: ros2)

## Packages

This repo contains several pacakges:
- define_inspection_goals
- define_inspection_goals_srvs
- exploration_gui
- exploration_manager
- exploration_manager_actions
- exploration_manager_msgs
- frontier_extraction
- frontier_extraction_msg
- frontier_extraction_srvs
- gazebo_odom

# Run Framework [with YOLO]
Launch the Gazebo environment with XBot2 running + enable driving:
- ros2 launch gazebo_odom start_simulation.launch.py
- ros2 service call /xbotcore/omnisteering/switch std_srvs/srv/SetBool data:\ true\
Launch Nav2 framework:
- ros2 launch centauro_ros_nav centauro_nav.launch.py
Launch exploration BT:
- ros2 launch exploration_manager exploration_manager.launch.py
Run YOLO object detection
- ros2 run object_detection_manager object_detection_node
Run exploration GUI for monitoring and simpify exploration requests: 
- ros2 run exploration_gui exploration_gui_node

_Send the target:_

To send the target from the GUI you have to go to the corresponding tab and select the object class that you want to deal with. This can be found in the drop down menu or, in the case of "Others", it can be written in the text box.
Then you can send the request to the robot throught the 2 buttons. _Inspect_ starts the visual inspection operation while _Reach_ simply moves the robot to the object. 