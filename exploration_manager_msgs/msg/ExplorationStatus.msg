bool active_task
string target_object
bool location_known
geometry_msgs/Point object_target_pos
bool is_driving
geometry_msgs/Point robot_pos
geometry_msgs/Pose nav_target_pose
uint8 frontiers_number
bool finished
uint8 task_id
uint8 images_collected

#ROS2 Client Status
bool save_img_srv
bool cancel_nav_srv
bool get_frontiers_srv
bool nav_to_pose_srv
bool get_objects_srv
bool get_insp_goals_srv
bool send_cand_target_srv