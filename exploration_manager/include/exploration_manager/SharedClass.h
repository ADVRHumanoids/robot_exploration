#ifndef __COMMON__
#define __COMMON__

#include <fstream>
#include <array>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf_conversions/tf_eigen.h"

#include <frontier_extraction/Frontier.h>

class SharedClass
{
  public:
    std::string object_name, world_frame, base_frame;

    geometry_msgs::Pose locomotion_target;
    std::vector<frontier_extraction::Frontier> frontiers;
    bool need_exploration, is_driving, finished_exploration;   
    // Eigen::Affine3d last_robot_pose;

    //TF Transforms
    tf::StampedTransform last_robot_pose, object_pose;
    ros::Time now;

    float min_nav_target_distance;
    bool known_object_pose, force_frontier_update;

    //Constructor
    SharedClass(){
      object_name = "obj";
      world_frame = "map";
      base_frame = "base_link";
      frontiers = {};
      need_exploration = false;
      is_driving = false;
      finished_exploration = false;
      known_object_pose = false;
      force_frontier_update = false;
      // last_robot_pose = Eigen::Affine3d::Identity();
      min_nav_target_distance = 0.04f;
    }
};

SharedClass *bt_data_;

#endif
