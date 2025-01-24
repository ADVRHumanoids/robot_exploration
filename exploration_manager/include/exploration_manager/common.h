#ifndef __COMMON__
#define __COMMON__

#include <fstream>
#include <array>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf_conversions/tf_eigen.h"

#include <frontier_extraction/Frontier.h>

class commonData
{
  public:
    std::string object_name;

    geometry_msgs::Pose locomotion_target;
    std::vector<frontier_extraction::Frontier> frontiers;
    bool need_exploration, is_driving, finished_exploration;   
    // Eigen::Affine3d last_robot_pose;

    //TF Listener and structs
    tf::StampedTransform last_robot_pose, object_pose;
    ros::Time now;

    //Constructor
    commonData(){
      object_name = "obj";
      frontiers = {};
      need_exploration = false;
      is_driving = false;
      finished_exploration = false;
      // last_robot_pose = Eigen::Affine3d::Identity();
    }
};

commonData bt_data;//= commonData();

#endif
