#ifndef __COMMON__
#define __COMMON__

#include <fstream>
#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rclcpp/rclcpp.hpp"

#include <frontier_extraction_msgs/msg/frontier.hpp>

#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

/*
  Task ID:  
      - 1 Find/Reach Object
      - 2 Inspect Object
*/

#define INSPECTION_IMAGES 2

class Task{
  public:

    Task(unsigned int task_id = 0, std::string object_name="obj"):
      id(task_id), object_name(object_name)
    {}
    
    unsigned int id;
    std::string object_name;
    int inspection_steps, images_collected;
};

class SharedClass
{
  public:
    std::string world_frame, base_frame;

    geometry_msgs::msg::Pose locomotion_target;
    std::vector<frontier_extraction_msgs::msg::Frontier> frontiers;
    bool need_exploration, is_driving, finished_exploration, active_task;   

    //TF Transforms
    geometry_msgs::msg::TransformStamped last_robot_pose, object_pose;
    rclcpp::Time now;

    float min_nav_target_distance;
    bool known_object_pose, force_frontier_update, acquire_image;

    std::vector<Task> tasks;
    
    int current_task;

    //Constructor
    SharedClass(){
      tasks = {};
      current_task = 0;

      world_frame = "map";
      base_frame = "base_link";

      frontiers = {};
      active_task = false;
      need_exploration = false;
      is_driving = false;
      acquire_image = false;
      finished_exploration = false;
      known_object_pose = false;
      force_frontier_update = false;
      
      min_nav_target_distance = 0.04f;
    }
};

extern SharedClass *bt_data_;

#endif
