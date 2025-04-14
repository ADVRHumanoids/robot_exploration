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

#include "rclcpp/rclcpp.hpp"

/*
  Task ID:  
      - 1 Find/Reach Object
      - 2 Inspect Object
*/

#define INSPECTION_IMAGES 5

using namespace std::chrono_literals;

class RosClientsStatus{
  public:

    RosClientsStatus():
      save_img_srv(false), send_cand_target_srv(false), cancel_nav_srv(false), nav_to_pose_srv(false),
      get_frontiers_srv(false), get_objects_srv(false), get_insp_goals_srv(false)
    {}

    bool save_img_srv, send_cand_target_srv, cancel_nav_srv, nav_to_pose_srv;
    bool get_frontiers_srv, get_objects_srv, get_insp_goals_srv;
};

class Task{
  public:

    Task(unsigned int task_id = 0, std::string object_name="obj"):
      id(task_id), object_name(object_name), nav_targets({}),
      inspection_steps(0), images_collected(0)
    {}

    geometry_msgs::msg::Pose getLastNavTarget(){
      if(inspection_steps < 0 || inspection_steps >= static_cast<int>(nav_targets.size()))
         return geometry_msgs::msg::Pose();

      return nav_targets[inspection_steps];
    }

    void addNavTarget(geometry_msgs::msg::Pose nav_target){
      nav_targets.push_back(nav_target);
    }

    int getNavTargetsNumber(){
      return static_cast<int>(nav_targets.size());
    }

    void clearNavTargets(){
      nav_targets.clear();
      inspection_steps = 0;
    }    
    
    unsigned int id;
    std::string object_name;
    int inspection_steps, images_collected;

    private:
      std::vector<geometry_msgs::msg::Pose> nav_targets;
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

    RosClientsStatus ros_status;
    
    int current_task;

    //Constructor
    SharedClass(){
      tasks.reserve(2);
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
