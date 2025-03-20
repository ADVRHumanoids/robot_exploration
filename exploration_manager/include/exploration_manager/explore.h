#ifndef __EXPLORE__
#define __EXPLORE__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <exploration_manager/SharedClass.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace BT;

class Explore : public BT::SyncActionNode
{
  public:
    Explore(const std::string& name,
                      const BT::NodeConfig &config,
                      rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return {};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    
  private:
    rclcpp::Node::SharedPtr node_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    float cost_, cost_min_, squared_euclidean_distance_, squared_distance_to_prev_target_;
    int max_frontier_idx_;
    float min_dist_frontier_robot_, temp_distance_;
    float cost_n_points_, cost_euclidean_distance_, cost_distance_prev_target_;
    float cost_neighbors_, cost_rotation_distance_;
    float robot_yaw_, temp_ang_diff_, temp_ang_, angle_;
    float min_distance_robot_frontier_, close_frontiers_distance_;

    std::vector<uint8_t> close_frontiers_;
    
};

#endif
