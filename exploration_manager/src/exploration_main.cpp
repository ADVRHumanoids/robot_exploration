
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include <exploration_manager/check_exploration_request.h>
#include <exploration_manager/collect_object_pose.h>
#include <exploration_manager/collect_frontiers.h>
#include <exploration_manager/explore.h>
#include <exploration_manager/check_locomotion_status.h>
#include <exploration_manager/send_nav_pose.h>

#include <exploration_manager/is_exploration_required.h>

#include <exploration_manager/SharedClass.h>

#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

SharedClass *bt_data_ = new SharedClass();


using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("exploration_main");

    //Get Parameters
    node->declare_parameter("bt_file", "");
    std::string bt_file = node->get_parameter("bt_file").as_string();

    node->declare_parameter("robot_exploration.world_frame", "map");
    bt_data_->world_frame = node->get_parameter("robot_exploration.world_frame").as_string();
    node->declare_parameter("robot_exploration.base_frame", "pelvis");
    bt_data_->base_frame = node->get_parameter("robot_exploration.base_frame").as_string();

    //Tf buffer and listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Init robot pose in world frame
     try {
        // std::cout << "Look for " << bt_data_->object_name << std::endl;
        bt_data_->now = node->get_clock()->now();
        
        bt_data_->last_robot_pose = tf_buffer_->lookupTransform(
                                            bt_data_->base_frame, bt_data_->world_frame,
                                            bt_data_->now);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "Could not transform!");
    }

    BT::BehaviorTreeFactory bt_factory;

    //Spin ROS node on a separate thread to allow also tick of the tree while managing ros services/actions 
    std::thread spin_thread = std::thread([node]() {
        rclcpp::spin(node->get_node_base_interface()); 
    });

    //Register BT Nodes
    bt_factory.registerSimpleCondition("IsExplorationRequired", std::bind(IsExplorationRequired));

    bt_factory.registerNodeType<CheckExplorationRequest>("CheckExplorationRequest", node);
    bt_factory.registerNodeType<CollectObjectPose>("CollectObjectPose", node);
    bt_factory.registerNodeType<CollectFrontiers>("CollectFrontiers", node);
    bt_factory.registerNodeType<Explore>("Explore", node);
    bt_factory.registerNodeType<CheckLocomotionStatus>("CheckLocomotionStatus", node);
    bt_factory.registerNodeType<SendNavPose>("SendNavPose", node);

    BT::Tree tree = bt_factory.createTreeFromFile(bt_file);

    //Tick tree
    tree.tickWhileRunning(); 

    while(1);

    rclcpp::shutdown();
    return 0;
};
   
    