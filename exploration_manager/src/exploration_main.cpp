
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
#include <exploration_manager/acquire_image.h>
#include <exploration_manager/define_inspection_goals.h>

#include <exploration_manager/is_request_active.h>
#include <exploration_manager/can_acquire_image.h>
#include <exploration_manager/is_object_pose_known.h>
#include <exploration_manager/is_task_inspection.h>

#include <exploration_manager/SharedClass.h>

#include "exploration_manager_msgs/msg/exploration_status.hpp"

SharedClass *bt_data_ = new SharedClass();

using namespace std::chrono_literals;

class ExploratioMain : public rclcpp::Node
{
  public:
    ExploratioMain()
    : Node("exploration_main")
    {        
        exp_status_pub_ = this->create_publisher<exploration_manager_msgs::msg::ExplorationStatus>("/exploration_status", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ExploratioMain::main_loop, this));
    }

  private:
    void main_loop()
    {
        exp_status_msg_.active_task = bt_data_->active_task;
        exp_status_msg_.location_known = bt_data_->known_object_pose;

        if(bt_data_->current_task >= 0 && bt_data_->current_task < static_cast<int>(bt_data_->tasks.size())){
            exp_status_msg_.task_id = bt_data_->tasks[bt_data_->current_task].id;
            exp_status_msg_.target_object = bt_data_->tasks[bt_data_->current_task].object_name;
            exp_status_msg_.images_collected = bt_data_->tasks[bt_data_->current_task].images_collected;
        }
        else{
            exp_status_msg_.task_id = 0;
            exp_status_msg_.target_object = "-";
            exp_status_msg_.images_collected = 0;
        }

        if(bt_data_->known_object_pose){
            exp_status_msg_.object_target_pos.x = bt_data_->object_pose.transform.translation.x;
            exp_status_msg_.object_target_pos.y = bt_data_->object_pose.transform.translation.y;
            exp_status_msg_.object_target_pos.z = bt_data_->object_pose.transform.translation.z;
        }
        else{
            exp_status_msg_.object_target_pos.x = 0.0;
            exp_status_msg_.object_target_pos.y = 0.0;
            exp_status_msg_.object_target_pos.z = 0.0;
        }
        exp_status_msg_.is_driving = bt_data_->is_driving;

        if(bt_data_->is_driving)
            exp_status_msg_.nav_target_pose = bt_data_->locomotion_target;

        exp_status_msg_.robot_pos.x = bt_data_->last_robot_pose.transform.translation.x;
        exp_status_msg_.robot_pos.y = bt_data_->last_robot_pose.transform.translation.y;
        exp_status_msg_.robot_pos.z = bt_data_->last_robot_pose.transform.translation.z;

        exp_status_msg_.frontiers_number = bt_data_->frontiers.size();
        exp_status_msg_.finished = !bt_data_->active_task;

        //Set ROS2 Client Status
        exp_status_msg_.save_img_srv = bt_data_->ros_status.save_img_srv;
        exp_status_msg_.cancel_nav_srv = bt_data_->ros_status.cancel_nav_srv;
        exp_status_msg_.get_frontiers_srv = bt_data_->ros_status.get_frontiers_srv;
        exp_status_msg_.nav_to_pose_srv = bt_data_->ros_status.nav_to_pose_srv;
        exp_status_msg_.get_objects_srv = bt_data_->ros_status.get_objects_srv;
        exp_status_msg_.get_insp_goals_srv = bt_data_->ros_status.get_insp_goals_srv;
        exp_status_msg_.send_cand_target_srv = bt_data_->ros_status.send_cand_target_srv;

        exp_status_pub_->publish(exp_status_msg_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<exploration_manager_msgs::msg::ExplorationStatus>::SharedPtr exp_status_pub_;
    exploration_manager_msgs::msg::ExplorationStatus exp_status_msg_;
};

// Function to continuously tick the Behavior Tree
void runBehaviorTree(Tree& tree) {
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickWhileRunning();
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Adjust tick rate if needed
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<rclcpp::Node> node = std::make_shared<ExploratioMain>();

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
        bt_data_->now = node->get_clock()->now();
        
        bt_data_->last_robot_pose = tf_buffer_->lookupTransform(
                                            bt_data_->world_frame, bt_data_->base_frame,
                                            bt_data_->now);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "Could not transform from %s to %s!", bt_data_->world_frame.c_str(),
                                                                              bt_data_->base_frame.c_str());
    }

    BT::BehaviorTreeFactory bt_factory;

    //Register BT Nodes
    bt_factory.registerSimpleCondition("IsRequestActive", std::bind(IsRequestActive));
    bt_factory.registerSimpleCondition("CanAcquireImage", std::bind(CanAcquireImage));
    bt_factory.registerSimpleCondition("IsTaskInspection", std::bind(IsTaskInspection));
    bt_factory.registerSimpleCondition("IsObjectPoseKnown", std::bind(IsObjectPoseKnown));

    bt_factory.registerNodeType<CheckExplorationRequest>("CheckExplorationRequest", node);
    bt_factory.registerNodeType<CollectObjectPose>("CollectObjectPose", node);
    bt_factory.registerNodeType<CollectFrontiers>("CollectFrontiers", node);
    bt_factory.registerNodeType<Explore>("Explore", node);
    bt_factory.registerNodeType<CheckLocomotionStatus>("CheckLocomotionStatus", node);
    bt_factory.registerNodeType<SendNavPose>("SendNavPose", node);
    bt_factory.registerNodeType<AcquireImage>("AcquireImage", node);
    bt_factory.registerNodeType<DefineInspectionGoals>("DefineInspectionGoals", node);

    BT::Tree tree = bt_factory.createTreeFromFile(bt_file);

    // Run the BT in a separate thread
    std::thread bt_thread(runBehaviorTree, std::ref(tree));

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
};
   
    