#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class SimOdomGazebo : public rclcpp::Node
{
public:
  SimOdomGazebo()
  : Node("sim_gz_odom_connect")
  {
    //Define callback for subscription
    auto odom_callback =
      [this](nav_msgs::msg::Odometry::SharedPtr msg) -> void {        
        // Publish Transform "odom" -> "pelvis"

        t_.header.stamp = msg->header.stamp;
        t_.header.frame_id = "odom";
        t_.child_frame_id = "pelvis";

        t_.transform.translation.x = msg->pose.pose.position.x;
        t_.transform.translation.y = msg->pose.pose.position.y;
        t_.transform.translation.z = msg->pose.pose.position.z;

        t_.transform.rotation.x = msg->pose.pose.orientation.x;
        t_.transform.rotation.y = msg->pose.pose.orientation.y;
        t_.transform.rotation.z = msg->pose.pose.orientation.z;
        t_.transform.rotation.w = msg->pose.pose.orientation.w;

        tf_static_broadcaster_->sendTransform(t_);
      };
    
    // Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/centauro/odom", 10, odom_callback);

    // TF broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

geometry_msgs::msg::TransformStamped t_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimOdomGazebo>());
  rclcpp::shutdown();
  return 0;
}