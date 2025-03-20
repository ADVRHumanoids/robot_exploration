#include <exploration_manager/collect_object_pose.h>

CollectObjectPose::CollectObjectPose(const std::string& name,
                                     const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    node_->declare_parameter("robot_exploration.distance_target_object", 0.85);
    distance_target_object_ = node_->get_parameter("robot_exploration.distance_target_object").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus CollectObjectPose::tick(){
    // RCLCPP_INFO(node_->get_logger(), "CollectObjectPose");
    
    //Get objects's pose
    try {
        // std::cout << "Look for " << bt_data_->object_name << std::endl;
        bt_data_->now = node_->get_clock()->now();
        
        bt_data_->object_pose = tf_buffer_->lookupTransform(
            bt_data_->world_frame, bt_data_->object_name,
            bt_data_->now);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_WARN(node_->get_logger(), "CollectObjectPose: Object TF not found!");
        bt_data_->need_exploration = true;
        bt_data_->known_object_pose = false;
        return BT::NodeStatus::FAILURE;
    }

    // Select nav target in the line, at a distance of 0.85m from object
    angle_ = atan2(bt_data_->object_pose.transform.translation.y - bt_data_->last_robot_pose.transform.translation.y,
                   bt_data_->object_pose.transform.translation.x - bt_data_->last_robot_pose.transform.translation.x);
    
    bt_data_->locomotion_target.position.x = bt_data_->object_pose.transform.translation.x - distance_target_object_*cos(angle_);
    bt_data_->locomotion_target.position.y = bt_data_->object_pose.transform.translation.y - distance_target_object_*sin(angle_);

    //TODO: Improve
    bt_data_->locomotion_target.orientation.x = 0;
    bt_data_->locomotion_target.orientation.y = 0;
    bt_data_->locomotion_target.orientation.z = 0;
    bt_data_->locomotion_target.orientation.w = 1;
    
    return BT::NodeStatus::SUCCESS;
}
