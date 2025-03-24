#include <exploration_manager/collect_object_pose.h>

CollectObjectPose::CollectObjectPose(const std::string& name,
                                     const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    node_->declare_parameter("robot_exploration.distance_to_object_pose", 0.85);
    distance_to_object_pose_ = node_->get_parameter("robot_exploration.distance_to_object_pose").as_double();

    //Service Client
    get_objects_info_srv_ = node_->create_client<object_detection_srvs::srv::GetObjectsInfo>("/get_objects_info");

    get_objects_req_ = std::make_shared<object_detection_srvs::srv::GetObjectsInfo::Request>();
}

BT::NodeStatus CollectObjectPose::tick(){
    // RCLCPP_INFO(node_->get_logger(), "CollectObjectPose");
    
    //Get objects's pose
    get_objects_req_->object_class = bt_data_->object_name;

    get_objects_fut_ = get_objects_info_srv_->async_send_request(get_objects_req_);
    get_objects_res_ = get_objects_fut_.get(); // Blocking call

    if (get_objects_res_ != nullptr && get_objects_res_->objects_data.size() > 0){

        //NOTE: ATM take the first one
        bt_data_->object_pose.transform.translation.x = get_objects_res_->objects_data[0].centroid.x;
        bt_data_->object_pose.transform.translation.y = get_objects_res_->objects_data[0].centroid.y;
        bt_data_->object_pose.transform.translation.z = get_objects_res_->objects_data[0].centroid.z;

        bt_data_->need_exploration = false;
        bt_data_->known_object_pose = true;
    }
    else{
        RCLCPP_WARN(node_->get_logger(), "CollectObjectPose: No object found!");
        bt_data_->need_exploration = true;
        bt_data_->known_object_pose = false;
        return BT::NodeStatus::FAILURE;
    }

    // Select nav target in the line, at X distance from object
    angle_ = atan2(bt_data_->object_pose.transform.translation.y - bt_data_->last_robot_pose.transform.translation.y,
                   bt_data_->object_pose.transform.translation.x - bt_data_->last_robot_pose.transform.translation.x);
    
    // Transform into [-3.14; 3.14]
    if(angle_ > 3.14)
        angle_ = angle_ - 6.28*(1.0 + std::floor(angle_/6.28));
    else if(angle_ < -3.14)
        angle_ = angle_ + 6.28*(1.0 + std::floor(-angle_/6.28));
    
    bt_data_->locomotion_target.position.x = bt_data_->object_pose.transform.translation.x - distance_to_object_pose_*cos(angle_);
    bt_data_->locomotion_target.position.y = bt_data_->object_pose.transform.translation.y - distance_to_object_pose_*sin(angle_);

    //TODO: Improve
    //Define orientation to face the object
    bt_data_->locomotion_target.orientation.x = 0;
    bt_data_->locomotion_target.orientation.y = 0;
    bt_data_->locomotion_target.orientation.z = sin(angle_/2.0);
    bt_data_->locomotion_target.orientation.w = cos(angle_/2.0);
    
    return BT::NodeStatus::SUCCESS;
}
