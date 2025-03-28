#include <exploration_manager/collect_object_pose.h>

CollectObjectPose::CollectObjectPose(const std::string& name,
                                     const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    //Service Client
    get_objects_info_srv_ = node_->create_client<object_detection_srvs::srv::GetObjectsInfo>("/get_objects_info");

    get_objects_req_ = std::make_shared<object_detection_srvs::srv::GetObjectsInfo::Request>();
    get_objects_res_ = nullptr;
    
    service_available_ = get_objects_info_srv_->wait_for_service(20s);

    RCLCPP_INFO(node_->get_logger(), "CollectObjectPose: Service is %sAVILABLE!", ((service_available_)?"":"UN"));    
}

BT::NodeStatus CollectObjectPose::tick(){
    RCLCPP_INFO(node_->get_logger(), "CollectObjectPose");
    
    //Get objects's pose
    if(bt_data_->current_task < static_cast<int>(bt_data_->tasks.size()))
        get_objects_req_->object_class = bt_data_->tasks[bt_data_->current_task].object_name;
    else
        return BT::NodeStatus::FAILURE;

    if(service_available_){
        get_objects_fut_ = get_objects_info_srv_->async_send_request(get_objects_req_);
        get_objects_res_ = get_objects_fut_.get(); // Blocking call
    }

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
    
    return BT::NodeStatus::SUCCESS;
}
