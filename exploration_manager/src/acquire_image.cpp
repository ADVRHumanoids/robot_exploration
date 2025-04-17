#include <exploration_manager/acquire_image.h>

AcquireImage::AcquireImage(const std::string& name,
                           const BT::NodeConfig &config,
                           rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   
    //Service Client
    acquire_image_srv_ = node_->create_client<std_srvs::srv::Trigger>("/save_image");
    acquire_image_req_ = std::make_shared<std_srvs::srv::Trigger::Request>();

    bt_data_->ros_status.save_img_srv = acquire_image_srv_->wait_for_service(10s);    
}

BT::NodeStatus AcquireImage::tick(){
    RCLCPP_INFO(node_->get_logger(), "AcquireImage");
    
    if(bt_data_->ros_status.save_img_srv){
        acquire_image_fut_ = acquire_image_srv_->async_send_request(acquire_image_req_).share();
        acquire_image_res_ = acquire_image_fut_.get(); // Blocking call
    }

    bt_data_->acquire_image = false;

    bt_data_->tasks[bt_data_->current_task].inspection_steps ++;

    //NOTE: For now only the valid poses are considered --> so tasks.size() generally < INSPECTION_IMAGES
    // if(bt_data_->tasks[bt_data_->current_task].inspection_steps >= INSPECTION_IMAGES){
    if(bt_data_->tasks[bt_data_->current_task].inspection_steps >= 
    bt_data_->tasks[bt_data_->current_task].getNavTargetsNumber()){
        RCLCPP_WARN(node_->get_logger(), "AcquireImage: Finished acquisiton!!");

        bt_data_->current_task ++;
        if(bt_data_->current_task  >= static_cast<int>(bt_data_->tasks.size())){
            RCLCPP_WARN(node_->get_logger(), "Finished all tasks!!");
            bt_data_->active_task = false;
        }
    }

    if(bt_data_->ros_status.save_img_srv && acquire_image_res_ != nullptr){
        bt_data_->tasks[bt_data_->current_task].images_collected ++;
        return BT::NodeStatus::SUCCESS;
    }
    else{
        //Try again
        bt_data_->ros_status.save_img_srv = acquire_image_srv_->wait_for_service(1s);
        RCLCPP_WARN(node_->get_logger(), "AcquireImage: Server Not Available!! Go on anyway...");
    }
    
    return BT::NodeStatus::FAILURE;
}