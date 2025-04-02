#include <exploration_manager/define_inspection_goals.h>

DefineInspectionGoals::DefineInspectionGoals(const std::string& name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{   

    //Service Client
    get_insp_goals_srv_ = node_->create_client<define_inspection_goals_srvs::srv::GetInspectionGoals>("/get_inspection_goals");

    get_insp_goals_req_ = std::make_shared<define_inspection_goals_srvs::srv::GetInspectionGoals::Request>();
    get_insp_goals_res_ = nullptr;
    
    service_available_ = get_insp_goals_srv_->wait_for_service(20s);

    //TODO: From params
    min_distance_to_object_ = 0.8;
    max_distance_to_object_ = 1.5;

    get_insp_goals_req_->goals_number = INSPECTION_IMAGES;
}

BT::NodeStatus DefineInspectionGoals::tick(){
    // RCLCPP_INFO(node_->get_logger(), "DefineInspectionGoals");
            
    if(bt_data_->tasks[bt_data_->current_task].getNavTargetsNumber() == 0){
        bt_data_->tasks[bt_data_->current_task].clearNavTargets();

        if(service_available_){
            get_insp_goals_req_->object_pos.x = bt_data_->object_pose.transform.translation.x;
            get_insp_goals_req_->object_pos.y = bt_data_->object_pose.transform.translation.y;
            get_insp_goals_req_->object_pos.z = bt_data_->object_pose.transform.translation.z;

            get_insp_goals_fut_ = get_insp_goals_srv_->async_send_request(get_insp_goals_req_).share();
            get_insp_goals_res_ = get_insp_goals_fut_.get(); // Blocking call

            if (!get_insp_goals_res_)
                return BT::NodeStatus::FAILURE;

            for(auto p : get_insp_goals_res_->poses)
                bt_data_->tasks[bt_data_->current_task].addNavTarget(p);
        }
    }
    
    return BT::NodeStatus::SUCCESS;
}