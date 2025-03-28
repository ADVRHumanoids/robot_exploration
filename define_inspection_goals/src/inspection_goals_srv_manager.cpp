#include <define_inspection_goals/inspection_goals_srv_manager.h>

namespace inspection_goals{

    InspectionGoalsSrvManager::InspectionGoalsSrvManager()
    : Node("inspection_goals_srv_node")
    {
        initNode();
    }
    void InspectionGoalsSrvManager::initNode()
    {
        //Service Server
        get_targets_srv_ = this->create_service<std_srvs::srv::Trigger>("/get_inspection_goals",
                           std::bind(&InspectionGoalsSrvManager::getInspectionPointsSrv, this, std::placeholders::_1, std::placeholders::_2));
        //Publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/inspection_nav_goals",
                            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        
        //Client for costmap
        get_costmap_srv_ = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
        get_costmap_req_ = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        get_costmap_req_->specs.layer = "map";
    }
    
    void InspectionGoalsSrvManager::getInspectionPointsSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Collect Costmap");
        
        get_costmap_fut_ = get_costmap_srv_->async_send_request(get_costmap_req_);
        get_costmap_res_ = get_costmap_fut_.get(); // Blocking call
        RCLCPP_INFO(this->get_logger(), "Costmap Collected!");

        //TODO: Take from "request" 
        geometry_msgs::msg::Point object_pos;
        object_pos.x = 3.9;
        object_pos.y = 1.15;

        double distance_from_obj = 1.0;
        
        if(get_costmap_res_)
        {
            costmap_ = get_costmap_res_->map;

            marker_array_.markers.clear();

            //Add Object
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "inspection_nav_goals";
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15;

            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            marker.color.g = 0.0;
            
            marker.pose.position.x = object_pos.x;
            marker.pose.position.y = object_pos.y;
            marker.pose.position.z = 0.05;

            marker.pose.orientation.w = 1.0f;
            marker.id = marker_array_.markers.size();

            marker_array_.markers.push_back(marker);

            double ang_resolution = 360.0/30.0;

            for(int i = 0; i < 30; i++){
                visualization_msgs::msg::Marker marker2;

                marker2.header.frame_id = "map";
                marker2.ns = "inspection_nav_goals";
                marker2.type = visualization_msgs::msg::Marker::SPHERE;
                marker2.action = visualization_msgs::msg::Marker::ADD;
                marker2.scale.x = marker2.scale.y = marker2.scale.z = 0.10;

                marker2.color.a = 1.0;
                marker2.color.r = 1.0;
                marker2.color.b = 0.0;
                marker2.color.g = 0.0;
                
                marker2.pose.position.x = object_pos.x + distance_from_obj*cos(static_cast<double>(i)*ang_resolution);
                marker2.pose.position.y = object_pos.y + distance_from_obj*sin(static_cast<double>(i)*ang_resolution);
                marker2.pose.position.z = 0.05;

                marker2.pose.orientation.w = 1.0f;
                marker2.id = marker_array_.markers.size();

                marker_array_.markers.push_back(marker2);
            }
        }
    }

    InspectionGoalsSrvManager::~InspectionGoalsSrvManager(){
    }
}