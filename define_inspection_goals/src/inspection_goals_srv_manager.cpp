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
        get_targets_srv_ = this->create_service<define_inspection_goals_srvs::srv::GetInspectionGoals>("/get_inspection_goals",
                           std::bind(&InspectionGoalsSrvManager::getInspectionPointsSrv, this, std::placeholders::_1, std::placeholders::_2));
        //Publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/inspection_nav_goals",
                            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        
        // Subscriber
        auto getCostmap =
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
                occupancy_ = msg;
            };

        //Client for costmap
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 10, getCostmap);

        occupancy_ = nullptr;
    }

    void InspectionGoalsSrvManager::getInspectionPointsSrv(const std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Request>  request,
                                                                 std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Response> response)
    {
        if(occupancy_ == nullptr)
            return ;

        double distance_from_obj = 1.2;
    
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
        
        marker.pose.position.x = request->object_pos.x;
        marker.pose.position.y = request->object_pos.y;
        marker.pose.position.z = 0.05;

        marker.pose.orientation.w = 1.0f;
        marker.id = marker_array_.markers.size();

        marker_array_.markers.push_back(marker);

        double ang_resolution = 6.28/static_cast<double>(request->goals_number);
        double temp_ang_ = 0.0;

        for(int i = 0; i < request->goals_number; i++){
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
            
            temp_ang_ = static_cast<double>(i)*ang_resolution;
            marker2.pose.position.x = request->object_pos.x + distance_from_obj*cos(temp_ang_);
            marker2.pose.position.y = request->object_pos.y + distance_from_obj*sin(temp_ang_);
            marker2.pose.position.z = 0.05;

            marker2.pose.orientation.w = 1.0f;
            marker2.id = marker_array_.markers.size();
            
            //Define Pose
            temp_pose_.position = marker2.pose.position;
            if(!isObjInLos(request->object_pos, temp_pose_.position))
                continue;
            
            if(temp_ang_ > 0)
                temp_ang_ -= 3.14;
            else
                temp_ang_ += 3.14;

            temp_pose_.orientation.z = sin(0.5*temp_ang_);
            temp_pose_.orientation.w = cos(0.5*temp_ang_);

            response->poses.push_back(temp_pose_);
            
            marker_array_.markers.push_back(marker2);
        }

        marker_pub_->publish(marker_array_);
    }

    bool InspectionGoalsSrvManager::isGridCellFree(const int id, const int th) const
    {
        return id >= 0 && id < occupancy_->info.width*occupancy_->info.height && 
               occupancy_->data[id] <= th;
    }

    bool InspectionGoalsSrvManager::isObjInLos(const geometry_msgs::msg::Point& obj,
                                               const geometry_msgs::msg::Point cand_robot)
    {
        //Check if there are obstacles in the trajectory
        obj_pos_ = static_cast<int>((obj.x - occupancy_->info.origin.position.x)/occupancy_->info.resolution) +
                   static_cast<int>((obj.y - occupancy_->info.origin.position.y)/occupancy_->info.resolution)*occupancy_->info.width;

        robot_pos_ = static_cast<int>((cand_robot.x - occupancy_->info.origin.position.x)/occupancy_->info.resolution) +
                     static_cast<int>((cand_robot.y - occupancy_->info.origin.position.y)/occupancy_->info.resolution)*occupancy_->info.width;
        
        if(!isGridCellFree(robot_pos_, 80))
            return false;

        to_move_x_ = obj_pos_%occupancy_->info.width - robot_pos_%occupancy_->info.width;
        to_move_y_ = obj_pos_/occupancy_->info.width - robot_pos_/occupancy_->info.width;

        if(abs(to_move_x_) > abs(to_move_y_)){
            increment_x_ = static_cast<double>(to_move_x_)/fabs(static_cast<double>(to_move_x_));
            increment_y_ = static_cast<double>(to_move_y_)/fabs(static_cast<double>(to_move_x_));
        }
        else{
            increment_x_ = static_cast<double>(to_move_x_)/fabs(static_cast<double>(to_move_y_));
            increment_y_ = static_cast<double>(to_move_y_)/fabs(static_cast<double>(to_move_y_));
        }

        //To avoid every time abs
        to_move_x_ = abs(to_move_x_) - static_cast<int>(0.6/occupancy_->info.resolution);
        to_move_y_ = abs(to_move_y_) - static_cast<int>(0.6/occupancy_->info.resolution);

        to_move_x_doub_ = 0.0;
        to_move_y_doub_ = 0.0;

        //Check trajectory
        while(fabs(to_move_x_doub_) < to_move_x_ && fabs(to_move_y_doub_) < to_move_y_){
            to_move_x_doub_ += increment_x_;
            to_move_y_doub_ += increment_y_;

            temp_cell_ = robot_pos_ + static_cast<int>(to_move_y_doub_)*occupancy_->info.width + 
                                      static_cast<int>(to_move_x_doub_);
            
            if(!isGridCellFree(temp_cell_))
                return false;
        }

        return true;
    }

    InspectionGoalsSrvManager::~InspectionGoalsSrvManager(){
    }
}