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

        //TODO: get from parameter
        min_distance_from_obj_ = 1.10;
        max_distance_from_obj_ = 1.50;

        distance_resolution_ = 0.10;

        publish_markers_ = true;
    }

    void InspectionGoalsSrvManager::publishMarkers(std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Response> response)
    {
        marker_array_.markers.clear();
        for(auto p : response->poses)
        {
            //Add Object
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "inspection_nav_goals";
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10;

            marker.color.a = 1.0;
            marker.color.r = 0.8;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
            
            marker.pose.position.x = p.position.x;
            marker.pose.position.y = p.position.y;
            marker.pose.position.z = 0.05;

            marker.pose.orientation.w = 1.0f;
            marker.id = marker_array_.markers.size();

            marker_array_.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array_);

    }

    void InspectionGoalsSrvManager::getInspectionPointsSrv(const std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Request>  request,
                                                                 std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Response> response)
    {
        if(occupancy_ == nullptr)
            return ;
        
        //Look for N valid points
        findInspectionPoints(request, response);

        //TODO: Order the targets to minimize execution time
        if(publish_markers_)
            publishMarkers(response);
    }

    void InspectionGoalsSrvManager::findInspectionPoints(const std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Request>  request,
                                                               std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Response> response)
    {
        ang_resolution_ = 0.5;
        rotation_to_check_ = 6.28;
        temp_ang_ = 0.0;

        //Check for goals_number points --> Consider the angle
        for(int i = 0; i < request->goals_number; i++){            
            //Update resolution based on missing area to check and missing targets to find
            ang_resolution_ = rotation_to_check_/static_cast<double>(2+request->goals_number-i);

            if(ang_resolution_ <= 0.10)
                break;
            
            //For the considered angle, find a valid candidate target pose
            if(!validGridInRangeDistance(request->object_pos, temp_ang_)){
                i--;
                //If NOT successfull --> scale a portion of ang_resolution_
                rotation_to_check_ -= ang_resolution_*0.05;
                temp_ang_ += ang_resolution_*0.05;
                continue;
            }
           
            //+3.14 to face the target (object)
            temp_pose_.orientation.z = sin(0.5*(temp_ang_+3.14));
            temp_pose_.orientation.w = cos(0.5*(temp_ang_+3.14));

            //If successfull --> scale ang_resolution_
            rotation_to_check_ -= ang_resolution_;
            temp_ang_ += ang_resolution_;
            
            response->poses.push_back(temp_pose_);
        }
    }

    bool InspectionGoalsSrvManager::isGridCellFree(const int id, const int th) const
    {
        return id >= 0 && id < occupancy_->info.width*occupancy_->info.height && 
               static_cast<int>(occupancy_->data[id]) <= th &&
               static_cast<int>(occupancy_->data[id]) >= 0;
    }

    bool InspectionGoalsSrvManager::validGridInRangeDistance(const geometry_msgs::msg::Point& obj, 
                                                             const double angle)
    {
        temp_sin_ = sin(angle);
        temp_cos_ = cos(angle);
        
        temp_distance_ = min_distance_from_obj_;

        int robot_pos_;
        int obj_pos_ = static_cast<int>((obj.x - occupancy_->info.origin.position.x)/occupancy_->info.resolution) +
                       static_cast<int>((obj.y - occupancy_->info.origin.position.y)/occupancy_->info.resolution)*occupancy_->info.width;

        //Considering the current angle (wrt to object) look for valid objects in min-max distance from obj
        while(temp_distance_ <= max_distance_from_obj_){        
            temp_pose_.position.x = obj.x + temp_distance_*temp_cos_;
            temp_pose_.position.y = obj.y + temp_distance_*temp_sin_;
            
            robot_pos_ = static_cast<int>((temp_pose_.position.x - occupancy_->info.origin.position.x)/occupancy_->info.resolution) +
                         static_cast<int>((temp_pose_.position.y - occupancy_->info.origin.position.y)/occupancy_->info.resolution)*occupancy_->info.width;

            //If not completely free, continue!
            if(!isGridCellFree(robot_pos_, 0)){
                temp_distance_ += distance_resolution_;
                continue;   
            }
            
            //If in LoS --> return True
            //Else return false because further target poses on the same line cannot have Los true
            return isObjInLos(obj_pos_, robot_pos_);
        }

        return false;
    }

    bool InspectionGoalsSrvManager::isObjInLos(const int obj_pos, const int robot_pos)
    {
        //Check if there are obstacles in the trajectory
        to_move_x_ = obj_pos%occupancy_->info.width - robot_pos%occupancy_->info.width;
        to_move_y_ = obj_pos/occupancy_->info.width - robot_pos/occupancy_->info.width;

        if(abs(to_move_x_) > abs(to_move_y_)){
            increment_x_ = static_cast<double>(to_move_x_)/fabs(static_cast<double>(to_move_x_));
            increment_y_ = static_cast<double>(to_move_y_)/fabs(static_cast<double>(to_move_x_));
        }
        else{
            increment_x_ = static_cast<double>(to_move_x_)/fabs(static_cast<double>(to_move_y_));
            increment_y_ = static_cast<double>(to_move_y_)/fabs(static_cast<double>(to_move_y_));
        }

        //To avoid every time abs
        to_move_x_ = abs(to_move_x_) - static_cast<int>(0.75/occupancy_->info.resolution);
        to_move_y_ = abs(to_move_y_) - static_cast<int>(0.75/occupancy_->info.resolution);

        if(to_move_x_ < 0)
            to_move_x_ = 0;
        if(to_move_y_ < 0)
            to_move_y_ = 0;

        to_move_x_doub_ = 0.0;
        to_move_y_doub_ = 0.0;

        //Check trajectory
        while(fabs(to_move_x_doub_) < to_move_x_ && fabs(to_move_y_doub_) < to_move_y_){
            to_move_x_doub_ += increment_x_;
            to_move_y_doub_ += increment_y_;

            temp_cell_ = robot_pos + static_cast<int>(to_move_y_doub_)*occupancy_->info.width + 
                                      static_cast<int>(to_move_x_doub_);
            
            if(!isGridCellFree(temp_cell_))
                return false;
        }

        return true;
    }

    InspectionGoalsSrvManager::~InspectionGoalsSrvManager(){
    }
}