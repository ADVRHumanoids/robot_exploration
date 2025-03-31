#ifndef __INSPECTION_GOALS_SRV__
#define __INSPECTION_GOALS_SRV___

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "define_inspection_goals_srvs/srv/get_inspection_goals.hpp"

namespace inspection_goals{
    
    class InspectionGoalsSrvManager : public rclcpp::Node {

    public:  
        InspectionGoalsSrvManager();
        ~InspectionGoalsSrvManager();
        
    private:    
        //MarkerArray Publisher        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        visualization_msgs::msg::MarkerArray marker_array_;

        //Subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_;

        rclcpp::Service<define_inspection_goals_srvs::srv::GetInspectionGoals>::SharedPtr get_targets_srv_;

        //
        geometry_msgs::msg::Pose temp_pose_;
        int obj_pos_, robot_pos_, to_move_x_, to_move_y_, temp_cell_;
        double increment_x_, increment_y_;
        double to_move_x_doub_, to_move_y_doub_;


        // ----- Private Methods ----
        void initNode();

        void getInspectionPointsSrv(const std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Request>  request,
                                          std::shared_ptr<define_inspection_goals_srvs::srv::GetInspectionGoals::Response> response);

        //Check for object in line of sight (no objects between obj and robot)
        bool isObjInLos(const geometry_msgs::msg::Point& obj,
                        const geometry_msgs::msg::Point cand_robot);

        bool isGridCellFree(const int id, const int th=98) const;
    
    };
}
#endif
