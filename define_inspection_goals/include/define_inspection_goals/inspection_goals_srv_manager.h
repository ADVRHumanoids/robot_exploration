#ifndef __INSPECTION_GOALS_SRV__
#define __INSPECTION_GOALS_SRV___

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_msgs/srv/get_costmap.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace inspection_goals{
    
    class InspectionGoalsSrvManager : public rclcpp::Node {

    public:  
        InspectionGoalsSrvManager();
        ~InspectionGoalsSrvManager();
        
    private:    
        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        visualization_msgs::msg::MarkerArray marker_array_;

        rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_srv_;
        nav2_msgs::srv::GetCostmap::Request::SharedPtr get_costmap_req_;
        rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture get_costmap_fut_;
        nav2_msgs::srv::GetCostmap::Response::SharedPtr get_costmap_res_;

        nav2_msgs::msg::Costmap costmap_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_targets_srv_;

        // ----- Private Methods ----
        void initNode();

        void getInspectionPointsSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    };
}
#endif
