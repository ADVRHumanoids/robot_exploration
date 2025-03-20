#ifndef __FRONTIER_EXTRACTION_H__
#define __FRONTIER_EXTRACTION_H__

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "frontier_extraction_msgs/msg/frontier.hpp"
#include "frontier_extraction_srvs/srv/get_frontiers.hpp"


namespace frontier_extraction{
    
    class Frontier2DExtractionManager : public rclcpp::Node {

    public:  
        Frontier2DExtractionManager();
        ~Frontier2DExtractionManager();
        
    private:    
        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
        rclcpp::Service<frontier_extraction_srvs::srv::GetFrontiers>::SharedPtr get_frontiers_srv_;

        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_;

        visualization_msgs::msg::MarkerArray marker_array_;

        int width_, height_;
        float resolution_;

        //0: map_open, 1 map_close, 2: frontier open
        std::set<int> frontier_points_;
        std::vector<int> frontier_clustersing_; //list of indices defining the number of the cluster in which the point of the set at the same distance belongs
        
        std::vector<int> frontiers_dim_; //Size of cluster number i
        std::vector<std::array<float, 2>> centroids_;

        std::set<int> visited_;
        std::set<int> to_visit_;

        int temp_pos_, temp_pos2_;

        int max_cluster_, min_frontier_points_, max_frontier_distance_;
        int marker_id_, marker_id2_, cell_distance_;
        int i,j;

        // ----- Private Methods ----
        void initNode();

        bool isValidCell(const int idx) const;
        bool isFrontier(const int idx1, const int idx2) const;

        void extractFrontiers(geometry_msgs::msg::Point robot_pose);

        void clearMarkers();
        void printMarkers();
        
        void innerExtractFrontiers(const int idx);

        void getFrontiersSrv(const std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Request> request,
                             std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response>      response);


        void setFrontierResponse(std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response>      response);

        void frontierClustering();
    };
}
#endif
