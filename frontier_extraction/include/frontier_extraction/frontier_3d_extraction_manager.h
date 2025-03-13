#ifndef __3D_FRONTIER_EXTRACTION_H__
#define __3D_FRONTIER_EXTRACTION_H__

#include "rclcpp/rclcpp.hpp"

#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include "octomap_msgs/msg/octomap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "frontier_extraction_msgs/msg/frontier.hpp"
#include "frontier_extraction_srvs/srv/get_frontiers.hpp"


namespace frontier_extraction{

    typedef octomap::point3d point3d;
    
    class Frontier3DExtractionManager : public rclcpp::Node {

    public:  
        Frontier3DExtractionManager();
        ~Frontier3DExtractionManager();

    private:    
        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
        rclcpp::Service<frontier_extraction_srvs::srv::GetFrontiers>::SharedPtr get_frontiers_srv_;

        point3d pelvis_pos_map_;

        octomap_msgs::msg::Octomap::SharedPtr octomap_msg_;
        octomap::OcTree *octree_;

        visualization_msgs::msg::MarkerArray marker_array_;

        //0: map_open, 1 map_close, 2: frontier open
        std::vector<std::pair<point3d, int>> frontier_points_;  //point, cluster ID
        std::vector<std::pair<point3d, int>> frontier_clusters_; //centroid, n° points
       

        octomap::OcTreeNode *n_cur_frontier_;
        bool frontier_true_;         // whether or not a frontier point
        bool belong_old_;            //whether or not belong to old group
        double distance_;
        int num_occupied_;

        int min_frontier_points_;

        // ----- Private Methods ----
        void initNode();

        void extractFrontiers();

        void clearMarkers();
        void printMarkers();

        bool isFrontierXY(const point3d &p);
        
        void getFrontiersSrv(const std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Request> request,
                             std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response>      response);

        void setFrontierResponse(std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response>      response);

    };
}
#endif
