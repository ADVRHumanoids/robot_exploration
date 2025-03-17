#ifndef FRONTIER_EXTRACTION_H
#define FRONTIER_EXTRACTION_H

#include <ros/ros.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <frontier_extraction/Frontier.h>
#include <frontier_extraction/GetFrontiers.h>


namespace frontier_extraction{

    using namespace Eigen;
    
    class Frontier2DExtractionManager {

    public:  
        Frontier2DExtractionManager( std::string ns = "", double rate = 50.0 );
        ~Frontier2DExtractionManager();
        
        void main_loop(const ros::TimerEvent& timer);
        
        void spin();

    private:    
        
        // ROS Node --------------------------------------------------------------------------------
        ros::NodeHandle nh_; /* ROSE node handle */
        ros::Timer timer_; /* ROS timer */
        double period_; /* loop period value */
        double rate_; /* loop rate */
        double time_; /* time */

        ros::ServiceServer get_frontiers_srv_;
        ros::Publisher marker_pub_;

        Affine3d pelvis_T_map_;

        nav_msgs::OccupancyGrid::ConstPtr grid_;

        visualization_msgs::MarkerArray marker_array_;

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

        int max_cluster_, min_frontier_points_;

        int i,j;

        // ----- Private Methods ----
        void initROSNode(double rate);
            
        void initParams();

        const bool isValidCell(const int idx);
        const bool isFrontier(const int idx1, const int idx2);

        void extractFrontiers();

        void clearMarkers();
        void printMarkers();
        
        void innerExtractFrontiers(const int idx);

        bool getFrontiersSrv(frontier_extraction::GetFrontiers::Request  &req,
                             frontier_extraction::GetFrontiers::Response &res);

        void setFrontierResponse(frontier_extraction::GetFrontiers::Response &res);

        void frontierClustering();
    };
}
#endif
