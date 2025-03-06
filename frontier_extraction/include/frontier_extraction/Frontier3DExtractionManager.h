#ifndef FRONTIER_EXTRACTION_H
#define FRONTIER_EXTRACTION_H

#include <ros/ros.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <frontier_extraction/Frontier.h>
#include <frontier_extraction/GetFrontiers.h>


namespace frontier_extraction{

    using namespace Eigen;
    typedef octomap::point3d point3d;
    
    class Frontier3DExtractionManager {

    public:  
        Frontier3DExtractionManager( std::string ns = "", double rate = 2.5 );
        ~Frontier3DExtractionManager();
        
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
        ros::Subscriber occupancy_sub_;

        point3d pelvis_pos_map_;

        octomap_msgs::Octomap::ConstPtr octomap_msg_;
        octomap::OcTree *octree_;

        visualization_msgs::MarkerArray marker_array_;

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
        void initROSNode(double rate);
            
        void initParams();

        void extractFrontiers();

        void clearMarkers();
        void printMarkers();

        bool isFrontierXY(const point3d &p);

        void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
        
        bool getFrontiersSrv(frontier_extraction::GetFrontiers::Request  &req,
                             frontier_extraction::GetFrontiers::Response &res);

        void setFrontierResponse(frontier_extraction::GetFrontiers::Response &res);
    };
}
#endif
