#include <frontier_extraction/Frontier3DExtractionManager.h>
#include <chrono>

namespace frontier_extraction{

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    Frontier3DExtractionManager::Frontier3DExtractionManager ( std::string ns, double rate )
    //   : _nh ( ns ),
    {
        //initialization ROS node
        
        ROS_INFO_STREAM ("I am initializing the Node...");
        initROSNode(rate);

        ROS_INFO_STREAM ("Init Params...");
        initParams();    
            
        //initialization done
        ROS_INFO_STREAM ("Initialization done.");
    }

    void Frontier3DExtractionManager::initROSNode(double rate)
    {
        // init ROS node
        rate_ = rate;
        period_ = 1.0 / rate_;
        timer_ = nh_.createTimer(ros::Duration(period_), &Frontier3DExtractionManager::main_loop, this, false, false);
        time_ = 0.0;

        get_frontiers_srv_ = nh_.advertiseService("get_frontiers", &Frontier3DExtractionManager::getFrontiersSrv, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frontier_markers", 1000);

        occupancy_sub_ = nh_.subscribe("/local/octomap_full", 10, &Frontier3DExtractionManager::octomapCallback, this);
    }

    void Frontier3DExtractionManager::initParams(){
        min_frontier_points_ = 5;

        frontier_points_ = {};
        frontier_clusters_ = {};
    }
    
    void Frontier3DExtractionManager::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg){
        octomap_msg_ = msg;

        if(octomap_msg_ != nullptr){
            octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*octomap_msg_));
        }
    }

    bool Frontier3DExtractionManager::getFrontiersSrv(frontier_extraction::GetFrontiers::Request  &req,
             frontier_extraction::GetFrontiers::Response &res)
    {
        if(octree_ == nullptr)
            return false;

        // auto t1 = high_resolution_clock::now();
        pelvis_pos_map_.x() = req.robot_pose.x;
        pelvis_pos_map_.y() = req.robot_pose.y;
        pelvis_pos_map_.z() = req.robot_pose.z;

        extractFrontiers();        
        // auto t2 = high_resolution_clock::now();

        // ROS_INFO("Frontier Points added: %ld", frontier_points_.size());
        
        // Publish frontiers and set service response
        printMarkers();
        // auto t3 = high_resolution_clock::now();

        marker_pub_.publish(marker_array_);
        
        // duration<double, std::milli> ms_double = t2 - t1;
        // ROS_INFO("Extract Frontiers: %f", ms_double.count());
        // ms_double = t3 - t2;
        // ROS_INFO("Markers: %f", ms_double.count());

        setFrontierResponse(res);

        return true;
    }

    void Frontier3DExtractionManager::setFrontierResponse(frontier_extraction::GetFrontiers::Response &res)
    {
        // Fill response message with frontiers bigger than X pixels
        for(int i = 0; i < frontier_clusters_.size(); i++){
            if(frontier_clusters_[i].second >= min_frontier_points_){
                frontier_extraction::Frontier f;

                f.centroid.x = frontier_clusters_[i].first.x();
                f.centroid.y = frontier_clusters_[i].first.y();
                f.centroid.z = frontier_clusters_[i].first.z();

                f.number_of_points = frontier_clusters_[i].second;

                res.frontiers.push_back(f);
            }
        }
    }

    bool Frontier3DExtractionManager::isFrontierXY(const point3d &p){
        //if there are unknown around the cube, the cube is frontier
        num_occupied_ = 0;
        //Check on the same plane
         for(int f_x = -1; f_x <= 1; f_x++){
             for(int f_y = -1; f_y <= 1; f_y++){
                 if(f_x == 0 && f_y == 0)
                     continue;

                 n_cur_frontier_ = octree_->search(point3d(p.x() + f_x*octree_->getResolution(),
                                                           p.y() + f_y*octree_->getResolution(),
                                                           p.z()));
                
                 if(n_cur_frontier_ != nullptr && octree_->isNodeOccupied(n_cur_frontier_)){
                     num_occupied_ ++;
                     if(num_occupied_ > 6)
                         return false;
                 }
             }
         }

        //TODO: IMPROVE
        //Check on up and down (2*resolution to be more robust)        
        n_cur_frontier_ = octree_->search(point3d(p.x(),
                                                  p.y(),
                                                  p.z() - 1.0*octree_->getResolution()));
        
        if(n_cur_frontier_ == nullptr || !octree_->isNodeOccupied(n_cur_frontier_)){
            n_cur_frontier_ = octree_->search(point3d(p.x(),
                                                      p.y(),
                                                      p.z() + 1.0*octree_->getResolution()));

            if(n_cur_frontier_ == nullptr || !octree_->isNodeOccupied(n_cur_frontier_))
                return true;
        }        
        
        return false;
    }

    void Frontier3DExtractionManager::extractFrontiers(){
        frontier_points_.clear();
        
        ROS_WARN("Extract Frontiers");
        //Get Frontier Points
        for(octomap::OcTree::leaf_iterator n = octree_->begin_leafs(octree_->getTreeDepth()); n != octree_->end_leafs(); ++n)
        {
            // unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

            if(octree_->isNodeOccupied(*n) && isFrontierXY(n.getCoordinate()))
                frontier_points_.push_back({point3d(n.getX(), n.getY(), n.getZ()), 0});
        }

        ROS_WARN("Cluster Frontier Points");
        //Cluster frontier points
        int id = 1;
        double max_distance_ = pow(2.0*octree_->getResolution(), 2);
        
        if(frontier_points_.size() > 0){
            frontier_points_[0].second = 1;
            frontier_clusters_.resize(2);
            frontier_clusters_[1].first = frontier_points_[0].first;
            frontier_clusters_[1].second = 1;
        }
        else{
            frontier_clusters_.clear();
        }

        for(int i = 1; i < frontier_points_.size(); i++){                 
            for(int j = 0; j < i; j++){       

                if(fabs(frontier_points_[i].first.z() - frontier_points_[j].first.z()) >= octree_->getResolution())
                    continue;
                
                distance_ = pow(frontier_points_[i].first.x() - frontier_points_[j].first.x(), 2) + 
                            pow(frontier_points_[i].first.y() - frontier_points_[j].first.y(), 2);
                            
                if((frontier_points_[i].first.x() - pelvis_pos_map_.x())*(frontier_points_[j].first.x() - pelvis_pos_map_.x()) > 0.0 &&
                    distance_ <= max_distance_)
                {
                    if(frontier_points_[i].second == 0)
                    {
                        frontier_points_[i].second = frontier_points_[j].second;
                        frontier_clusters_[frontier_points_[i].second].first.x() += frontier_points_[i].first.x();
                        frontier_clusters_[frontier_points_[i].second].first.y() += frontier_points_[i].first.y();
                        frontier_clusters_[frontier_points_[i].second].first.z() += frontier_points_[i].first.z();

                        frontier_clusters_[frontier_points_[i].second].second ++;
                    }
                    else if(frontier_points_[i].second != frontier_points_[j].second){
                        for(int k = 0; k < i; k++){
                            if(frontier_points_[k].second == frontier_points_[j].second){
                                if(frontier_clusters_[frontier_points_[j].second].second > 0){
                                    frontier_clusters_[frontier_points_[i].second].first.x() += frontier_clusters_[frontier_points_[j].second].first.x();
                                    frontier_clusters_[frontier_points_[i].second].first.y() += frontier_clusters_[frontier_points_[j].second].first.y();
                                    frontier_clusters_[frontier_points_[i].second].first.z() += frontier_clusters_[frontier_points_[j].second].first.z();
                                    frontier_clusters_[frontier_points_[i].second].second += frontier_clusters_[frontier_points_[j].second].second;
                                    
                                    frontier_clusters_[frontier_points_[j].second].first.x() = 0;
                                    frontier_clusters_[frontier_points_[j].second].first.y() = 0;
                                    frontier_clusters_[frontier_points_[j].second].first.z() = 0;

                                    frontier_clusters_[frontier_points_[j].second].second = 0;
                                }

                                frontier_points_[k].second = frontier_points_[i].second;
                            }                        
                        }
                    }   
                }
            }

            if(frontier_points_[i].second == 0){
                id ++;
                frontier_points_[i].second = id;

                frontier_clusters_.push_back({frontier_points_[i].first, 1});
            }
        }

        //Divide point summation by n points --> centroid
        for(int i = 0; i < frontier_clusters_.size(); i++){
            if(frontier_clusters_[i].second > 0){
                frontier_clusters_[i].first.x() /= static_cast<double>(frontier_clusters_[i].second);
                frontier_clusters_[i].first.y() /= static_cast<double>(frontier_clusters_[i].second);
                frontier_clusters_[i].first.z() /= static_cast<double>(frontier_clusters_[i].second);
            }  
        }
    }

    void Frontier3DExtractionManager::clearMarkers(){

        if(marker_array_.markers.size() > 0){
            for(int i = 0; i < marker_array_.markers.size(); i++){
                marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
            }

            marker_pub_.publish(marker_array_);

            marker_array_.markers.clear();
        }
    }

    void Frontier3DExtractionManager::printMarkers(){

        int id = 0;

        clearMarkers();
           
        //Publish frontier points --> Markers
        for(int i = 0; i < frontier_points_.size(); i++){

            // if(frontier_clusters_[frontier_points_[i].second].second < min_frontier_points_)
            //     continue;

            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = octomap_msg_->resolution;

            marker.color.a = 1.0;
            if(frontier_clusters_[frontier_points_[i].second].second >= min_frontier_points_){
                marker.color.r = 1.0*static_cast<float>(frontier_points_[i].second)/
                                    static_cast<float>(frontier_clusters_.size());
                marker.color.b = 1.0 - marker.color.r;
                marker.color.g = 1.0 - 0.5*marker.color.r;
            }
            else{
                marker.color.r = 1.0;
                marker.color.b = 1.0;
                marker.color.g = 1.0;                
            }
            
            marker.pose.position.x = frontier_points_[i].first.x();
            marker.pose.position.y = frontier_points_[i].first.y();
            marker.pose.position.z = frontier_points_[i].first.z() + 0.025;

            marker.pose.orientation.w = 1.0f;
            marker.id = (id++);

            marker_array_.markers.push_back(marker);
        }

        // ADD CENTROIDS
        for(int i = 0; i < frontier_clusters_.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = octomap_msg_->resolution*1.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;

            marker.id = (id++);

            if(frontier_clusters_[i].second >= min_frontier_points_){
                marker.pose.position.x = frontier_clusters_[i].first.x();
                marker.pose.position.y = frontier_clusters_[i].first.y();
                marker.pose.position.z = frontier_clusters_[i].first.z() + 0.025;

                marker_array_.markers.push_back(marker);
            }
        }
    }

    void Frontier3DExtractionManager::main_loop(const ros::TimerEvent& timer)
    {
        time_ += period_; // update time
    }

    void Frontier3DExtractionManager::spin()
    {
        timer_.start();
        //ROS_INFO_STREAM("ForceEstim started looping time " << 1.0/_period << "Hz");
        ros::spin();
    }

    Frontier3DExtractionManager::~Frontier3DExtractionManager(){
    }
}
