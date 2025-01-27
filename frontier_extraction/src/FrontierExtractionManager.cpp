#include <frontier_extraction/FrontierExtractionManager.h>
#include <chrono>

namespace frontier_extraction{

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    FrontierExtractionManager::FrontierExtractionManager ( std::string ns, double rate )
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

    void FrontierExtractionManager::initROSNode(double rate)
    {
        // init ROS node
        rate_ = rate;
        period_ = 1.0 / rate_;
        timer_ = nh_.createTimer(ros::Duration(period_), &FrontierExtractionManager::main_loop, this, false, false);
        time_ = 0.0;

        get_frontiers_srv_ = nh_.advertiseService("get_frontiers", &FrontierExtractionManager::getFrontiersSrv, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frontier_markers", 1000);
    }

    void FrontierExtractionManager::initParams(){
        min_frontier_points_ = 10;
    }
    
    bool FrontierExtractionManager::getFrontiersSrv(frontier_extraction::GetFrontiers::Request  &req,
             frontier_extraction::GetFrontiers::Response &res)
    {
        // Collect occupancy map
        auto t1 = high_resolution_clock::now();
        grid_  = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("projected_map",ros::Duration(5));

        // Extract Frontiers
        auto t2 = high_resolution_clock::now();
        extractFrontiers();
        
        auto t3 = high_resolution_clock::now();

        ROS_INFO("Frontier Points added: %ld", frontier_points_.size());
        
        // Publish frontiers and set service response
        printMarkers();
        auto t4 = high_resolution_clock::now();

        marker_pub_.publish(marker_array_);
        
        duration<double, std::milli> ms_double = t2 - t1;
        ROS_INFO("TIME WAIT MSG: %f", ms_double.count());
        ms_double = t3 - t2;
        ROS_INFO("Extract Frontiers: %f", ms_double.count());
        ms_double = t4 - t3;
        ROS_INFO("Markers: %f", ms_double.count());

        setFrontierResponse(res);

        return true;
    }

    void FrontierExtractionManager::setFrontierResponse(frontier_extraction::GetFrontiers::Response &res)
    {
        // Fill response message with frontiers bigger than X pixels
        for(int i = 0; i < frontiers_dim_.size(); i++){
            if(frontiers_dim_[i] >= min_frontier_points_){
                frontier_extraction::Frontier f;

                f.centroid.x = centroids_[i][0];
                f.centroid.y = centroids_[i][1];

                f.number_of_points = frontiers_dim_[i];

                res.frontiers.push_back(f);
            }
        }
    }

    const bool FrontierExtractionManager::isValidCell(const int idx){ return (idx >= 0 && idx < width_*height_);}
   
    const bool FrontierExtractionManager::isFrontier(const int idx1, const int idx2){ 
         return ((grid_->data[idx1] == -1 && grid_->data[idx2] == 0) ||
                 (grid_->data[idx2] == -1 && grid_->data[idx1] == 0));
    }

    void FrontierExtractionManager::innerExtractFrontiers(const int idx){
        
        int temp = idx;
        bool frontier_found = false;

        for(i = -1; i <= 1; i++){
            for(j = -1; j <= 1; j++){
                if((i == 0 && j == 0) || (abs(i) == abs(j)))
                    continue;

                temp = idx + i + j*width_;

                if(isValidCell(temp)){
                    if(!frontier_found && isFrontier(idx, temp)){
                        frontier_points_.insert(idx);
                        frontier_found = true;
                    }
                    if(visited_.find(temp) == visited_.end())
                        to_visit_.insert(temp);
                }
            }
        }
    }

    void FrontierExtractionManager::extractFrontiers(){

        resolution_ = grid_->info.resolution;
        width_ = grid_->info.width;
        height_ = grid_->info.height;

        frontier_points_.clear();

        Affine3d robot_pose_ = Affine3d::Identity();

        temp_pos_ = static_cast<int>((robot_pose_(0,3) - grid_->info.origin.position.x)/resolution_ +
                                     ((robot_pose_(1,3) - grid_->info.origin.position.y)/resolution_)*width_);
        
        //Start "wave" expansion
        if(isValidCell(temp_pos_)){
            to_visit_.clear();
            visited_.clear();

            to_visit_.insert(temp_pos_);

            while(to_visit_.size() > 0){
                temp_pos_ = *(to_visit_.begin());
                to_visit_.erase(to_visit_.begin());

                if(isValidCell(temp_pos_))
                    innerExtractFrontiers(temp_pos_);
                
                visited_.insert(temp_pos_);
            }
        }

        //Frontier clustering
        frontierClustering();
    }

    void FrontierExtractionManager::clearMarkers(){

        for(int i = 0; i < marker_array_.markers.size(); i++){
            marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        marker_pub_.publish(marker_array_);

        marker_array_.markers.clear();

        centroids_.clear();
        centroids_.resize(frontiers_dim_.size(), {0.0f ,0.0f});
    }

    void FrontierExtractionManager::printMarkers(){

        int id = 0;

        clearMarkers();
   
        for(auto itr : frontier_points_){
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25;

            if(frontiers_dim_[frontier_clustersing_[id]] < min_frontier_points_){
                id++;
                continue;
            }

            marker.color.a = 1.0;
            marker.color.r = 1.0*static_cast<float>(frontier_clustersing_[id])/static_cast<float>(max_cluster_);
            marker.color.b = 1.0 - marker.color.r;
            marker.color.g = 1.0 - 0.5*marker.color.r;
            
            marker.pose.position.x = (itr%width_)*resolution_ + grid_->info.origin.position.x;
            marker.pose.position.y = (itr/width_)*resolution_ + grid_->info.origin.position.y;

            centroids_[frontier_clustersing_[id]][0] += marker.pose.position.x;
            centroids_[frontier_clustersing_[id]][1] += marker.pose.position.y;

            marker.pose.orientation.w = 1.0f;
            marker.id = (id++);

            marker_array_.markers.push_back(marker);
        }

        //ADD CENTROIDS
        for(int i = 0; i < centroids_.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
            marker.color.a = 0.85;
            marker.color.r = 1.0;

            marker.id = (id++);

            if(frontiers_dim_[i] >= min_frontier_points_){
                centroids_[i][0] = centroids_[i][0]/static_cast<float>(frontiers_dim_[i]);
                centroids_[i][1] = centroids_[i][1]/static_cast<float>(frontiers_dim_[i]);

                marker.pose.position.x = centroids_[i][0];
                marker.pose.position.y = centroids_[i][1];

                marker_array_.markers.push_back(marker);
            }
        }
    }

    void FrontierExtractionManager::frontierClustering(){
        
        frontier_clustersing_.clear();
        frontier_clustersing_.resize(frontier_points_.size(), 0);

        int id = 0, id2 = 0;
        max_cluster_ = 1;
        
        int distance;

        int max_distance = 1;

        //frontier_clustersing_[id] = max_cluster_;
        frontiers_dim_ = {0};

        // For each point classified as frontier
        for(auto itr : frontier_points_){
            
            // Check min distance wrt already classified points            
            id2 = 0;
            for(auto itr2 : frontier_points_){
                               
                if(itr == itr2)
                    break;
                
                distance = std::abs(itr%width_ - itr2%width_) + std::abs(itr/width_ - itr2/width_);

                if(distance <= max_distance){
                    if(frontier_clustersing_[id] != 0 && frontier_clustersing_[id] != frontier_clustersing_[id2])
                    {
                        for(int i = 0; i < id; i++){
                            if(frontier_clustersing_[i] == frontier_clustersing_[id])
                                frontier_clustersing_[i] = frontier_clustersing_[id2];
                        }

                        frontiers_dim_[frontier_clustersing_[id2]] += frontiers_dim_[frontier_clustersing_[id]];
                        frontiers_dim_[frontier_clustersing_[id]] = 0;
                    }

                    frontier_clustersing_[id] = frontier_clustersing_[id2];
                }

                id2 ++;
            }

            if(frontier_clustersing_[id] == 0){
                frontier_clustersing_[id] = (max_cluster_++);
                frontiers_dim_.push_back(1);
            }
            else
                frontiers_dim_[frontier_clustersing_[id]] ++;

            id ++;
        }
    }

    void FrontierExtractionManager::main_loop(const ros::TimerEvent& timer)
    {
        time_ += period_; // update time
    }

    void FrontierExtractionManager::spin()
    {
        timer_.start();
        //ROS_INFO_STREAM("ForceEstim started looping time " << 1.0/_period << "Hz");
        ros::spin();
    }

    FrontierExtractionManager::~FrontierExtractionManager(){
    }
}