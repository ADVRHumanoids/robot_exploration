#include <frontier_extraction/frontier_3d_extraction_manager.h>
#include <chrono>

namespace frontier_extraction{

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    
    Frontier3DExtractionManager::Frontier3DExtractionManager ()
    : Node("frontier_3d_extractor")
    {
        initNode();
    }

    void Frontier3DExtractionManager::initNode()
    {
       //Service Server
        get_frontiers_srv_ = this->create_service<frontier_extraction_srvs::srv::GetFrontiers>("/get_frontiers",
                             std::bind(&Frontier3DExtractionManager::getFrontiersSrv, this, 
                                        std::placeholders::_1, std::placeholders::_2));
        //Publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers",
                            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        
        
        //NOTE: Instead of subscriber can also use: /local/octomap_full as service!!

        // Subscriber
        auto octomapCallback =
            [this](octomap_msgs::msg::Octomap::SharedPtr msg) -> void {
                octomap_msg_ = msg;

                if(octomap_msg_ != nullptr){
                    //TODO
                    octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*octomap_msg_));
                }
            };

        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/local/octomap_full", 10, octomapCallback);

        min_frontier_points_ = 5;

        frontier_points_ = {};
        frontier_clusters_ = {};
    }
    
    void Frontier3DExtractionManager::getFrontiersSrv(const std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Request>  request,
                                                      std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response> response)
    {
        if(octree_ == nullptr)
            return ;

        auto t1 = high_resolution_clock::now();
        pelvis_pos_map_.x() = request->robot_pose.x;
        pelvis_pos_map_.y() = request->robot_pose.y;
        pelvis_pos_map_.z() = request->robot_pose.z;

        extractFrontiers();        
        auto t2 = high_resolution_clock::now();

        // RCLCPP_INFO(this->get_logger(), "Frontier Points added: %ld", static_cast<int>(frontier_points_.size()));
        
        // Publish frontiers and set service response
        printMarkers();
        auto t3 = high_resolution_clock::now();

        marker_pub_->publish(marker_array_);
        
        duration<double, std::milli> ms_double = t2 - t1;
        // RCLCPP_INFO(this->get_logger(), "Extract Frontiers: %f", ms_double.count());
        ms_double = t3 - t2;
        // RCLCPP_INFO(this->get_logger(), "Markers: %f", ms_double.count());

        setFrontierResponse(response);
    }

    void Frontier3DExtractionManager::setFrontierResponse(std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response> response)
    {
        // Fill response message with frontiers bigger than X pixels
        for(int i = 0; i < static_cast<int>(frontier_clusters_.size()); i++){
            if(frontier_clusters_[i].second >= min_frontier_points_){
                frontier_extraction_msgs::msg::Frontier f;

                f.centroid.x = frontier_clusters_[i].first.x();
                f.centroid.y = frontier_clusters_[i].first.y();
                f.centroid.z = frontier_clusters_[i].first.z();

                f.number_of_points = frontier_clusters_[i].second;

                response->frontiers.push_back(f);
            }
        }
    }

    bool Frontier3DExtractionManager::isFrontierXY(const point3d &p){
        //if there are unknown around the cube, the cube is frontier
        num_occupied_ = 0;
        
        // Check neighbors at the same level (Z) of the candidate 
        for(int f_x = -1; f_x <= 1; f_x++){
            for(int f_y = -1; f_y <= 1; f_y++){
                if(f_x == 0 && f_y == 0)
                    continue;

                n_cur_frontier_ = octree_->search(point3d(p.x() + f_x*octree_->getResolution(),
                                                          p.y() + f_y*octree_->getResolution(),
                                                          p.z()));
            
                if(n_cur_frontier_ != nullptr && octree_->isNodeOccupied(n_cur_frontier_))
                    num_occupied_ ++;
            }
        }

        bool frontier = false;
        if(num_occupied_ > 5)
            return false;
        else if(num_occupied_ >= 3)
            frontier = true;
        
        int num_occupied_up_ = 0;
        // Check neighbors at one level higher (Z) of the candidate 
        for(int f_x = -1; f_x <= 1; f_x++){
            for(int f_y = -1; f_y <= 1; f_y++){

                n_cur_frontier_ = octree_->search(point3d(p.x() + f_x*octree_->getResolution(),
                                                          p.y() + f_y*octree_->getResolution(),
                                                          p.z() + 1.0*octree_->getResolution()));
            
                if(n_cur_frontier_ != nullptr && octree_->isNodeOccupied(n_cur_frontier_)){
                    num_occupied_up_ ++;
                    if(frontier && num_occupied_up_ >= 2)
                        return false;
                    else if(!frontier && num_occupied_up_ >= 3)
                        return false;
                }
            }
        }
        
        //Not a frontier if below ALL occupied (to be more robust on the floor with noise)
        for(int f_x = -1; f_x <= 1; f_x++){
            for(int f_y = -1; f_y <= 1; f_y++){

                n_cur_frontier_ = octree_->search(point3d(p.x() + f_x*octree_->getResolution(),
                                                          p.y() + f_y*octree_->getResolution(),
                                                          p.z() - 1.0*octree_->getResolution()));
            
                if(n_cur_frontier_ == nullptr || !octree_->isNodeOccupied(n_cur_frontier_))
                    return true;
            }
        }

        return false;
    }

    void Frontier3DExtractionManager::extractFrontiers(){
        frontier_points_.clear();
        
        // RCLCPP_INFO(this->get_logger(), "Extract Frontiers");
        //Get Frontier Points
        //Max Tree Depth 16 --> Speed up considering higher Depth  /octree_->getTreeDepth()
        for(octomap::OcTree::leaf_iterator n = octree_->begin_leafs(15); n != octree_->end_leafs(); ++n)
        {
            // unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

            if(octree_->isNodeOccupied(*n) && isFrontierXY(n.getCoordinate()))
                frontier_points_.push_back({point3d(n.getX(), n.getY(), n.getZ()), 0});
        }

        // RCLCPP_INFO(this->get_logger(), "Cluster Frontier Points");
        //Cluster frontier points
        int id = 1;
        double max_distance_ = pow(2.5*2.0*octree_->getResolution(), 2);
        
        if(static_cast<int>(frontier_points_.size()) > 0){
            frontier_points_[0].second = 1;
            frontier_clusters_.resize(2);
            frontier_clusters_[1].first = frontier_points_[0].first;
            frontier_clusters_[1].second = 1;
        }
        else{
            frontier_clusters_.clear();
        }

        for(int i = 1; i < static_cast<int>(frontier_points_.size()); i++){                 
            for(int j = 0; j < i; j++){       

                if(fabs(frontier_points_[i].first.z() - frontier_points_[j].first.z()) >= 1.5*octree_->getResolution())
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
                        int temp_cluster_id = frontier_points_[j].second;
                        for(int k = 0; k < i; k++){
                            if(frontier_points_[k].second == temp_cluster_id){
                                //Update centroid - num points once
                                if(frontier_clusters_[frontier_points_[k].second].second > 0){
                                    frontier_clusters_[frontier_points_[i].second].first.x() += frontier_clusters_[frontier_points_[k].second].first.x();
                                    frontier_clusters_[frontier_points_[i].second].first.y() += frontier_clusters_[frontier_points_[k].second].first.y();
                                    frontier_clusters_[frontier_points_[i].second].first.z() += frontier_clusters_[frontier_points_[k].second].first.z();
                                    frontier_clusters_[frontier_points_[i].second].second += frontier_clusters_[frontier_points_[k].second].second;
                                    
                                    frontier_clusters_[frontier_points_[k].second].first.x() = 0;
                                    frontier_clusters_[frontier_points_[k].second].first.y() = 0;
                                    frontier_clusters_[frontier_points_[k].second].first.z() = 0;

                                    frontier_clusters_[frontier_points_[k].second].second = 0;
                                }
                                //Update all cluster id
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
        for(int i = 0; i < static_cast<int>(frontier_clusters_.size()); i++){
            if(frontier_clusters_[i].second > 0){
                frontier_clusters_[i].first.x() /= static_cast<double>(frontier_clusters_[i].second);
                frontier_clusters_[i].first.y() /= static_cast<double>(frontier_clusters_[i].second);
                frontier_clusters_[i].first.z() /= static_cast<double>(frontier_clusters_[i].second);
            }  
        }
    }

    void Frontier3DExtractionManager::clearMarkers(){

        if(static_cast<int>(marker_array_.markers.size()) > 0){
            for(int i = 0; i < static_cast<int>(marker_array_.markers.size()); i++){
                marker_array_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
            }

            marker_pub_->publish(marker_array_);

            marker_array_.markers.clear();
        }
    }

    void Frontier3DExtractionManager::printMarkers(){

        int id = 0;

        clearMarkers();
           
        //Publish frontier points --> Markers
        for(int i = 0; i < static_cast<int>(frontier_points_.size()); i++){

            // if(frontier_clusters_[frontier_points_[i].second].second < min_frontier_points_)
            //     continue;

            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = octomap_msg_->resolution;

            marker.color.a = 1.0;
            if(frontier_clusters_[frontier_points_[i].second].second >= min_frontier_points_){
                marker.color.r = 1.0*static_cast<float>(frontier_points_[i].second)/
                                    static_cast<float>(static_cast<int>(frontier_clusters_.size()));
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
        for(int i = 0; i < static_cast<int>(frontier_clusters_.size()); i++){
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
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

    Frontier3DExtractionManager::~Frontier3DExtractionManager(){
    }
}
