#include <frontier_extraction/frontier_2d_extraction_manager.h>

namespace frontier_extraction{

    Frontier2DExtractionManager::Frontier2DExtractionManager()
    : Node("frontier_2d_extractor")
    {
        initNode();
    }

    void Frontier2DExtractionManager::initNode()
    {
        //Service Server
        get_frontiers_srv_ = this->create_service<frontier_extraction_srvs::srv::GetFrontiers>("/get_frontiers",
                             std::bind(&Frontier2DExtractionManager::getFrontiersSrv, this, std::placeholders::_1, std::placeholders::_2));
        //Publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers",
                            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
        
        // Subscriber
        auto occupancyCallback =
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
                occupancy_ = msg;
            };

        occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, occupancyCallback);


        min_frontier_points_ = 10;
    }
    
    void Frontier2DExtractionManager::getFrontiersSrv(const std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Request>  request,
                                                      std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response> response)
    {
        if(occupancy_ == nullptr)
            return ;

        // Extract Frontiers
        extractFrontiers(request->robot_pose);
        
        RCLCPP_INFO(this->get_logger(), "Frontier Points added: %ld", frontier_points_.size());
        
        // Publish frontiers and set service response
        printMarkers();

        marker_pub_->publish(marker_array_);
        
        setFrontierResponse(response);
    }

    void Frontier2DExtractionManager::setFrontierResponse(std::shared_ptr<frontier_extraction_srvs::srv::GetFrontiers::Response> response)
    {
        // Fill response message with frontiers bigger than X pixels
        for(int i = 0; i < static_cast<int>(frontiers_dim_.size()); i++){
            if(frontiers_dim_[i] >= min_frontier_points_){
                frontier_extraction_msgs::msg::Frontier f;

                f.centroid.x = centroids_[i][0];
                f.centroid.y = centroids_[i][1];

                f.number_of_points = frontiers_dim_[i];

                response->frontiers.push_back(f);
            }
        }
    }

    bool Frontier2DExtractionManager::isValidCell(const int idx) const { return (idx >= 0 && idx < width_*height_);}
   
    bool Frontier2DExtractionManager::isFrontier(const int idx1, const int idx2) const { 
         return ((occupancy_->data[idx1] == -1 && occupancy_->data[idx2] == 0) ||
                 (occupancy_->data[idx2] == -1 && occupancy_->data[idx1] == 0));
    }

    void Frontier2DExtractionManager::innerExtractFrontiers(const int idx){
        
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

    void Frontier2DExtractionManager::extractFrontiers(geometry_msgs::msg::Point robot_pose){

        resolution_ = occupancy_->info.resolution;
        width_ = occupancy_->info.width;
        height_ = occupancy_->info.height;

        frontier_points_.clear();

        temp_pos_ = static_cast<int>((robot_pose.x - occupancy_->info.origin.position.x)/resolution_ +
                                     ((robot_pose.y - occupancy_->info.origin.position.y)/resolution_)*width_);
        
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

    void Frontier2DExtractionManager::clearMarkers(){

        for(int i = 0; i < static_cast<int>(marker_array_.markers.size()); i++){
            marker_array_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
        }

        marker_pub_->publish(marker_array_);

        marker_array_.markers.clear();

        centroids_.clear();
        centroids_.resize(frontiers_dim_.size(), {0.0f ,0.0f});
    }

    void Frontier2DExtractionManager::printMarkers(){

        int id = 0;

        clearMarkers();
   
        for(auto itr : frontier_points_){
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25;

            if(frontiers_dim_[frontier_clustersing_[id]] < min_frontier_points_){
                id++;
                continue;
            }

            marker.color.a = 1.0;
            marker.color.r = 1.0*static_cast<float>(frontier_clustersing_[id])/static_cast<float>(max_cluster_);
            marker.color.b = 1.0 - marker.color.r;
            marker.color.g = 1.0 - 0.5*marker.color.r;
            
            marker.pose.position.x = (itr%width_)*resolution_ + occupancy_->info.origin.position.x;
            marker.pose.position.y = (itr/width_)*resolution_ + occupancy_->info.origin.position.y;

            centroids_[frontier_clustersing_[id]][0] += marker.pose.position.x;
            centroids_[frontier_clustersing_[id]][1] += marker.pose.position.y;

            marker.pose.orientation.w = 1.0f;
            marker.id = (id++);

            marker_array_.markers.push_back(marker);
        }

        //ADD CENTROIDS
        for(int i = 0; i < static_cast<int>(centroids_.size()); i++){
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "frontiers";
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
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

    void Frontier2DExtractionManager::frontierClustering(){
        
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

    Frontier2DExtractionManager::~Frontier2DExtractionManager(){
    }
}