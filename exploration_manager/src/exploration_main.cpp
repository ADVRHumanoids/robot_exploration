#include <ros/ros.h>
#include <iostream>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <exploration_manager/CheckExplorationRequest.h>
#include <exploration_manager/CollectObjectPose.h>
#include <exploration_manager/CollectFrontiers.h>
#include <exploration_manager/Explore.h>
#include <exploration_manager/CheckLocomotionStatus.h>
#include <exploration_manager/SendNavPose.h>

#include <exploration_manager/IsExplorationRequired.h>

#include <exploration_manager/SharedClass.h>


int main(int argc, char **argv)
{
	ros::init (argc, argv, "ExplorationManager" );
    
    if(argc < 1)
    {
        ROS_INFO("Needed param: config file name (.xml)");
        return 0;
    }
    else{
        ROS_INFO("Received: %s", argv[1]);
    }

    ros::NodeHandle nh;

    bt_data_ = new SharedClass();

    ros::param::get("robot_exploration/world_frame", bt_data_->world_frame);
    ros::param::get("robot_exploration/base_frame", bt_data_->base_frame);
    
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;
    
    factory.registerSimpleCondition("IsExplorationRequired", std::bind(IsExplorationRequired));
   
    factory.registerNodeType<CheckExplorationRequest>("CheckExplorationRequest");
    factory.registerNodeType<CollectObjectPose>("CollectObjectPose");
    factory.registerNodeType<CollectFrontiers>("CollectFrontiers");
    factory.registerNodeType<Explore>("Explore");
    factory.registerNodeType<CheckLocomotionStatus>("CheckLocomotionStatus");
    factory.registerNodeType<SendNavPose>("SendNavPose");
    
    auto tree = factory.createTreeFromFile(argv[1]);

    PublisherZMQ publisher_zmq(tree);
    BT::FileLogger logger_file(tree, "bt_exploration.fbl");

    ros::Rate loop_rate(25);
    while(ros::ok()){
        tree.tickRoot();
                
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
