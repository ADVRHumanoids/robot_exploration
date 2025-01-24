#include <ros/ros.h>
#include <iostream>
#include <frontier_extraction/FrontierExtractionManager.h>

using namespace frontier_extraction;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "FrontierExtractionNode" );

    FrontierExtractionManager manager("frontier_extraction_node");
	manager.spin();

	return 0;
}
