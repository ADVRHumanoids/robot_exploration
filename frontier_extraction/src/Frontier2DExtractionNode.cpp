#include <ros/ros.h>
#include <iostream>
#include <frontier_extraction/Frontier2DExtractionManager.h>

using namespace frontier_extraction;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "Frontier2DExtractionNode" );

    Frontier2DExtractionManager manager("frontier_extraction_node");
	manager.spin();

	return 0;
}
