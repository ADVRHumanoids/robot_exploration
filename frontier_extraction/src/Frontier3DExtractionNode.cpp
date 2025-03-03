#include <ros/ros.h>
#include <iostream>
#include <frontier_extraction/Frontier3DExtractionManager.h>

using namespace frontier_extraction;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "Frontier3DExtractionNode" );

    Frontier3DExtractionManager manager("frontier_extraction_node");
	manager.spin();

	return 0;
}
