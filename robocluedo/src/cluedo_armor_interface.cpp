
#include "ros/ros.h"
#include "armor_tools/armor_tools.h"

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "cluedo_armor_interface" );
	ros::NodeHandle nh;
	
	ArmorTools tools;
	
	return 0;
}
