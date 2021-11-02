
#include "ros/ros.h"
#include "robocluedo_msgs/GoTo.h"
#include "std_msgs/Empty.h"
#include <iostream>
#include <string>

#define SERVICE_GO_TO "/go_to"
#define PUBLISHER_HINT_SIGNAL "/hint_signal"

#define OUTLABEL "[cluedo_movement_controller]"
#define OUTLOG std::cout << OUTLABEL << " "
#define LOGSQUARE( str ) "[" << str << "] "


// publisher to hint_signal
ros::Publisher* pub_hint_signal;


// stub mplementation of the movement service
bool GoToCallback( robocluedo_msgs::GoTo::Request& where, robocluedo_msgs::GoTo::Response& success )
{
	// "go to" the given position
	(ros::Duration(1)).sleep();
	ROS_INFO( "%s position reached -> %s", OUTLABEL, where.where.c_str() );
	
	// signal the event to the oracle
	ROS_INFO( "%s issuing signal to the Oracle...", OUTLABEL );
	pub_hint_signal->publish( std_msgs::Empty( ) );
	
	// return success
	success.success  = true;
	return true;
	
}


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "cluedo_movement_controller" );
	ros::NodeHandle nh;
	
	// expose the service go_to
	OUTLOG << "Advertising service " << LOGSQUARE( SERVICE_GO_TO ) << "..." << std::endl;
	ros::ServiceServer srv_goto = nh.advertiseService( SERVICE_GO_TO, GoToCallback );
	OUTLOG << "Advertising service " << LOGSQUARE( SERVICE_GO_TO ) << "... OK" << std::endl;
	
	// publisher to the Orace
	OUTLOG << "Creating publisher " << LOGSQUARE( PUBLISHER_HINT_SIGNAL ) << "..." << std::endl;
	ros::Publisher pub = nh.advertise<std_msgs::Empty>( PUBLISHER_HINT_SIGNAL, 1000 );
	OUTLOG << "Creating publisher " << LOGSQUARE( PUBLISHER_HINT_SIGNAL ) << "... OK" << std::endl;
	pub_hint_signal = &pub;
	
	// spin
	OUTLOG << "ready!" << std::endl;
	ros::spin( );
	
	return 0;
}
