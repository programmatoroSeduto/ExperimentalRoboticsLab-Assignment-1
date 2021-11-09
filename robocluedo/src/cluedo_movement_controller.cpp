
/********************************************//**
 *  
 * \file cluedo_movement_controller.cpp
 * <div><b>ROS Node Name</b> 
 *      <ul><li>cluedo_movement_controller</li></ul></div>
 * \brief stub ROS node, movement controller
 * 
 * \authors Francesco Ganci (S4143910)
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * It represents the 'act' part inside the behavioural architecture. Actually
 * it doesn't do anything: it is a 'wait' which can signal the Oracle (as a laser
 * sensor when the robot enters in a room). In future, it should become a
 * path planning facility. 
 * </p>
 * 
 * <b>UML component</b><br>
 * (See \ref robocluedo_arch the overal architecture, for further informations)<br>
 * <img src="UML_components_cluedo_movement_controller.png" /><br>
 * 
 * <b>Publishers:</b> <br>
 * <ul>
 *     <li>
 * 			<i> \ref PUBLISHER_HINT_SIGNAL</i> : <a href="http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html">std_msgs::Empty</a> <br>
 * 			signal for the Oracle: hint "trigger" <br><br>
 * 		</li>
 * </ul>
 * 
 * <b>Services:</b> <br>
 * <ul>
 *     <li>
 * 			<i> \ref SERVICE_GO_TO</i> : GoTo.srv <br>
 * 			give the controller the command to go somewhere <br><br>
 * 		</li>
 * </ul>
 * 
 * @todo this is a <i>STUB implementation</i>. It should be replaced 
 *     with a real movement controller. 
 * 
 ***********************************************/

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



/// \private publisher to hint_signal
ros::Publisher* pub_hint_signal;



/********************************************//**
 *  
 * \brief implementation of service \ref SERVICE_INTERFACE_FIND_CONSISTENT_HYP
 * 
 * Stub mplementation of the movement service. It waits one second in order
 * to "simulate" the duration of the motion, and issues a signal to the Oracle.
 * 
 * @param where the room to reach
 * @param success success flag
 * 
 * @see GoTo.srv
 * @see cluedo_oracle.cpp
 * 
 ***********************************************/
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



/********************************************//**
 *  
 * \brief ROS node main - cluedo_movement_controller
 * 
 * spawning of the service, publisher to the oracle, and spin. 
 * 
 ***********************************************/
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
