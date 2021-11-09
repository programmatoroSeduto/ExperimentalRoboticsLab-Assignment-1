
/********************************************//**
 *  
 * \file cluedo_random_room.cpp
 * <div><b>ROS Node Name</b> 
 * 		<ul><li>cluedo_random_room</li></ul></div>
 * \brief A simple service: choose randomly a room to reach. 
 * 
 * \authors Francesco Ganci (S4143910)
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * The node implements a service which returns a room, randomly choosen
 * among the available ones, to reach. 
 * </p>
 * 
 * <b>UML component</b><br>
 * (See \ref robocluedo_arch the overal architecture, for further informations)<br>
 * <img src="UML_components_cluedo_random_room.png" /><br>
 * 
 * <b>Services:</b> <br>
 * <ul>
 *     <li>
 * 			<i> \ref SERVICE_RANDOM_ROOM </i> : RandomRoom.srv <br>
 * 			    request for a random target <br><br>
 * 		</li>
 * </ul>
 * 
 * <b>Parameters:</b> <br>
 * <ul>
 * 		<li>
 * 			[GET] <i> \ref PATH_PARAMETER_SERVER_WHERE </i> : string <br>
 * 			   the path of the file containing the PLACEs <br><br>
 * 		</li>
 * </ul>
 * 
 ***********************************************/

#include "ros/ros.h"
#include "robocluedo_msgs/RandomRoom.h"

#include <vector>
#include <string>
#include <random>
#include <fstream>

#define PATH_PARAMETER_SERVER_WHERE "cluedo_path_where"
#define SERVICE_RANDOM_ROOM "/random_room"
#define OUTLOG std::cout << "[cluedo_random_room] "
#define LOGSQUARE( str ) "[" << str << "] "



/// the set of rooms
std::vector<std::string> rooms;

/// \private random number generator
std::uniform_int_distribution<std::mt19937::result_type> randgen;

/// \private
std::mt19937 rng;



/********************************************//**
 *  
 * \brief import names of the rooms from file
 * 
 * @param path the path of the file containing the names
 * 
 * @returns success or not
 * 
 ***********************************************/
bool ImportNamesOfRooms( const std::string& path )
{
	// open the file
	OUTLOG << "reading from fiile " << LOGSQUARE( path ) << std::endl;
	std::ifstream filestream( path );
	if( !filestream.is_open( ) )
	{
		OUTLOG << "ERROR: no existing file!" << std::endl;
		return false;
	}
	
	// read the file
	rooms = std::vector<std::string>( );
	std::string temp = "";
	int line = 1;
	while( getline( filestream, temp ) )
	{
		OUTLOG << "line" << LOGSQUARE( line ) << "READ " << LOGSQUARE( temp ) << std::endl;
		++line;
		rooms.push_back( temp );
	}
	
	// close the file
	OUTLOG << "closing file ..." << std::endl;
	filestream.close( );
	
	return true;
}



/********************************************//**
 *  
 * \brief get randomly the name of one room from the list \ref rooms .
 * 
 * @returns the room
 * 
 ***********************************************/
std::string Choose( )
{
	int generated_random_number = randgen( rng );
	ROS_INFO( "generated: %d", generated_random_number );
	
	return rooms[ generated_random_number ];
}



/********************************************//**
 *  
 * \brief implementation of service \ref SERVICE_RANDOM_ROOM
 * 
 * It returns a room among the available ones. 
 * 
 * @param empty the empty request
 * @param room the choosen room
 * 
 * @see RandomRoom.srv
 * 
 * @todo should the node choose a room which is not also the actual
 *    position? 
 * 
 ***********************************************/
bool ChooseRoomRandom( robocluedo_msgs::RandomRoom::Request& empty, robocluedo_msgs::RandomRoom::Response& room )
{
	room.room = Choose( );
	return true;
}



/********************************************//**
 *  
 * \brief ROS node main - cluedo_random_room
 * 
 * read the input file, expose the service, and spin. 
 * 
 ***********************************************/
int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "cluedo_random_room" );
	ros::NodeHandle nh;
	
	// init the list of rooms
	if( !ros::param::has( PATH_PARAMETER_SERVER_WHERE ) )
	{
		// ERRORE il param non esiste nel server
		return 0;
	}
	std::string path;
	ros::param::get( PATH_PARAMETER_SERVER_WHERE, path );
	if( !ImportNamesOfRooms( path ) )
	{
		// ERRORE il path non esiste
		return 0;
	}
	int nRooms = rooms.size( );
	
	// setup the random number generator
	std::random_device dev;
	// seed?
	rng = std::mt19937(dev());
	randgen = std::uniform_int_distribution<std::mt19937::result_type>( 0, nRooms-1 );
	
	// expose the service
	OUTLOG << "Advertising service " << LOGSQUARE( SERVICE_RANDOM_ROOM ) << "..." << std::endl;
	ros::ServiceServer srv = nh.advertiseService( SERVICE_RANDOM_ROOM, ChooseRoomRandom );
	OUTLOG << "Advertising service " << LOGSQUARE( SERVICE_RANDOM_ROOM ) << "... OK" << std::endl;
	
	// spin and wait
	OUTLOG << "ready!" << std::endl;
	ros::spin();
	
	return 0;
}
