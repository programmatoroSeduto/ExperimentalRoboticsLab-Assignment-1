
#include "ros/ros.h"
#include "robocluedo/RandomRoom.h"
#include <vector>
#include <string>
#include <random>
#include <fstream>

#define PATH_PARAMETER_SERVER_WHERE "cluedo_path_where"
#define SERVICE_RANDOM_ROOM "/random_room"
#define OUTLOG std::cout << "[cluedo_random_room] "
#define LOGSQUARE( str ) "[" << str << "] "


// the set of rooms
std::vector<std::string> rooms;


// random number generator
std::uniform_int_distribution<std::mt19937::result_type> randgen;
std::mt19937 rng;


// read the rooms from file
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


// get one room randomly
std::string Choose( )
{
	int generated_random_number = randgen( rng );
	//OUTLOG << " generated " << LOGSQUARE( generated_random_number );
	ROS_INFO( "generated: %d", generated_random_number );
	return rooms[ generated_random_number ];
	//return "STUB.";
}


// the service
bool ChooseRoomRandom( robocluedo::RandomRoom::Request& empty, robocluedo::RandomRoom::Response& room )
{
	room.room = Choose( );
	return true;
}


// ... 
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
