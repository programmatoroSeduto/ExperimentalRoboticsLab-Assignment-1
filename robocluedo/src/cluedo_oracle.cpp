
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "robocluedo_msgs/Hint.h"
#include "robocluedo_msgs/CheckSolution.h"

#include <vector>
#include <string>
#include <random>
#include <fstream>
#include <algorithm>

#define PUBLISHER_HINT "/hint"
#define SUBSCRIBER_HINT_SIGNAL "/hint_signal"
#define SERVICE_CHECK_SOLUTION "/check_solution"

#define PATH_PARAMETER_SERVER_WHERE "cluedo_path_where"
#define PATH_PARAMETER_SERVER_WHO "cluedo_path_who"
#define PATH_PARAMETER_SERVER_WHAT "cluedo_path_what"
#define MAX_NUM_HINTS 25
#define MAX_SIZE_HINT 4

#define OUTLABEL "[cluedo_oracle] "
#define OUTLOG std::cout << OUTLABEL << " "
#define OUTERR OUTLOG << "ERROR: "
#define LOGSQUARE( str ) "[" << str << "] "



// the entire set of hints
std::vector<std::string> hints_who;
std::vector<std::string> hints_where;
std::vector<std::string> hints_what;

// who killed Dr Black
robocluedo_msgs::Hint solution_who;

// where Dr Black was killed
robocluedo_msgs::Hint solution_where;

// what's the murder weapon
robocluedo_msgs::Hint solution_what;

// the algorithm for generating random numbers
std::mt19937 rng;

// the channel for the hints
ros::Publisher* hint_channel;

// the set of hypotheses
std::vector<robocluedo_msgs::Hint> mysterylist;



// generate random numbers from 0 to capmax included
int randomIndex( int capmax )
{
	if( capmax == 0 ) return 0;
	
	std::uniform_int_distribution<std::mt19937::result_type> randgen( 0, capmax );
	return randgen( rng );
}



// read entities from the files
bool importDataFrom( std::string path, std::vector<std::string>& list )
{
	// open the file
	OUTLOG << "reading from fiile " << LOGSQUARE( path ) << std::endl;
	std::ifstream filestream( path );
	if( !filestream.is_open( ) )
	{
		// OUTLOG << "ERROR: no existing file!" << std::endl;
		return false;
	}
	
	// read the file
	// rooms = std::vector<std::string>( );
	std::string temp = "";
	int line = 1;
	while( getline( filestream, temp ) )
	{
		OUTLOG << "line" << LOGSQUARE( line ) << "READ " << LOGSQUARE( temp ) << std::endl;
		++line;
		list.push_back( temp );
	}
	
	// close the file
	OUTLOG << "closing file ..." << std::endl;
	filestream.close( );
	
	return true;
}



// choose randomly a hint, and delete it from the list
std::string chooseHintFrom( std::vector<std::string>& list )
{
	int ridx = randomIndex( list.size()-1 );
	std::string choice = list[ ridx ];
	
	return choice;
}



// callback: hint signal
void hintCallback( const std_msgs::EmptyConstPtr& emptySignal )
{
	// should the oracle to provide the solution?
	if( !randomIndex( 1 ) ) 
	{
		ROS_INFO_STREAM( OUTLABEL << "hint requeste refused. " );
		return;
	}
	else if( mysterylist.empty( ) )
	{
		ROS_INFO_STREAM( OUTLABEL << "MysteryLIst is empty. " );
		return;
	}
	
	// get the last message
	robocluedo_msgs::Hint h = *(mysterylist.end() - 1);
	mysterylist.pop_back( );
	
	// prepare the message and publish it
	ROS_INFO_STREAM( OUTLABEL << "publishing hint (" << "ID:" << h.HintID << ", PROP:" << h.HintType << ", VALUE:" << h.HintContent << ")" );
	hint_channel->publish( h );
}



// check if the solution from the robot is correct
bool checkSolutionCallback( robocluedo_msgs::CheckSolution::Request& hyp, robocluedo_msgs::CheckSolution::Response& misterySolved )
{
	ROS_INFO_STREAM( OUTLABEL << "evaluating the solution WHERE" << LOGSQUARE( hyp.Where ) << " WHO " << LOGSQUARE( hyp.Who ) << " WHAT " << LOGSQUARE( hyp.What ) );
	if( (hyp.Who != solution_who.HintContent) || (hyp.Where != solution_where.HintContent) || (hyp.What != solution_what.HintContent) )
	{
		ROS_INFO_STREAM( OUTLABEL << "solution wrong. " );
		misterySolved.MysterySolved = false;
	}
	else
	{
		ROS_INFO_STREAM( OUTLABEL << "SUCCESS! Found the solution. " );
		misterySolved.MysterySolved = true;
	}
	
	return true;
}



// generate the solution of the case
void generateMystery( std::vector<std::string> list_who, std::vector<std::string> list_where, std::vector<std::string> list_what )
{
	ROS_INFO_STREAM( OUTLABEL << "case generation started " );
	
	// shuffle the arrays before starting
	std::random_shuffle( list_who.begin(), list_who.end() );
	std::random_shuffle( list_where.begin(), list_where.end() );
	std::random_shuffle( list_what.begin(), list_what.end() );
	
	// generate the solution without the ID
	solution_where.HintType = "where";
	solution_where.HintContent = chooseHintFrom( list_where );
	
	solution_who.HintType = "who";
	solution_who.HintContent = chooseHintFrom( list_who );
	
	solution_what.HintType = "what";
	solution_what.HintContent = chooseHintFrom( list_what );
	
	ROS_INFO_STREAM( OUTLABEL << "the solution is " << "(where:" << solution_where.HintContent << ", who:" << solution_who.HintContent << ", what:" << solution_what.HintContent << ")" );
	
	// generate the ID of the solution
	int solutionID = randomIndex( MAX_NUM_HINTS-1 );
	solution_who.HintID = solutionID;
	solution_where.HintID = solutionID;
	solution_what.HintID = solutionID;
	
	ROS_INFO_STREAM( OUTLABEL << "the solution has ID:" << solutionID );
	
	/*
	 * for MAX_NUM_HINTS times:
	 * 	generate a number from 3 to MAX_SIZE_HINT
	 * 	for the number choosen before:
	 *    choose one of the lists (number from 0 to 2)
	 *    take the element from the selected list
	 *    type and value
	 *    and put it into the mystery list in the right cell
	 * 
	 * if the ID belongs to the solution, insert it instead of another new random hint
	 */
	for( int i=0; i<MAX_NUM_HINTS; ++i )
	{
		if( i == solutionID )
		{
			mysterylist.push_back( solution_what );
			mysterylist.push_back( solution_where );
			mysterylist.push_back( solution_who );
		}
		else
		{
			for( int j=0; j<MAX_SIZE_HINT; ++j )
			{
				robocluedo_msgs::Hint h;
				h.HintID = i;
				switch( randomIndex( 2 ) )
				{
				case 0:
					h.HintType = "who";
					h.HintContent = chooseHintFrom( list_who );
				break;
				case 1:
					h.HintType = "where";
					h.HintContent = chooseHintFrom( list_where );
				break;
				case 2:
					h.HintType = "what";
					h.HintContent = chooseHintFrom( list_what );
				break;
				}
				
				mysterylist.push_back( h );
			}
		}
	}
	
	ROS_INFO_STREAM( OUTLABEL << "hints generation finished. Generated: " << mysterylist.size() );
	
	// final shuffle
	std::random_shuffle( mysterylist.begin(), mysterylist.end() );
}



// main of the oracle
int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "cluedo_oracle" );
	ros::NodeHandle nh;
	
	// get the paths of the config files from the parameter server
	std::string path_who = "";
	std::string path_where = "";
	std::string path_what = "";
	if( !ros::param::has( PATH_PARAMETER_SERVER_WHO ) )
	{
		OUTERR << "unable to find the parameter " << LOGSQUARE( PATH_PARAMETER_SERVER_WHO ) << std::endl;
		return 0;
	}
	else ros::param::get( PATH_PARAMETER_SERVER_WHO, path_who );
	if( !ros::param::has( PATH_PARAMETER_SERVER_WHERE ) )
	{
		OUTERR << "unable to find the parameter " << LOGSQUARE( PATH_PARAMETER_SERVER_WHERE ) << std::endl;
		return 0;
	}
	else ros::param::get( PATH_PARAMETER_SERVER_WHERE, path_where );
	if( !ros::param::has( PATH_PARAMETER_SERVER_WHAT ) )
	{
		OUTERR << "unable to find the parameter " << LOGSQUARE( PATH_PARAMETER_SERVER_WHAT ) << std::endl;
		return 0;
	}
	else ros::param::get( PATH_PARAMETER_SERVER_WHAT, path_what );
	
	// load data from who
	hints_who = std::vector<std::string>();
	if( !importDataFrom( path_who, hints_who ) )
	{
		OUTERR << "unable to locate the data file " << LOGSQUARE( path_who ) << std::endl;
		return 0;
	}
	// from where
	hints_where = std::vector<std::string>();
	if( !importDataFrom( path_where, hints_where ) )
	{
		OUTERR << "unable to locate the data file " << LOGSQUARE( path_where ) << std::endl;
		return 0;
	}
	// and from what
	hints_what = std::vector<std::string>();
	if( !importDataFrom( path_what, hints_what ) )
	{
		OUTERR << "unable to locate the data file " << LOGSQUARE( path_what ) << std::endl;
		return 0;
	}
	
	// setup the random number generator (seed?)
	std::random_device dev;
	rng = std::mt19937(dev());
	
	// generate the solution of the case
	generateMystery( hints_who, hints_where, hints_what );
	
	// subscriber: hint_signal
	OUTLOG << "subscribing to the topic " << LOGSQUARE( SUBSCRIBER_HINT_SIGNAL ) << "..." << std::endl;
	ros::Subscriber sub_hint_signal = nh.subscribe( SUBSCRIBER_HINT_SIGNAL, 1000, hintCallback );
	OUTLOG << "subscribing to the topic " << LOGSQUARE( SUBSCRIBER_HINT_SIGNAL ) << "... OK" << std::endl;
	
	// publisher: hint
	OUTLOG << "Creating publisher " << LOGSQUARE( PUBLISHER_HINT ) << "..." << std::endl;
	ros::Publisher pub_hint = nh.advertise<robocluedo_msgs::Hint>( PUBLISHER_HINT, 1000 );
	hint_channel = &pub_hint;
	OUTLOG << "Creating publisher " << LOGSQUARE( PUBLISHER_HINT ) << "... OK" << std::endl;
	
	// service: check_solution
	OUTLOG << "Advertising service " << LOGSQUARE( SERVICE_CHECK_SOLUTION  ) << "..." << std::endl;
	ros::ServiceServer srv = nh.advertiseService( SERVICE_CHECK_SOLUTION, checkSolutionCallback );
	OUTLOG << "Advertising service " << LOGSQUARE( SERVICE_CHECK_SOLUTION ) << "... OK" << std::endl;
	
	//spin
	OUTLOG << "ready!" << std::endl;
	ros::spin( );
	
	return 0;
}
