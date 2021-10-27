
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "robocluedo/Hint.h"
#include "robocluedo/CheckSolution.h"

#include <vector>
#include <string>
#include <random>
#include <fstream>

#define PUBLISHER_HINT "/hint"
#define SUBSCRIBER_HINT_SIGNAL "/hint_signal"
#define SERVICE_CHECK_SOLUTION "/check_solution"

#define PATH_PARAMETER_SERVER_WHERE "cluedo_path_where"
#define PATH_PARAMETER_SERVER_WHO "cluedo_path_who"
#define PATH_PARAMETER_SERVER_WHAT "cluedo_path_what"

#define OUTLABEL "[cluedo_oracle] "
#define OUTLOG std::cout << OUTLABEL << " "
#define OUTERR OUTLOG << "ERROR: "
#define LOGSQUARE( str ) "[" << str << "] "


// the entire set of hints
std::vector<std::string> hints_who;
std::vector<std::string> hints_where;
std::vector<std::string> hints_what;

// who killed Dr Black
std::string solution_who;

// where Dr Black was killed
std::string solution_where;

// what's the murder weapon
std::string solution_what;

// the algorithm for generating random numbers
std::mt19937 rng;

// the channel for the hints
ros::Publisher* hint_channel;


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
	ROS_INFO_STREAM( OUTLABEL << "evaluated hint id " << LOGSQUARE( ridx ) << " value " << LOGSQUARE( choice ) );
	
	return choice;
}


// delete a hint
bool deleteHintFrom( std::string hintToRemove, std::vector<std::string>& list )
{
	if( list.size() < 1 ) return false;
	
	for( auto it = list.begin(); it != list.end(); ++it )
		if( *it == hintToRemove )
		{
			ROS_INFO_STREAM( OUTLABEL << "hint deleted: " << LOGSQUARE( *it ) );
			list.erase( it );
			return true;
		}
	
	return false;
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
	
	// prepare the choices
	std::vector<std::string> v;
	std::vector<std::string> vType;
	if( hints_who.size() > 0 ) 
	{
		v.push_back( chooseHintFrom( hints_who ) );
		vType.push_back( "WHO" );
	}
	if( hints_where.size() > 0 ) 
	{
		v.push_back( chooseHintFrom( hints_where ) );
		vType.push_back( "WHERE" );
	}
	if( hints_what.size() > 0 ) 
	{
		v.push_back( chooseHintFrom( hints_what ) );
		vType.push_back( "WHAT" );
	}
	
	// make the final choice
	robocluedo::Hint h;
	if( v.size() == 0 ) 
	{
		ROS_INFO_STREAM( OUTLABEL << "no hint to send. " );
		return;
	}
	else if( v.size() == 1 ) 
	{
		h.HintType = vType[0];
		h.HintContent = v[0];
	}
	else
	{
		int finalChoice = randomIndex( v.size()-1 );
		h.HintType = vType[ finalChoice ];
		h.HintContent = v[ finalChoice ];
	}
	ROS_INFO_STREAM( OUTLABEL << "sending hint type " << LOGSQUARE( h.HintType ) << "value " << LOGSQUARE( h.HintContent ) );
	
	// publish the hint
	hint_channel->publish( h );
	
	// delete the hint
	if( h.HintType == "WHO" )
		deleteHintFrom( h.HintContent, hints_who );
	if( h.HintType == "WHERE" )
		deleteHintFrom( h.HintContent, hints_where );
	if( h.HintType == "WHAT" )
		deleteHintFrom( h.HintContent, hints_what );
}


// check if the solution from the robot is correct
bool checkSolutionCallback( robocluedo::CheckSolution::Request& hyp, robocluedo::CheckSolution::Response& misterySolved )
{
	ROS_INFO_STREAM( OUTLABEL << "evaluating the solution WHERE" << LOGSQUARE( hyp.Where ) << " WHO " << LOGSQUARE( hyp.Who ) << " WHAT " << LOGSQUARE( hyp.What ) );
	if( (hyp.Who != solution_who) || (hyp.Where != solution_where) || (hyp.What != solution_what) )
	{
		ROS_INFO_STREAM( OUTLABEL << "solution wrong. " );
		misterySolved.MisterySolved = false;
		return false;
	}
	else
	{
		ROS_INFO_STREAM( OUTLABEL << "SUCCESS! Found the solution. " );
		misterySolved.MisterySolved = true;
		return true;
	}
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
	solution_who = chooseHintFrom( hints_who );
	solution_where = chooseHintFrom( hints_where );
	solution_what = chooseHintFrom( hints_what );
	
	// subscriber: hint_signal
	OUTLOG << "subscribing to the topic " << LOGSQUARE( SUBSCRIBER_HINT_SIGNAL ) << "..." << std::endl;
	ros::Subscriber sub_hint_signal = nh.subscribe( SUBSCRIBER_HINT_SIGNAL, 1000, hintCallback );
	OUTLOG << "subscribing to the topic " << LOGSQUARE( SUBSCRIBER_HINT_SIGNAL ) << "... OK" << std::endl;
	
	// publisher: hint
	OUTLOG << "Creating publisher " << LOGSQUARE( PUBLISHER_HINT ) << "..." << std::endl;
	ros::Publisher pub_hint = nh.advertise<robocluedo::Hint>( PUBLISHER_HINT, 1000 );
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
