
/********************************************//**
 *  
 * \file cluedo_oracle.cpp
 * <div><b>ROS Node Name</b> 
 *      <ul><li>cluedo_oracle</li></ul></div>
 * \brief The referee of the game
 * 
 * \authors Francesco Ganci (S4143910)
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * This node implements a referee for the game: it prepares the hints and
 * the mystery, then sends the hints to the robot and checks the 
 * hypotheses from the robot. <br>
 * </p>
 * 
 * <b>UML component</b><br>
 * (See \ref robocluedo_arch the overal architecture, for further informations)<br>
 * <img src="UML_components_cluedo_oracle.png" /><br>
 * 
 * <b>Publishers:</b> <br>
 * <ul>
 *     <li>
 * 			<i> \ref PUBLISHER_HINT </i> : Hint.msg <br>
 * 			    the channel which the hint is issued through <br><br>
 * 		</li>
 * </ul>
 * 
 * <b>Subscribers:</b> <br>
 * <ul>
 *     <li>
 * 			<i> \ref SUBSCRIBER_HINT_SIGNAL </i> : <a href="http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html">std_msgs::Empty</a> <br>
 * 			    ask the oracle for a hint <br><br>
 * 		</li>
 * </ul>
 * 
 * <b>Services:</b> <br>
 * <ul>
 *     <li>
 * 			<i> \ref SERVICE_CHECK_SOLUTION </i> : CheckSolution.srv <br>
 * 			    check if a complete hypothesis is the solution of the case or not <br><br>
 * 		</li>
 * </ul>
 * 
 * <b>Parameters:</b> <br>
 * <ul>
 * 		<li>
 * 			[GET] <i> \ref PATH_PARAMETER_SERVER_WHERE </i> : string <br>
 * 			   path of the file containing all the PLACEs <br><br>
 * 		</li>
 * 		<li>
 * 			[GET] <i> \ref PATH_PARAMETER_SERVER_WHO </i> : string <br>
 * 			   path of the file containing all the PERSONs <br><br>
 * 		</li>
 * 		<li>
 * 			[GET] <i> \ref PATH_PARAMETER_SERVER_WHAT </i> : string <br>
 * 			   path of the file containing all the WEAPONs <br><br>
 * 		</li>
 * 		<li>
 * 			[GET, optional] <i>/cluedo_max_hypotheses</i> : string <br>
 * 			   the maximum number of hypothesis IDs <br><br>
 * 		</li>
 * </ul>
 * 
 * @todo expose the value \ref MAX_SIZE_HINT o the parameter server as done with \ref MAX_NUM_HINTS.
 * 
 * 
 ***********************************************/
 
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

/// default maximum number of hint IDs
#define MAX_NUM_HINTS 2

/// <i>maximum number of</i> hints inside each ID
#define MAX_SIZE_HINT 10

#define OUTLABEL "[cluedo_oracle] "
#define OUTLOG( msg ) ROS_INFO_STREAM( OUTLABEL << msg )
#define OUTERR( msg ) OUTLOG( "ERROR: " << msg )
#define LOGSQUARE( str ) "[" << str << "] "



/// vector of hints - who
std::vector<std::string> hints_who;

/// vector of hints - where
std::vector<std::string> hints_where;

/// vector of hints - what
std::vector<std::string> hints_what;

/// SOLUTION - who killed Dr Black
robocluedo_msgs::Hint solution_who;

/// SOLUTION - where Dr Black was killed
robocluedo_msgs::Hint solution_where;

/// SOLUTION - what's the murder weapon
robocluedo_msgs::Hint solution_what;

/// \private the algorithm for generating random numbers
std::mt19937 rng;

/// \private the channel for the hints
ros::Publisher* hint_channel;

/// shuffled list of hypotheses
std::vector<robocluedo_msgs::Hint> mysterylist;



/// \private generate random numbers from 0 to capmax included
int randomIndex( int capmax )
{
	if( capmax == 0 ) return 0;
	
	std::uniform_int_distribution<std::mt19937::result_type> randgen( 0, capmax );
	return randgen( rng );
}



/********************************************//**
 *  
 * \brief import entities from file
 * 
 * The function reads a text file line per line, and creates an item for
 * each line. Extremely simple: there's no parsing inside. 
 * 
 * @param path of the file to read
 * @param list where to put the entities
 * 
 * @returns success or not
 * 
 ***********************************************/
bool importDataFrom( std::string path, std::vector<std::string>& list )
{
	// open the file
	OUTLOG( "reading from fiile " << LOGSQUARE( path ) );
	std::ifstream filestream( path );
	if( !filestream.is_open( ) )
	{
		OUTERR( "no existing file!" );
		return false;
	}
	
	// read the file
	// rooms = std::vector<std::string>( );
	std::string temp = "";
	int line = 1;
	while( getline( filestream, temp ) )
	{
		OUTLOG( "line" << LOGSQUARE( line ) << "READ " << LOGSQUARE( temp ) );
		++line;
		list.push_back( temp );
	}
	
	// close the file
	OUTLOG( "closing file ..." );
	filestream.close( );
	
	return true;
}



/********************************************//**
 *  
 * \brief choose randomly a hint from a list
 * 
 * Same as the python method Random.choose()
 * 
 * @param list (reference) the list from where to take the hint
 * 
 * @returns the randomly choosen value from 'list'
 * 
 ***********************************************/
std::string chooseHintFrom( std::vector<std::string>& list )
{
	int ridx = randomIndex( list.size()-1 );
	std::string choice = list[ ridx ];
	
	return choice;
}



/********************************************//**
 *  
 * \brief subscriber to \ref SUBSCRIBER_HINT_SIGNAL and publisher of \ref PUBLISHER_HINT
 * 
 * The arrival of a message means that the robot has entered in one room. 
 * The Oracle can decide if drop the hint or not: if the Oracle refuses
 * the request, nothing is issued through the topic \ref PUBLISHER_HINT.
 * Otherwise, it takes from the list of hints \ref mysterylist the last 
 * one, remove it from the list, and publishes it. 
 * 
 * @param emptySignal empty 'request'
 * 
 * @see <a href="http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html">std_msgs::Empty</a> [IN]
 * @see Hint.msg [OUT]
 * 
 ***********************************************/
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



/********************************************//**
 *  
 * \brief implementation of service \ref SERVICE_CHECK_SOLUTION
 * 
 * During the game, the Oracle knows the solution of the case. The robot,
 * when is ready for the charge, can check if the consistent hypothesis
 * is the solution of the case using this service. 
 * 
 * @param hyp           the hypothesis to check
 * @param mysterySolved if the solution is correct or not
 * 
 * @see CheckSolution.srv
 * 
 ***********************************************/
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



/********************************************//**
 *  
 * \brief generate the solution of the case and the hints
 * 
 * This is called once when the system starts. Here is how it works:
 * <ol>
 * <li>shuffle of the three arrays from the text files</li>
 * <li>generation of the solution and its ID from 0 to (tot_hints-1)</li>
 * <li>
 * for each index from 0 to tot_hints-1 (the available IDs):
 * <ol>
 * <li>if the ID is the one choosen before for the solution, place the elements
 *      of the solution, then continue</li>
 * <li>else, for each index from 0 to \ref MAX_SIZE_HINTS -1:</li>
 * <ol>
 * <li>generate randomly one hint with the given ID and push inside the array</li>
 * <li>or don't generate</li>
 * </ol>
 * </ol>
 * </li>
 * <li>shuffle the final list</li>
 * </ol>
 * 
 * @param list_who the list of the individuals PERSON from file
 * @param list_where  the list of the individuals PLACE from file
 * @param list_whatthe list of the individuals WEAPON from file
 * @param tot_hints the maximum number of hints ID the oracle must generate
 * 
 * @todo could this algorithm be further improved? 
 * 
 ***********************************************/
void generateMystery( std::vector<std::string> list_who, std::vector<std::string> list_where, std::vector<std::string> list_what, int tot_hints )
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
	
	// generate the case
	for( int i=0; i<tot_hints; ++i )
	{
		if( i == solutionID )
		{
			mysterylist.push_back( solution_what );
			mysterylist.push_back( solution_where );
			mysterylist.push_back( solution_who );
			
			continue;
		}
		else
		{
			/// @todo let the user to modify this value instead of putting a constant. 
			for( int j=0; j<MAX_SIZE_HINT; ++j )
			{
				robocluedo_msgs::Hint h;
				h.HintID = i;
				/// @todo there should be parameters for altering the probabilities in generating the hints belonging to an ID
				switch( randomIndex( 5 ) )
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
				default:
					continue;
				break;
				}
				
				mysterylist.push_back( h );
			}
		}
	}
	
	ROS_INFO_STREAM( OUTLABEL << "hints generation finished. Generated: " << mysterylist.size() << " hints" );
	
	// final shuffle
	std::random_shuffle( mysterylist.begin(), mysterylist.end() );
}



/********************************************//**
 *  
 * \brief ROS node main - cluedo_oracle
 * 
 * find the parameters, load data, generate the mystery, setup the channels,
 * then spin. 
 * 
 ***********************************************/
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
		OUTERR( "unable to find the parameter " << LOGSQUARE( PATH_PARAMETER_SERVER_WHO ) );
		return 0;
	}
	else ros::param::get( PATH_PARAMETER_SERVER_WHO, path_who );
	if( !ros::param::has( PATH_PARAMETER_SERVER_WHERE ) )
	{
		OUTERR( "unable to find the parameter " << LOGSQUARE( PATH_PARAMETER_SERVER_WHERE ) );
		return 0;
	}
	else ros::param::get( PATH_PARAMETER_SERVER_WHERE, path_where );
	if( !ros::param::has( PATH_PARAMETER_SERVER_WHAT ) )
	{
		OUTERR( "unable to find the parameter " << LOGSQUARE( PATH_PARAMETER_SERVER_WHAT ) );
		return 0;
	}
	else ros::param::get( PATH_PARAMETER_SERVER_WHAT, path_what );
	
	// load data from who
	hints_who = std::vector<std::string>();
	if( !importDataFrom( path_who, hints_who ) )
	{
		OUTERR( "unable to locate the data file " << LOGSQUARE( path_who ) );
		return 0;
	}
	// from where
	hints_where = std::vector<std::string>();
	if( !importDataFrom( path_where, hints_where ) )
	{
		OUTERR( "unable to locate the data file " << LOGSQUARE( path_where ) );
		return 0;
	}
	// and from what
	hints_what = std::vector<std::string>();
	if( !importDataFrom( path_what, hints_what ) )
	{
		OUTERR( "unable to locate the data file " << LOGSQUARE( path_what ) );
		return 0;
	}
	
	// setup the random number generator (seed?)
	std::random_device dev;
	rng = std::mt19937(dev());
	
	// generate the solution of the case
	if( !ros::param::has( "cluedo_max_hypotheses" ) )
		generateMystery( hints_who, hints_where, hints_what, MAX_NUM_HINTS );
	else
	{
		int tot_hints;
		ros::param::get( "cluedo_max_hypotheses", tot_hints );
		ROS_INFO_STREAM( OUTLABEL << "found a max number of hypotheses: " << tot_hints );
		
		generateMystery( hints_who, hints_where, hints_what, tot_hints );
	}
	
	// subscriber: hint_signal
	OUTLOG( "subscribing to the topic " << LOGSQUARE( SUBSCRIBER_HINT_SIGNAL ) << "..." );
	ros::Subscriber sub_hint_signal = nh.subscribe( SUBSCRIBER_HINT_SIGNAL, 1000, hintCallback );
	OUTLOG( "subscribing to the topic " << LOGSQUARE( SUBSCRIBER_HINT_SIGNAL ) << "... OK" );
	
	// publisher: hint
	OUTLOG( "Creating publisher " << LOGSQUARE( PUBLISHER_HINT ) << "..." );
	ros::Publisher pub_hint = nh.advertise<robocluedo_msgs::Hint>( PUBLISHER_HINT, 1000 );
	hint_channel = &pub_hint;
	OUTLOG( "Creating publisher " << LOGSQUARE( PUBLISHER_HINT ) << "... OK" );
	
	// service: check_solution
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_CHECK_SOLUTION  ) << "..." );
	ros::ServiceServer srv = nh.advertiseService( SERVICE_CHECK_SOLUTION, checkSolutionCallback );
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_CHECK_SOLUTION ) << "... OK" );
	
	//spin
	OUTLOG( "ready!" );
	ros::spin( );
	
	return 0;
}
