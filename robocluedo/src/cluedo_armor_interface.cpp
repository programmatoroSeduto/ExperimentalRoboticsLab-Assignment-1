
#include "ros/ros.h"
#include "armor_tools/armor_tools.h"
#include "armor_tools/armor_cluedo.h"

#include "std_srvs/Trigger.h"
#include "armor_msgs/ArmorDirective.h"
#include "armor_msgs/ArmorDirectiveList.h"
#include "armor_msgs/ArmorDirectiveReq.h"
#include "armor_msgs/ArmorDirectiveRes.h"
#include "armor_msgs/QueryItem.h"
#include "robocluedo_msgs/AddHint.h"
#include "robocluedo_msgs/Hypothesis.h"
#include "robocluedo_msgs/FindConsistentHypotheses.h"
#include "robocluedo_msgs/DiscardHypothesis.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define SERVICE_INTERFACE_ADD_HINT "/cluedo_armor/add_hint"
#define SERVICE_INTERFACE_FIND_CONSISTENT_HYP "/cluedo_armor/find_consistent_h"
#define SERVICE_INTERFACE_WRONG_HYPOTHESIS "/cluedo_armor/wrong_hypothesis"
#define SERVICE_INTERFACE_SAVE_ONTOLOGY "/cluedo_armor/backup"

#define ONTOLOGY_PARAM "cluedo_path_owlfile"
#define PARAM_ONTOLOGY_BACKUP_PATH "cluedo_path_owlfile_backup"
#define OUTLABEL "[cluedo_armor_interface]"
#define OUTLOG( msg ) ROS_INFO_STREAM( OUTLABEL << " " << msg );
#define LOGSQUARE( str ) "[" << str << "] "
#define SS( this_string ) std::string( this_string )
#define SSS( this_thing ) std::to_string( this_thing )


// connection to aRMOR
ArmorCluedo* armor;



// check if a given file exists
bool fileExist( std::string path )
{
    return (std::ifstream(path)).good();
}



// perform difference between the first array and the intersection between the two arrays
std::vector<std::string> PerformDifferenceBetween( std::vector<std::string> list1, std::vector<std::string> list2 )
{
	std::vector<std::string> to_return;
	
	for( auto it1 = list1.begin(); it1 != list1.end(); ++it1 )
	{
		bool found = false;
		for( auto it2 = list2.begin(); it2 != list2.end(); ++it2 )
		{
			if( *it2 == *it1 ) 
			{
				found = true;
				break;
			}
		}
		
		if( !found )
			to_return.push_back( *it1 );
	}
	
	return to_return;
}



// service SERVICE_INTERFACE_ADD_HINT
bool ServiceAddHint( robocluedo_msgs::AddHint::Request& hint, robocluedo_msgs::AddHint::Response& success )
{
	OUTLOG( "called service " << SERVICE_INTERFACE_ADD_HINT );
	
	// check for the existence of the given hypothesis ID (in case, create it)
	std::string hypname = "";
	hypname += SS("HP") + SSS( hint.hypID );
	if( !armor->ExistsIndiv( hypname ) )
		armor->AddIndiv( hypname, "HYPOTHESIS", false );
	
	// if the Belem is not defined, add it
	if( !armor->ExistsIndiv( hint.Belem ) )
	{
		if( hint.property == "where" )
			armor->AddIndiv( hint.Belem, "PLACE" );
		else if( hint.property == "who" )
			armor->AddIndiv( hint.Belem, "PERSON" );
		else if( hint.property == "what" )
			armor->AddIndiv( hint.Belem, "WEAPON" );
		else
		{
			OUTLOG( "ERROR: not a valid hint property." );
			success.success = false;
			
			return true;
		}
	}
	
	// add the predicate
	OUTLOG( "set object property " << "(" << hypname << ", " << hint.Belem << "):" << hint.property );
	armor->SetObjectProperty( hint.property, hypname, hint.Belem );
	
	// perform the update
	armor->UpdateOntology( );
	
	success.success = true;
	return true;
}



// service SERVICE_INTERFACE_FIND_CONSISTENT_HYP
bool ServiceFindConsistentHypotheses( robocluedo_msgs::FindConsistentHypotheses::Request& empty, robocluedo_msgs::FindConsistentHypotheses::Response& hyplist )
{
	OUTLOG( "called service " << SERVICE_INTERFACE_FIND_CONSISTENT_HYP );
	
	// perform the update before starting
	armor->UpdateOntology( );
	
	std::vector<std::string> list_to_return;
	{
		// get all the consistent hypotheses
		std::vector<std::string> consistent_hyp = armor->FindCompleteHypotheses( );
		
		// get the inconsistent hypotheses
		std::vector<std::string> inconsistent_hyp = armor->FindInconsistentHypotheses( );
		
		// remove the intersection between the two arrays
		list_to_return = PerformDifferenceBetween( consistent_hyp, inconsistent_hyp );
	}
	
	// expand the list
	hyplist.hyp = std::vector<robocluedo_msgs::Hypothesis>( );
	robocluedo_msgs::Hypothesis h;
	for( std::string hptag : list_to_return )
	{
		h.tag = hptag;
		h.who = armor->GetValuedOfIndiv( "who", hptag )[0];
		h.where = armor->GetValuedOfIndiv( "where", hptag )[0];
		h.what = armor->GetValuedOfIndiv( "what", hptag )[0];
		
		hyplist.hyp.push_back( h );
	}
	
	return true;
}



// discard a hypothesis
bool DiscardHypothesis( robocluedo_msgs::DiscardHypothesis::Request& tag, robocluedo_msgs::DiscardHypothesis::Response& success )
{
	OUTLOG( "called service " << SERVICE_INTERFACE_WRONG_HYPOTHESIS );
	
	// remove the hypothesis from the database
	success.success = armor->RemoveHypothesis( tag.hypothesisTag );
	
	return true;
}



// save the ontology
bool ServiceBackupOntology( std_srvs::Trigger::Request& emptyrequest, std_srvs::Trigger::Response& success )
{
	OUTLOG( "called service " << SERVICE_INTERFACE_SAVE_ONTOLOGY );
	
	if( !ros::param::has( PARAM_ONTOLOGY_BACKUP_PATH ) )
	{
		success.success = false;
		success.message = SS( "unable to find the parameter" ) + SS( PARAM_ONTOLOGY_BACKUP_PATH );
		
		OUTLOG( "ERROR: " << success.message );
	}
	else
	{
		std::string save_path;
		ros::param::get( PARAM_ONTOLOGY_BACKUP_PATH, save_path );
		armor->SaveOntology( save_path );
		success.success = true;
		
		OUTLOG( "Ontology save here -> " << save_path );
	}
	
	return true;
}



/*
 * OPERAZIONI IMPLEMENTATE SU SERVIZIO:
 * 	registrare un hint e updte immediato
 * 	recuperare tutti i valori delle ipotesi consistenti
 *  elimina un'ipotesi dal sistema (scartata per via di una risposta negativa dall'oracolo)
 */
int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "cluedo_armor_interface" );
	ros::NodeHandle nh;
	
	// percorso della oltology da parameter server
	std::string ontology_file_path;
	if( !ros::param::has( ONTOLOGY_PARAM ) )
	{
		OUTLOG( "ERROR: parameter '" << ONTOLOGY_PARAM << "' not defined." );
		return 0;
	}
	ros::param::get( ONTOLOGY_PARAM, ontology_file_path );
	if( !fileExist( ontology_file_path ) )
	{
		OUTLOG( "ERROR: the file '" << ONTOLOGY_PARAM << "' doesn't exist." );
		return 0;
	}
	OUTLOG( "Ontology found! " << LOGSQUARE( ontology_file_path ) );
	
	// connessione ad aRMOR
	OUTLOG( "loading armor ..." );
	ArmorCluedo armorcluedo;
	if( !armorcluedo.Init( ontology_file_path ) || !armorcluedo.TestInterface( ) )
	{
		OUTLOG( "ERROR: unable to load aRMOR!" );
		return 0;
	}
	armor = &armorcluedo;
	OUTLOG( "OK!" );
	
	// servizio per registrare un hint dall'oracolo
	OUTLOG( "opening server " << LOGSQUARE( SERVICE_INTERFACE_ADD_HINT ) << " ..." );
	ros::ServiceServer srv_add_hint = nh.advertiseService( SERVICE_INTERFACE_ADD_HINT , ServiceAddHint );
	OUTLOG( "OK!" );
	
	// servizio per ottenere tutte le ipotesi consistenti
	OUTLOG( "opening server " << LOGSQUARE( SERVICE_INTERFACE_FIND_CONSISTENT_HYP ) << " ..." );
	ros::ServiceServer srv_find_cons_hyp = nh.advertiseService( SERVICE_INTERFACE_FIND_CONSISTENT_HYP, ServiceFindConsistentHypotheses );
	OUTLOG( "OK!" );
	
	// servizio per scartare ipotesi
	OUTLOG( "opening server " << LOGSQUARE( SERVICE_INTERFACE_WRONG_HYPOTHESIS ) << " ..." );
	ros::ServiceServer srv_wrong_hyp = nh.advertiseService( SERVICE_INTERFACE_WRONG_HYPOTHESIS, DiscardHypothesis );
	OUTLOG( "OK!" );
	
	// servizio per scaricare la ontology su file
	OUTLOG( "opening server " << LOGSQUARE( SERVICE_INTERFACE_SAVE_ONTOLOGY ) << " ..." );
	ros::ServiceServer srv_backup = nh.advertiseService( SERVICE_INTERFACE_SAVE_ONTOLOGY, ServiceBackupOntology );
	OUTLOG( "OK!" );
	
	// spin
	OUTLOG( "ready!" );
	ros::spin( );
	
	return 0;
}
