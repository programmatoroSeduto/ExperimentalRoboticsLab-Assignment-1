#ifndef __H_ARMOR_TOOLS_H__
#define __H_ARMOR_TOOLS_H__

#include "ros/ros.h"
#include "armor_msgs/QueryItem.h"
#include "armor_msgs/ArmorDirective.h"
#include "armor_msgs/ArmorDirectiveReq.h"
#include "armor_msgs/ArmorDirectiveRes.h"

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <fstream>

#define ARMOR_DEFAULT_URI "http://www.emarolab.it/cluedo-ontology"
#define ARMOR_DEFAULT_REASONER "PELLET"
#define ARMOR_DEFAULT_CLIENT "armor_client"
#define ARMOR_DEFAULT_REFERENCE "cluedo"
#define ARMOR_DEFAULT_TIMEOUT 5.00
#define ARMOR_DEFAULT_DEBUGMODE true

#define ARMOR_SERVICE_SINGLE_REQUEST "/armor_interface_srv"
#define ARMOR_SERVICE_MULTIPLE_REQUESTS "/armor_interface_serialized_srv"

#define ARMOR_CLASS_LABEL "[armor_tools]"
#define ARMOR_INFO( msg ) if( this->DebugMode ) ROS_INFO_STREAM( ARMOR_CLASS_LABEL << " " << msg )
#define ARMOR_ERR( msg ) if( this->DebugMode ) ROS_WARN_STREAM( ARMOR_CLASS_LABEL << " ERROR: " << msg )
#define ARMOR_CHECK_INTERFACE( returnval ) if( !IsLoadedInterface || !ArmorSrv.exists( ) ) { ARMOR_ERR( "bad interface!" ); return returnval; }
#define ARMOR_RES( msg ) msg.response.armor_response
#define ARMOR_RES_QUERY( msg ) msg.response.armor_response.queried_objects

#define SS( this_string ) std::string( this_string )
#define SSS( this_thing ) std::to_string( this_thing )
#define BOOL_TO_CSTR( booleanvalue ) ( booleanvalue ? "true" : "false" )
#define LOGSQUARE( str ) "[" << str << "] "




/********************************************//**
 *  
 * \brief A base client for aRMOR. 
 * 
 * ...more details...
 * 
 * 
 ***********************************************/
class ArmorTools
{
public:
	/********************************************//**
	 *  
	 * \brief first class constructor. 
	 * 
	 * ... more details
	 * 
	 * @param client client
	 * @param reference reference
	 * 
	 ***********************************************/
	ArmorTools(
		std::string client = ARMOR_DEFAULT_CLIENT,
		std::string reference = ARMOR_DEFAULT_REFERENCE,
		bool dbmode = false
	);
	
	
	/********************************************//**
	 *  
	 * \brief first class constructor. 
	 * 
	 * ... more details
	 * 
	 * @param dbmode debug mode?
	 * 
	 ***********************************************/
	ArmorTools( bool dbmode );
	
	
	/// class destructor
	~ArmorTools();
	
	
	/********************************************//**
	 *  
	 * \brief quick creation of an aRMOR request
	 * 
	 * ... more details
	 * 
	 * @param command (mandatory) the main command
	 * @param first_spec (optional) the first specifier
	 * @param second_spec (optional) the second specifier
	 * @param arg (optional) (from 1 to 5) the arguments of the request
	 * 
	 * @return the aRMOR service request. 
	 * 
	 ***********************************************/
	armor_msgs::ArmorDirective GetRequest(
		std::string command,
		std::string first_spec = "",
		std::string second_spec = "",
		std::string arg1 = "",
		std::string arg2 = "",
		std::string arg3 = "",
		std::string arg4 = "",
		std::string arg5 = ""
	);
	
	
	/********************************************//**
	 *  
	 * \brief send a command to aRMOR. 
	 * 
	 * ... more details
	 * 
	 * @param data reference to the request to send
	 * 
	 * @returns if the service was called or not
	 * 
	 ***********************************************/
	bool CallArmor( armor_msgs::ArmorDirective& data );
	
	
	/********************************************//**
	 *  
	 * \brief load the ontology from file. 
	 * 
	 * ... more details
	 * 
	 * @param path  
	 * @param uri 
	 * @param manipulationFlag 
	 * @param reasoner
	 * @param bufferend_reasoner
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool LoadOntology( 
		std::string path, 
		std::string uri = ARMOR_DEFAULT_URI,
		bool manipulationFlag = true,
		std::string reasoner = ARMOR_DEFAULT_REASONER,
		bool buffered_reasoner = true 
	);
	
	
	/********************************************//**
	 *  
	 * \brief open a connection with the aRMOR service. 
	 * 
	 * ... more details
	 * 
	 * @param timeout 
	 * 
	 * @returns success or not0
	 * 
	 ***********************************************/
	bool Connect( float timeout = ARMOR_DEFAULT_TIMEOUT );
	
	
	/********************************************//**
	 *  
	 * \brief connect to the server and load the ontology from file. 
	 * 
	 * ... more details
	 * 
	 * @param path  
	 * @param uri 
	 * @param manipulationFlag 
	 * @param reasoner
	 * @param bufferend_reasoner
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool ConnectAndLoad( 
		std::string path, 
		std::string uri = ARMOR_DEFAULT_URI,
		bool manipulationFlag = true,
		std::string reasoner = ARMOR_DEFAULT_REASONER,
		bool buffered_reasoner = true 
	);
	
	
	/********************************************//**
	 *  
	 * \brief save the ontology on file
	 * 
	 * ...
	 * 
	 * @param path where to save the OWL file
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool SaveOntology( std::string path );
	
	
	/********************************************//**
	 *  
	 * \brief send the command REASON
	 * 
	 * ...
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool UpdateOntology( );
	
	
	/********************************************//**
	 *  
	 * \brief print a request to the screen.
	 * 
	 * ...
	 * 
	 * @param d the aRMOR service
	 * 
	 ***********************************************/
	void PrintRequest( armor_msgs::ArmorDirective& d );
	
	
	/********************************************//**
	 *  
	 * \brief print the response to the screen.
	 * 
	 * ...
	 * 
	 * @param d the aRMOR service
	 * 
	 ***********************************************/
	void PrintResponse( armor_msgs::ArmorDirective& d );
	
	
	/********************************************//**
	 *  
	 * \brief toggle the debug mode
	 * 
	 * ...
	 * 
	 ***********************************************/
	void SwitchDebugMode( );
	
	
	/********************************************//**
	 *  
	 * \brief err code referred to the last call 
	 * 
	 * ...
	 * 
	 * @returns the last error code
	 * 
	 ***********************************************/
	int GetLastErrorCode( );
	
	
	/********************************************//**
	 *  
	 * \brief last err description
	 * 
	 * ...
	 * 
	 * @returns the last error description
	 * 
	 ***********************************************/
	std::string GetLastErrorDescription( );
	
	
	/********************************************//**
	 *  
	 * \brief check the 'success' flag referred to
	 * 			the last aRMOR call
	 * 
	 * ...
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool Success( );
	
	
	/********************************************//**
	 *  
	 * \brief check if the ontology was loaded or not
	 * 
	 * ...
	 * 
	 * @returns loaded or not
	 * 
	 ***********************************************/
	bool LoadedOntology( );
	
	
	/********************************************//**
	 *  
	 * \brief check the status of the interface
	 * 
	 * ...
	 * 
	 * @returns valid inferface or not
	 * 
	 ***********************************************/
	bool TestInterface( );
	
	
	/********************************************//**
	 *  
	 * \brief fill in a command and send it to aRMOR
	 * 
	 * ...
	 * 
	 * @param command (mandatory) the main command
	 * @param first_spec (optional) the first specifier
	 * @param second_spec (optional) the second specifier
	 * @param arg (optional) (from 1 to 5) the arguments of the request
	 * @param printResponse print the request before calling the service
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool SendCommand(
		std::string command,
		std::string first_spec = "",
		std::string second_spec = "",
		std::string arg1 = "",
		std::string arg2 = "",
		std::string arg3 = "",
		std::string arg4 = "",
		std::string arg5 = "",
		bool printRequest = false
	);
	
	
	/********************************************//**
	 *  
	 * \brief get a reference to the last response
	 * 
	 * ...
	 * 
	 * @returns reference to the last response
	 * 
	 ***********************************************/
	armor_msgs::ArmorDirectiveRes& GetLastRes( );
	
	
	/********************************************//**
	 *  
	 * \brief get a reference to the last request
	 * 
	 * ...
	 * 
	 * @returns last sent request to aRMOR
	 * 
	 ***********************************************/
	armor_msgs::ArmorDirectiveReq& GetLastReq( );
	
	
	/********************************************//**
	 *  
	 * \brief print the last response
	 * 
	 * ...
	 * 
	 ***********************************************/
	void PrintLastRes( );
	
	
	/********************************************//**
	 *  
	 * \brief print the last request
	 * 
	 * ...
	 * 
	 ***********************************************/
	void PrintLastReq( );
	
protected:

	/// debug mode enabled or not
	bool DebugMode = false;
	
private:
	// aRMOR service
	ros::ServiceClient ArmorSrv;
	
	// client name
	std::string ClientName = ARMOR_DEFAULT_CLIENT;
	
	// reference name
	std::string ReferenceName = ARMOR_DEFAULT_REFERENCE;
	
	// uri
	std::string uri = ARMOR_DEFAULT_URI;
	
	// interface loaded? 
	//    l'ultimo comando load ha avuto successo?
	bool IsLoadedInterface = false;
	
	// last response from the server
	armor_msgs::ArmorDirectiveRes LastRes;
	
	// last request sent to the server
	armor_msgs::ArmorDirectiveReq LastReq;
	
	// controlla se il dato file esiste
	bool FileExist( std::string path );
};


#endif
