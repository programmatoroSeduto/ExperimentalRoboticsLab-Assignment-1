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
#define ARMOR_SERVICE_SINGLE_REQUEST "/armor_interface_srv"
#define ARMOR_SERVICE_MULTIPLE_REQUESTS "/armor_interface_serialized_srv"
#define ARMOR_DEFAULT_TIMEOUT 5.00
#define ARMOR_DEFAULT_DEBUGMODE true

#define ARMOR_CLASS_LABEL "[armor_tools]"
#define ARMOR_INFO( msg ) if( this->DebugMode ) ROS_INFO_STREAM( ARMOR_CLASS_LABEL << " " << msg )
#define ARMOR_ERR( msg ) if( this->DebugMode ) ROS_WARN_STREAM( ARMOR_CLASS_LABEL << " ERROR: " << msg )
#define ARMOR_CHECK_INTERFACE( returnval ) if( !IsLoadedInterface || !ArmorSrv.exists( ) ) { ARMOR_ERR( "bad interface!" ); return returnval; }
#define ARMOR_RES( msg ) msg.response.armor_response
#define ARMOR_RES_QUERY( msg ) msg.response.armor_response.queried_objects
#define LOGSQUARE( str ) "[" << str << "] "

#define SS( this_string ) std::string( this_string )
#define SSS( this_thing ) std::to_string( this_thing )
#define BOOL_TO_CSTR( booleanvalue ) ( booleanvalue ? "true" : "false" )


// tools for low level communication with aRMOR
/*
 * INIZIALIZZAZIONE DELLA CLASSE
 * - costruttore
 *   - eventualmente set di client e reference
 *   - meglio attivare la debug mode per avere i messaggi a schermo
 * - connect
 * */
class ArmorTools
{
public:
	// costruttore
	ArmorTools(
		std::string client = ARMOR_DEFAULT_CLIENT,
		std::string reference = ARMOR_DEFAULT_REFERENCE,
		bool dbmode = false
	);
	
	// costruttore con dbmode
	ArmorTools( bool dbmode );
	
	// distruttore
	~ArmorTools();
	
	// set del client_name
	void SetClient( std::string client );
	
	// get de client name
	std::string GetClient(  );
	
	// set per reference_name
	void SetReference( std::string reference );
	
	// get per reference_name
	std::string GetReference( );
	
	// metodo veloce per scrivere una richiesta
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
	
	// esegue la chiamata a server
	bool CallArmor( armor_msgs::ArmorDirective& data );
	
	// il comando load
	bool LoadOntology( 
		std::string path, 
		std::string uri = ARMOR_DEFAULT_URI,
		bool manipulationFlag = true,
		std::string reasoner = ARMOR_DEFAULT_REASONER,
		bool buffered_reasoner = true 
	);
	
	// il comando connect
	bool Connect( float timeout = ARMOR_DEFAULT_TIMEOUT );
	
	// apri il server e carica (LOAD) la ontology
	bool ConnectAndLoad( 
		std::string path, 
		std::string uri = ARMOR_DEFAULT_URI,
		bool manipulationFlag = true,
		std::string reasoner = ARMOR_DEFAULT_REASONER,
		bool buffered_reasoner = true 
	);
	
	// imposta un service esternamente, ma solo se non è stata fatta la connessione
	bool SetArmorServiceClient( ros::ServiceClient& cl );
	
	// salva (SAVE) la ontology su file
	bool SaveOntology( std::string path );
	
	// operazione di reason
	bool UpdateOntology( );
	
	// operazione di lettura delle operazioni dal buffer
	bool ApplyCommands( );
	
	// metodi di stampa dei pacchetti: request
	void PrintRequest( armor_msgs::ArmorDirective& d );
	
	// e response
	void PrintResponse( armor_msgs::ArmorDirective& d );
	
	// debug mode
	void SwitchDebugMode( );
	
	// ultimo codice d'errore
	int GetLastErrorCode( );
	
	// ultima descrizione dell'errore
	std::string GetLastErrorDescription( );
	
	// bool dall'ultima operazione
	bool Success( );
	
	// check sul caricamento della ontology
	bool LoadedOntology( );
	
	// testa la validità dell'interfaccia
	bool TestInterface( );
	
	// chiamata diretta con istruzione
	bool SendCommand(
		std::string command,
		std::string first_spec = "",
		std::string second_spec = "",
		std::string arg1 = "",
		std::string arg2 = "",
		std::string arg3 = "",
		std::string arg4 = "",
		std::string arg5 = ""
	);
	
	// reference all'ultima risposta
	armor_msgs::ArmorDirectiveRes& GetLastRes( );
	
	// stampa l'ultimo pacchetto ricevuto da aRMOR
	void PrintLastRes( );
	
protected:

	// debug mode? (print all the messages to the screen?)
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
	
	// controlla se il dato file esiste
	bool FileExist( std::string path );
};


#endif
