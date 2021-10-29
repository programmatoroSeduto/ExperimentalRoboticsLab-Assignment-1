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
	
	ArmorTools( bool dbmode );
	
	// distruttore
	~ArmorTools();
	
	// set e get per client_name
	void SetClient( std::string client );
	std::string GetClient(  );
	
	// set e get per reference_name
	void SetReference( std::string reference );
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
	
	// aggiungi un individual ad una classe
	bool AddIndiviualToClass( std::string individual, std::string classname );
	
	// controlla se un individual esiste
	bool Exists( std::string individual );
	
	// controlla a che classi appartiene un certo individual
	std::vector<std::string> ClassOf( std::string individual );
	
	// ritorna tutti gli individuals appartenenti ad una certa classe
	std::vector<std::string> GetIndivFromClass( std::string classname );
	
	// aggiungi una proprietà binaria
	bool AddObjectProperty( std::string property, std::string Aelem, std::string Belem );
	
	// operazione di reason
	bool UpdateOntology( );
	
	// operazione di lettura delle operazioni dal buffer
	bool ApplyCommands( );
	
	// controlla se un'ipotesi è consistente o da scartare
	bool IsHypothesisConsistent( std::string hypTag );
	
	// trova tutte le ipotesi consistenti
	std::vector<std::string> GetConsistentHypotheses( );
	
	// controlla se una certa proprietà è verificata per un certo individual
	bool ValueOfProperty( std::string property, std::string Aelem );
	
	// metodi di stampa dei pacchetti: request
	void PrintRequest( armor_msgs::ArmorDirective& d );
	
	// e response
	void PrintResponse( armor_msgs::ArmorDirective& d );
	
	// debug mode
	void SwitchDebugMode( );
	
	// riguardo l'ultimo errore
	int GetLastErrorCode( );
	std::string GetLastErrorDescription( );
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
	
	// reference all'ultimo messaggio inviato
	const armor_msgs::ArmorDirectiveRes& GetLastRes( );
	
	// stampa l'ultimo pacchetto ricevuto da aRMOR
	void PrintLastRes( );
	
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
	
	// debug mode? (print all the messages to the screen?)
	bool DebugMode = false;
	
	// last response from the server
	armor_msgs::ArmorDirectiveRes LastRes;
	
	// controlla se il dato file esiste
	bool FileExist( std::string path );
	
	// rimuovi le parti inutili dai valori di ritorno da aRMOR
	//    da una stringa del tipo "<uri#value>" ad una del tipo "value"
	//    se la stringa non inizia col simbolo '<', funzione identità
	std::string FilterQueryValue( std::string data );
	
	// elenco delle classi
	std::vector<std::string> classes;
	
	// elenco degli individuals
	std::vector<std::string> individuals;
};


#endif
