
#include "armor_tools/armor_tools.h"




// === metodi pubblici

// costruttore
ArmorTools::ArmorTools(
		std::string client,
		std::string reference,
		bool dbmode
	):
	ClientName( client ),
	ReferenceName( reference ),
	DebugMode( dbmode )
{
	// stub init of the last response
	LastRes.success = true;
	LastRes.timeout = false;
	LastRes.exit_code = 0;
	LastRes.error_description = "";
	LastRes.is_consistent = true;
	// LastRes.queried_objects = std::vector<std::string>( );
	// LastRes.sparql_queried_objects = std::vector<armor_msgs::QueryItem>( );
}



// altro costruttore, solo flag debugmode
ArmorTools::ArmorTools( bool dbmode ):
	ClientName( ARMOR_DEFAULT_CLIENT ),
	ReferenceName( ARMOR_DEFAULT_REFERENCE ),
	DebugMode( dbmode )
{
	// stub init of the last response
	LastRes.success = true;
	LastRes.timeout = false;
	LastRes.exit_code = 0;
	LastRes.error_description = "";
	LastRes.is_consistent = true;
	// LastRes.queried_objects = std::vector<std::string>( );
	// LastRes.sparql_queried_objects = std::vector<armor_msgs::QueryItem>( );
}


// distruttore
ArmorTools::~ArmorTools()
{
	// ...
}



// set e get per client_name
void ArmorTools::SetClient( std::string client )
{
	this->ClientName = client;
}



std::string ArmorTools::GetClient(  )
{
	return this->ClientName;
}



// set e get per reference_name
void ArmorTools::SetReference( std::string reference )
{
	this->ReferenceName = reference;
}



std::string ArmorTools::GetReference( )
{
	return this->ReferenceName;
}



// metodo veloce per scrivere una richiesta
armor_msgs::ArmorDirective ArmorTools::GetRequest(
		std::string command,
		std::string first_spec,
		std::string second_spec,
		std::string arg1,
		std::string arg2,
		std::string arg3,
		std::string arg4,
		std::string arg5
	)
{
	armor_msgs::ArmorDirective adsrv;
	armor_msgs::ArmorDirectiveReq ad;
	
	ad.client_name = this->ClientName;
	ad.reference_name = this->ReferenceName;
	
	ad.command = command;
	ad.primary_command_spec = first_spec;
	ad.secondary_command_spec = second_spec;
	
	ad.args = std::vector<std::string>();
	ad.args.push_back( arg1 );
	ad.args.push_back( arg2 );
	ad.args.push_back( arg3 );
	ad.args.push_back( arg4 );
	ad.args.push_back( arg5 );
	
	adsrv.request.armor_request = ad;
	
	return adsrv;
}



// esegue la chiamata a server
bool ArmorTools::CallArmor( armor_msgs::ArmorDirective& data )
{
	// if there is no server, don't call it
	if( !this->ArmorSrv.exists() )
	{
		ARMOR_ERR( "no service found!" );
		return false;
	}
	
	// try to call it
	if( !this->ArmorSrv.call( data ) )
	{
		ARMOR_ERR( "unable to call ArmorService" << LOGSQUARE( ARMOR_SERVICE_SINGLE_REQUEST ) );
		return false;
	}
	
	// save the last received response
	this->LastRes = data.response.armor_response;
	
	return true;
}



// il comando load
bool ArmorTools::LoadOntology( 
		std::string path, 
		std::string uri,
		bool manipulationFlag,
		std::string reasoner,
		bool buffered_reasoner
	)
{
	// if there is no server, don't call it
	if( !this->ArmorSrv.exists() )
	{
		ARMOR_ERR( "no service found!" );
		return false;
	}
	
	// check if the file exists
	if( !this->FileExist( path ) )
	{
		ARMOR_ERR( "OWL file not found. " << LOGSQUARE( path ) );
		return false;
	}
	
	// prepare the command load
	armor_msgs::ArmorDirective load_cmd = GetRequest( 
		"LOAD", "FILE", "", 
		path, uri, BOOL_TO_CSTR( manipulationFlag ), reasoner, BOOL_TO_CSTR( buffered_reasoner ) 
	);
	
	// call the service
	if( !CallArmor( load_cmd ) ) 
	{
		ARMOR_ERR( "unable to call the service!" );
		return false;
	}
	
	return (IsLoadedInterface = load_cmd.response.armor_response.success);
}



// il comando connect
bool ArmorTools::Connect( float timeout )
{
	// check if the service exists
	if( this->ArmorSrv.exists() )
	{
		ARMOR_INFO( "service already connected!" );
		return false;
	}
	
	// connect to the service
	ros::NodeHandle nh;
	ARMOR_INFO( "Requiring client " << LOGSQUARE( ARMOR_SERVICE_SINGLE_REQUEST ) << "..." );
	this->ArmorSrv = nh.serviceClient<armor_msgs::ArmorDirective>( ARMOR_SERVICE_SINGLE_REQUEST );
	if( !this->ArmorSrv.waitForExistence( ros::Duration( timeout ) ) )
	{
		ARMOR_ERR( "ERROR: unable to contact the server - timeout expired (" << timeout << "s) " );
		return false;
	}
	ARMOR_INFO( "-> OK" );
	
	return true;
}



// apri il server e carica (LOAD) la ontology
bool ArmorTools::ConnectAndLoad( 
		std::string path, 
		std::string uri,
		bool manipulationFlag,
		std::string reasoner,
		bool buffered_reasoner
	)
{
	if ( Connect( ) && 
			LoadOntology( path, uri, manipulationFlag, reasoner, buffered_reasoner ) )
		return true;
	else
		return false;
}



// imposta un service esternamente, ma solo se non è stata fatta la connessione
bool ArmorTools::SetArmorServiceClient( ros::ServiceClient& cl )
{
	// check if the client is correctly set
	if( !cl.exists() )
	{
		ARMOR_ERR( "provided a not valid serviceClient." );
		return false;
	}
	
	// set the client
	this->ArmorSrv = cl;
	
	return true;
}



// salva (SAVE) la ontology su file
bool ArmorTools::SaveOntology( std::string path )
{
	ARMOR_CHECK_INTERFACE( false );
	
	// try to call the service
	auto srvdata = GetRequest( "SAVE", "INFERENCE", "", path );
	if( !CallArmor( srvdata ) ) 
		return false;
	
	return srvdata.response.armor_response.success;
}


	
// operazione di reason
bool ArmorTools::UpdateOntology( )
{
	ARMOR_CHECK_INTERFACE( false );
	
	auto srvdata = GetRequest( "REASON" );
	if( !CallArmor( srvdata ) ) 
		return false;
	
	return ARMOR_RES( srvdata ).success;
}


	
// operazione di lettura delle operazioni dal buffer
bool ArmorTools::ApplyCommands( )
{
	ARMOR_CHECK_INTERFACE( false );
	
	auto srvdata = GetRequest( "APPLY" );
	if( !CallArmor( srvdata ) ) 
		return false;
	
	return ARMOR_RES( srvdata ).success;
}


	
// metodi di stampa dei pacchetti
void ArmorTools::PrintRequest( armor_msgs::ArmorDirective& d )
{
	std::string str = SS("   Print Request: \n");
	armor_msgs::ArmorDirectiveReq r = d.request.armor_request;
	
	str += SS("\tclient_name : ") + SS( r.client_name ) + SS("\n");
	str += SS("\treference_name : ") + SS( r.reference_name ) + SS("\n");
	str += SS("\tcommand : ") + SS( r.command ) + SS("\n");
	str += SS("\tprimary_command_spec : ") + SS( r.primary_command_spec ) + SS("\n");
	str += SS("\tsecondary_command_spec : ") + SS( r.secondary_command_spec ) + SS("\n");
	
	str += SS("\targs : [");
	for( std::string arg : r.args )
		str += SS( arg ) + SS(" ");
	str += SS( "]" );
		
	str += SS( "\n\t---" );
	
	ROS_INFO_STREAM( str );
}


void ArmorTools::PrintResponse( armor_msgs::ArmorDirective& d )
{
	std::string str = SS("   Print Response: \n");
	armor_msgs::ArmorDirectiveRes r = d.response.armor_response;
	
	str += SS("\tsuccess : ") + ( r.success ? SS("true") : SS("false") ) + SS("\n");
	str += SS("\ttimeout : ") + ( r.timeout ? SS("true") : SS("false") ) + SS("\n");
	str += SS("\texit_code : ") + SSS(r.exit_code) + SS("\n");
	str += SS("\terror_description : ") + SS(r.error_description) + SS("\n");
	str += SS("\tis_consistent : ") + ( r.is_consistent ? SS("true") : SS("false") ) + SS("\n");
	
	str += SS("\tqueried_objects :\n");
	for( std::string s : r.queried_objects )
		str += SS("\t-\t") + s + SS("\n");
	
	str += SS("\tsparql_queried_objects : \n");
	for( armor_msgs::QueryItem s : r.sparql_queried_objects )
		str += SS("\t-\t") + SS("key: ") + SS( s.key ) + SS( " | value: " ) + SS( s.value ) + SS("\n");
	
	str += SS( "\t---" );
	
	ROS_INFO_STREAM( str );
}


	
// debug mode
void ArmorTools::SwitchDebugMode( )
{
	this->DebugMode = !this->DebugMode;
}



int ArmorTools::GetLastErrorCode( )
{
	return this->LastRes.exit_code;
}



std::string ArmorTools::GetLastErrorDescription( )
{
	return SS( this->LastRes.error_description );
}



bool ArmorTools::Success( )
{
	return this->LastRes.success;
}



// check sul caricamento della ontology	
bool ArmorTools::LoadedOntology( )
{
	return IsLoadedInterface;
}



// testa la validità dell'interfaccia
bool ArmorTools::TestInterface( )
{
	ARMOR_CHECK_INTERFACE( false );
	return true;
}



// invio diretto di un comando ad aRMOR
bool ArmorTools::SendCommand(
		std::string command,
		std::string first_spec,
		std::string second_spec,
		std::string arg1,
		std::string arg2,
		std::string arg3,
		std::string arg4,
		std::string arg5
	)
{
	ARMOR_CHECK_INTERFACE( false );
	
	// build the message to send
	auto srvdata = GetRequest( command, first_spec, second_spec, arg1, arg2, arg3, arg4, arg5 );
	
	// and send it
	return CallArmor( srvdata );
}



// reference all'ultimo messaggio inviato
armor_msgs::ArmorDirectiveRes& ArmorTools::GetLastRes( )
{
	armor_msgs::ArmorDirectiveRes& res = this->LastRes;
	return res;
}



// stampa l'ultimo pacchetto ricevuto da aRMOR
void ArmorTools::PrintLastRes( )
{
	armor_msgs::ArmorDirective armordata;
	armordata.response.armor_response = this->LastRes;
	PrintResponse( armordata );
}















// === metodi privati

// controlla se il dato file esiste
bool ArmorTools::FileExist( std::string path )
{
	return (std::ifstream(path)).good();
}
