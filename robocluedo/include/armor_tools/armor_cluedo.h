#ifndef __H_ARMOR_CLUEDO_H__
#define __H_ARMOR_CLUEDO_H__

#include "armor_tools/armor_tools.h"
#include <vector>
#include <iostream>
#include <string>

class ArmorCluedo : public ArmorTools
{
public:
	
	// default constructor
	//    instantiate, connect and load
	ArmorCluedo( bool debugmode = ARMOR_DEFAULT_DEBUGMODE );
	
	// destructor
	~ArmorCluedo( );
	
	// init the interface
	bool Init( std::string ontologyPath );
	
	
	// ======== individuals and classes
	//    methods for adding individuals to the ontology
	
	// add an individual
	bool AddIndiv( std::string indivname, std::string classname, bool makeDisjoint = true );
	
	// check the class of an individual
	std::vector<std::string> GetClassOfIndiv( std::string indivname, bool deep );
	
	// find the individuals belonging to a class
	std::vector<std::string> GetIndivOfClass( std::string classname );
	
	// check if an individual exists
	bool ExistsIndiv( std::string indivname );
	
	
	// ======== properties
	//    methods for checking and adding properties between individuals
	
	// set a property true
	bool SetObjectProperty( std::string prop, std::string Aelem, std::string Belem );
	
	// get the values of a property related to a gven individual
	std::vector<std::string> GetValuedOfIndiv( std::string prop, std::string indivname );
	
	
	// ======== hypotheses
	//    methods for formulating and working with hypotheses
	
	// find all the complete hypotheses
	std::vector<std::string> FindCompleteHypotheses( );
	
	// find all the inconsistent hypotheses
	std::vector<std::string> FindInconsistentHypotheses( );
	
	// remove one hyothesis
	bool RemoveHypothesis( std::string hypTag );
	
	// check if an hypothesis satisfies a property
	// bool HypothesisSatisfiedProperty( std::string hypTag, std::string prop, std::string value );
	
	
	// ======== utilities
	//    general-purpose methods
	
	// rewrite a string like '<uri#value>' into 'value'
	std::string FilterValue( std::string raw );
	
	// filter all the strings inside the array
	std::vector<std::string> FilterVector( std::vector<std::string>& itemlist );
	
private:
	// all the entities added
	std::vector<std::string> individuals;
	
	// the last queried hypothesis
	std::string LastHypothesis = "";
	std::string LastHyp_where = "";
	std::string LastHyp_who = "";
	std::string LastHyp_what = "";
	
	// check if a string exists in one array
	bool ExistsItem( std::string item, const std::vector<std::string>& container );
	
	// track individual
	bool TrackIndiv( std::string indivname );
	
	// disjoint all the individuals
	void DisjointAllIndiv( std::string from );
};

#endif
