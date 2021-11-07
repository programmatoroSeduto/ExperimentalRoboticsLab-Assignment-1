#ifndef __H_ARMOR_CLUEDO_H__
#define __H_ARMOR_CLUEDO_H__

#include "armor_tools/armor_tools.h"

#include <vector>
#include <iostream>
#include <string>




/********************************************//**
 *  
 * \brief additional services for aRMOR
 * \extends ArmorTools
 * 
 * ... mode details ...
 * 
 * 
 ***********************************************/
class ArmorCluedo : public ArmorTools
{
public:
	
	/********************************************//**
	 *  
	 * \brief class constructor of ArmorTools
	 * 
	 * ... more details
	 * 
	 * @param debugmode enable or not debug mode
	 * 
	 ***********************************************/
	ArmorCluedo( bool debugmode = ARMOR_DEFAULT_DEBUGMODE );
	
	
	/// class destructor
	~ArmorCluedo( );
	
	
	/********************************************//**
	 *  
	 * \brief initizalize the interface
	 * 
	 * ... more details
	 * 
	 * @param ontologyPath the path of the OWL file
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool Init( std::string ontologyPath );
	
	
	
	// ======== individuals and classes
	//    methods for adding individuals to the ontology
	
	/********************************************//**
	 *  
	 * \brief add an individual to the database
	 * 
	 * ... more details
	 * 
	 * @param indivname
	 * @param classname
	 * @param makeDisjoint
	 * 
	 * @returns success or not
	 * 
	 ***********************************************/
	bool AddIndiv( std::string indivname, std::string classname, bool makeDisjoint = true );
	
	
	/********************************************//**
	 *  
	 * \brief get the class of a given individual
	 * 
	 * ... more details
	 * 
	 * @param indivname the individual to be "classified"
	 * @param deep use deep search?
	 * 
	 * @returns a vector of classes (zero if the indiv. doesn't exist)
	 * 
	 ***********************************************/
	std::vector<std::string> GetClassOfIndiv( std::string indivname, bool deep );
	
	
	/********************************************//**
	 *  
	 * \brief find the individuals belonging to a class
	 * 
	 * ... more details
	 * 
	 * @param classname the class
	 * 
	 * @returns a vector of individals inside the class (eventually empty)
	 * 
	 ***********************************************/
	std::vector<std::string> GetIndivOfClass( std::string classname );
	
	
	/********************************************//**
	 *  
	 * \brief check if an individual exists
	 * 
	 * ... more details
	 * 
	 * @param indivname the name of the individual
	 * 
	 * @returns if the individual exists or not
	 * 
	 ***********************************************/
	bool ExistsIndiv( std::string indivname );
	
	
	
	// ======== properties
	//    methods for checking and adding properties between individuals
	
	/********************************************//**
	 *  
	 * \brief set a property true
	 * 
	 * ... more details
	 * 
	 * @param prop the property
	 * @param Aelem domain
	 * @param Belem image
	 * 
	 * @returns if the individual exists or not
	 * 
	 ***********************************************/
	bool SetObjectProperty( std::string prop, std::string Aelem, std::string Belem );
	
	
	/********************************************//**
	 *  
	 * \brief get the values of a property related to a gven individual
	 * 
	 * ... more details
	 * 
	 * @param prop the property
	 * @param indivname the name of the individual
	 * 
	 * @returns the values of the property for the given individual
	 * 
	 ***********************************************/
	std::vector<std::string> GetValuedOfIndiv( std::string prop, std::string indivname );
	
	
	
	// ======== hypotheses
	//    methods for formulating and working with hypotheses
	
	/********************************************//**
	 *  
	 * \brief find all the complete hypotheses
	 * 
	 * ... more details
	 * 
	 * @returns a vector containing the tags of all the COMPLETE hyp.
	 * 
	 ***********************************************/
	std::vector<std::string> FindCompleteHypotheses( );
	
	
	/********************************************//**
	 *  
	 * \brief find all the inconsistent hypotheses
	 * 
	 * ... more details
	 * 
	 * @returns a vector containing the tags of all the INCONSISTENT hyp.
	 * 
	 ***********************************************/
	std::vector<std::string> FindInconsistentHypotheses( );
	
	
	/********************************************//**
	 *  
	 * \brief discard one hypothesis
	 * 
	 * ... more details
	 * 
	 * @param hypTag the hypothesis tag to discard
	 * 
	 * @returns if the hypothesis has been discarded or not
	 * 
	 ***********************************************/
	bool RemoveHypothesis( std::string hypTag );
	
	
	
	// ======== utilities
	//    general-purpose methods
	
	/********************************************//**
	 *  
	 * \brief rewrite a string like '<uri#value>' into 'value'
	 * 
	 * ... more details
	 * 
	 * @param raw the string to be filtered
	 * 
	 * @returns the value inside the string
	 * 
	 ***********************************************/
	std::string FilterValue( std::string raw );
	
	
	/********************************************//**
	 *  
	 * \brief filter all the strings inside the array
	 * 
	 * ... more details
	 * 
	 * @param itemList the array of strings to be filtered
	 * 
	 * @returns the arrayy containing the filtered strings
	 * 
	 ***********************************************/
	std::vector<std::string> FilterVector( std::vector<std::string>& itemlist );
	
private:
	// all the entities added
	std::vector<std::string> individuals;
	
	// removed hypotheses
	std::vector<std::string> DiscardHypotheses;
	
	// check if a string exists in one array
	bool ExistsItem( std::string item, const std::vector<std::string>& container );
	
	// track individual
	bool TrackIndiv( std::string indivname );
	
	// disjoint all the individuals
	void DisjointAllIndiv( std::string from );
	
	// search for a specific iterator on a vector
	//   it return vector::end() if the element is not contained
	std::vector<std::string>::iterator GetPositionOf( std::string tag, std::vector<std::string>& list );
};

#endif
