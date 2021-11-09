#! /usr/bin/env python

"""! @file test_cluedo_armor_interface.py

@brief testing the node cluedo_armor_interface.cpp
 
@authors Francesco Ganci (S4143910)
@version v1.0

This node implements a simple test for the node cluedo_armor_interface.cpp :
try a simple reasoning process using the interface. 

@see test_cluedo_armor_interface.launch launch file for the test

"""

import rospy
from robocluedo_msgs.srv import AddHint, AddHintRequest, AddHintResponse
from robocluedo_msgs.srv import FindConsistentHypotheses, FindConsistentHypothesesRequest, FindConsistentHypothesesResponse
from robocluedo_msgs.msg import Hypothesis
from robocluedo_msgs.srv import DiscardHypothesis, DiscardHypothesisRequest, DiscardHypothesisResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

client_name_add_hint = "/cluedo_armor/add_hint"
client_add_hint = None

client_name_find_consistent_h = "/cluedo_armor/find_consistent_h"
client_find_consistent_h = None

client_name_wrong_h = "/cluedo_armor/wrong_hypothesis"
client_wrong_h = None

client_name_backup = "/cluedo_armor/backup"
client_backup = None


test_name = "test_cluedo_armor_interface"

def perform_tests( ):
	global client_add_hint
	global client_find_consistent_h
	global client_wrong_h
	global client_backup
	
	rospy.loginfo( "[%s] formulating hypothesis : %s(where:%s, what:%s, who:%s)", test_name, "HP1", "study", "knife", "mark" )
	client_add_hint( 1, "where", "HP1", "study" )
	client_add_hint( 1, "what", "HP1", "knife" )
	client_add_hint( 1, "who", "HP1", "mark" )
	
	rospy.loginfo( "[%s] expected 1 consistent hypothesis", test_name )
	rospy.loginfo( "[%s] asking for consistent hypotheses... ", test_name )
	hplist = client_find_consistent_h( ).hyp
	rospy.loginfo( "[%s] received size : %d ", test_name, len(hplist) )
	rospy.loginfo( "[%s] -> %s(where:%s, what:%s, who:%s)", test_name, hplist[0].tag, hplist[0].where, hplist[0].what, hplist[0].who )
	
	rospy.loginfo( "[%s] discarding the hypothesis... ", test_name )
	wrong_h_msg = DiscardHypothesisRequest( )
	wrong_h_msg.hypothesisTag = hplist[0].tag
	client_wrong_h( wrong_h_msg )
	rospy.loginfo( "[%s] asking again for consistent hypotheses... ", test_name )
	hplist = client_find_consistent_h( ).hyp
	rospy.loginfo( "[%s] received size (expected 0) : %d ", test_name, len(hplist) )
	
	rospy.loginfo( "[%s] saving ontology... ", test_name )
	client_backup( )
	rospy.loginfo( "[%s] OK!", test_name )




def main( ):
	# global 
	
	# altre operazioni utili prima di iniziare i test...
	
	perform_tests( )




def on_shut_msg( ):
	rospy.loginfo( "[%s] closing...", test_name )




if __name__ == "__main__":
	rospy.init_node( test_name )
	rospy.on_shutdown( on_shut_msg )
	
	# service : add hint
	rospy.loginfo( "[%s] asking for service [%s] ...", test_name, client_name_add_hint )
	rospy.wait_for_service( client_name_add_hint )
	client_add_hint = rospy.ServiceProxy( client_name_add_hint, AddHint )
	rospy.loginfo( "[%s] OK!", test_name )
	
	# service : find consistent hypotheses
	rospy.loginfo( "[%s] asking for service [%s] ...", test_name, client_name_find_consistent_h )
	rospy.wait_for_service( client_name_find_consistent_h )
	client_find_consistent_h = rospy.ServiceProxy( client_name_find_consistent_h, FindConsistentHypotheses )
	rospy.loginfo( "[%s] OK!", test_name )
	
	# service : wrong hypothesis
	rospy.loginfo( "[%s] asking for service [%s] ...", test_name, client_name_wrong_h )
	rospy.wait_for_service( client_name_wrong_h )
	client_wrong_h = rospy.ServiceProxy( client_name_wrong_h, DiscardHypothesis )
	rospy.loginfo( "[%s] OK!", test_name )
	
	# service : backup
	rospy.loginfo( "[%s] asking for service [%s] ...", test_name, client_name_backup )
	rospy.wait_for_service( client_name_backup )
	client_backup = rospy.ServiceProxy( client_name_backup, Trigger )
	rospy.loginfo( "[%s] OK!", test_name )
	
	main( )
