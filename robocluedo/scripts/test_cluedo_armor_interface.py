#! /usr/bin/env python

'''
rospy.loginfo( "[%s] asking for service [%s] ...", test_name,  )
rospy.wait_for_service( client_name_random_room )
client_random_room = rospy.ServiceProxy( client_name_random_room, RandomRoom )
rospy.loginfo( "[%s] OK!", test_name )

'''

import rospy
from robocluedo_msgs.srv import AddHint, AddHintRequest, AddHintResponse
from robocluedo_msgs.srv import FindConsistentHypotheses, FindConsistentHypothesesRequest, FindConsistentHypothesesResponse
from robocluedo_msgs.msg import Hypothesis
from robocluedo_msgs.srv import DiscardHypothesis, DiscardHypothesisRequest, DiscardHypothesisResponse

client_name_add_hint = "/cluedo_armor/add_hint"
client_add_hint = None

client_name_find_consistent_h = "/cluedo_armor/find_consistent_h"
client_find_consistent_h = None

client_name_wrong_h = "/cluedo_armor/wrong_hypothesis"
client_wrong_h = None


test_name = "testname"

def perform_tests( ):
	global client_add_hint
	global client_find_consistent_h
	global client_wrong_h
	
	rospy.loginfo( "[%s] formulating hypothesis : %s(where:%s, what:%s, who:%s)", test_name, "HP1", "study", "knife", "mark" )
	client_add_hint( 1, "where", "HP1", "study" )
	#client_add_hint( 1, "what", "HP1", "knife" )
	#client_add_hint( 1, "who", "HP1", "mark" )
	
	#rospy.loginfo( "[%s] expected 1 consistent hypothesis", test_name )
	#rospy.loginfo( "[%s] asking for consistent hypotheses... ", test_name )
	#hplist = client_find_consistent_h( ).hyp
	#rospy.loginfo( "[%s] received size : %d ", test_name, len(hplist) )
	
	pass




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
	
	main( )
