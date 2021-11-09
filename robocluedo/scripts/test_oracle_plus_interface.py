#! /usr/bin/env python

"""! @file test_oracle_plus_interface.py

@brief testing the nodes cluedo_oracle.cpp and cluedo_armor_interface.cpp
 
@authors Francesco Ganci (S4143910)
@version v1.0

Each time an hint arrives, store it into the ontology. 

@see test_cluedo_oracle_plus_interface.launch launch file for the test

"""

import rospy
from robocluedo_msgs.srv import CheckSolution, CheckSolutionRequest, CheckSolutionResponse
from std_msgs.msg import Empty
from robocluedo_msgs.msg import Hint
from robocluedo_msgs.srv import AddHint, AddHintRequest, AddHintResponse
from robocluedo_msgs.srv import FindConsistentHypotheses, FindConsistentHypothesesRequest, FindConsistentHypothesesResponse
from robocluedo_msgs.msg import Hypothesis
from robocluedo_msgs.srv import DiscardHypothesis, DiscardHypothesisRequest, DiscardHypothesisResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse




client_name_check_solution = "/check_solution"
client_check_solution = None

publisher_name_hint_sig = "/hint_signal"
publisher_hint_sig = None

subscriber_name_hint = "/hint"

client_name_add_hint = "/cluedo_armor/add_hint"
client_add_hint = None

client_name_find_consistent_h = "/cluedo_armor/find_consistent_h"
client_find_consistent_h = None

client_name_wrong_h = "/cluedo_armor/wrong_hypothesis"
client_wrong_h = None

client_name_backup = "/cluedo_armor/backup"
client_backup = None




test_name = "test_oracle_plus_interface"


def make_hint_to_ontology( id, prop, val ):
	srv_hint = AddHintRequest( )
	
	''' add hint request
	# the numeric ID of the hint
	int32 hypID

	# fields of the property
	string property
	string Aelem
	string Belem
	'''
	
	srv_hint.hypID = id
	srv_hint.property = prop
	srv_hint.Aelem = "HP" + str( id )
	srv_hint.Belem = val
	
	return srv_hint


def callback_hint( hint ):
	global hint_idx
	global received
	global client_add_hint
	
	rospy.loginfo( "[%s] (number %d) received: HP%d(%s:%s)", test_name, hint_idx, hint.HintID, hint.HintType, hint.HintContent )
	
	rospy.loginfo( "[%s] adding hint to the ontology... ", test_name )
	client_add_hint( make_hint_to_ontology( hint.HintID, hint.HintType, hint.HintContent ) )
	
	hplist = client_find_consistent_h( ).hyp
	if len(hplist) > 0:
		rospy.loginfo( "[%s] consistent hypotheses right now: ", test_name )
		rospy.loginfo( "[%s] received size : %d ", test_name, len(hplist) )
		for i in range( len(hplist) ):
			rospy.loginfo( "[%s] -> %s(where:%s, what:%s, who:%s)", test_name, hplist[i].tag, hplist[i].where, hplist[i].what, hplist[i].who )




hint_idx = 0
received = False
def perform_tests( ):
	global publisher_name_hint_sig
	global client_check_solution
	global hint_idx
	
	# rospy.loginfo( "number 1: hint system" )
	
	for hintno in range( 50 ):
		rospy.loginfo( "[%s] sending signal %d ...", test_name, hint_idx )
		publisher_hint_sig.publish( Empty( ) )
		hint_idx = hint_idx + 1
		rospy.sleep( rospy.Duration( 0.1 ) )
	
	rospy.loginfo( "[%s] saving the ontology...", test_name )
	client_backup( )




def main( ):
	# global 
	
	# altre operazioni utili prima di iniziare i test...
	
	perform_tests( )




def on_shut_msg( ):
	rospy.loginfo( "[%s] closing...", test_name )




if __name__ == "__main__":
	rospy.init_node( test_name )
	rospy.on_shutdown( on_shut_msg )
	
	# client : check solution
	rospy.loginfo( "[%s] asking for service [%s] ...", test_name, client_name_check_solution )
	rospy.wait_for_service( client_name_check_solution )
	client_check_solution = rospy.ServiceProxy( client_name_check_solution, CheckSolution )
	rospy.loginfo( "[%s] OK!", test_name )
	
	# publisher : hint signal
	rospy.loginfo( "[%s] opening publisher to topic [%s] ...", test_name, publisher_name_hint_sig )
	publisher_hint_sig = rospy.Publisher( publisher_name_hint_sig, Empty, queue_size=1 )
	rospy.loginfo( "[%s] OK!", test_name )
	
	# subscriber : hint
	rospy.loginfo( "[%s] subscribing to topic [%s] ...", test_name, subscriber_name_hint )
	# rospy.wait_for_message( subscriber_name_hint, Hint )
	rospy.Subscriber( subscriber_name_hint, Hint, callback_hint )
	rospy.loginfo( "[%s] OK!", test_name )
	
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
