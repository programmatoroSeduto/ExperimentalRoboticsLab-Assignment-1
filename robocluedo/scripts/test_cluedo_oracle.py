#! /usr/bin/env python

"""! @file test_cluedo_oracle.py

@brief testing the node cluedo_oracle.cpp
 
@authors Francesco Ganci (S4143910)
@version v1.0

Simple test for the hint request in cluedo_oracle.cpp: perform the 
request 25 times. 

@see test_cluedo_oracle.launch launch file for the test

"""

import rospy
from robocluedo_msgs.srv import CheckSolution, CheckSolutionRequest, CheckSolutionResponse
from std_msgs.msg import Empty
from robocluedo_msgs.msg import Hint




client_name_check_solution = "/check_solution"
client_check_solution = None

publisher_name_hint_sig = "/hint_signal"
publisher_hint_sig = None

subscriber_name_hint = "/hint"




test_name = "test_cluedo_oracle"


def callback_hint( hint ):
	global hint_idx
	global received
	
	rospy.loginfo( "[%s] (number %d) received: HP%d(%s:%s)", test_name, hint_idx, hint.HintID, hint.HintType, hint.HintContent )




hint_idx = 0
received = False
def perform_tests( ):
	global publisher_name_hint_sig
	global client_check_solution
	global hint_idx
	
	# rospy.loginfo( "number 1: hint system" )
	
	for hintno in range( 25 ):
		rospy.loginfo( "[%s] sending signal %d ...", test_name, hint_idx )
		publisher_hint_sig.publish( Empty( ) )
		hint_idx = hint_idx + 1
		rospy.sleep( rospy.Duration( 0.2 ) )
	
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
	
	main( )
