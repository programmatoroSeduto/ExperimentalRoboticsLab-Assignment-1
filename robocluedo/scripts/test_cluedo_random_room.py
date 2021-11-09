#! /usr/bin/env python

"""! @file test_cluedo_random_room.py

@brief testing the node cluedo_random_room.cpp
 
@authors Francesco Ganci (S4143910)
@version v1.0

Simple test for the target request in cluedo_random_room.cpp: perform the 
request 50 times. 

@see test_cluedo_random_room.launch launch file for the test

"""

import rospy
from robocluedo_msgs.srv import RandomRoom, RandomRoomRequest, RandomRoomResponse


test_name = "test_cluedo_random_room"

client_name_random_room = "/random_room"
client_random_room = None

nrooms = 50



def perform_tests( ):
	global client_random_room
	global nrooms
	
	roomslist = list( )
	for room_idx in range( nrooms ):
		rospy.loginfo( "[%s] room number (%d) -> %s", 
			test_name,
			room_idx+1,
			client_random_room( ).room )




def main( ):
	# global 
	
	# altre operazioni utili prima di iniziare i test...
	
	perform_tests( )




def on_shut_msg( ):
	rospy.loginfo( "[%s] closing...", test_name )




if __name__ == "__main__":
	rospy.init_node( test_name )
	
	rospy.loginfo( "[%s] INIT", test_name )
	rospy.on_shutdown( on_shut_msg )
	
	rospy.loginfo( "[%s] asking for service [%s] ...", test_name, client_name_random_room )
	rospy.wait_for_service( client_name_random_room )
	client_random_room = rospy.ServiceProxy( client_name_random_room, RandomRoom )
	rospy.loginfo( "[%s] OK!", test_name )
	
	main( )
