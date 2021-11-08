#! /usr/bin/env python

import rospy


test_name = "testname"

def perform_tests( ):
	# global 
	
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
	
	# services, clients, publshers, subscribers
	
	main( )
