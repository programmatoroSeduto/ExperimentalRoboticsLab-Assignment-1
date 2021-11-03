#! /usr/bin/env python

import rospy
import smach, smach_ros
import random

from robocluedo_msgs.srv import GoTo, GoToRequest, GoToResponse
from robocluedo_msgs.msg import Hint
from robocluedo_msgs.srv import CheckSolution, CheckSolutionRequest, CheckSolutionResponse
from robocluedo_msgs.srv import AddHint, AddHintRequest, AddHintResponse
from robocluedo_msgs.srv import FindConsistentHypotheses, FindConsistentHypothesesRequest, FindConsistentHypothesesResponse
from robocluedo_msgs.srv import DiscardHypothesis, DiscardHypothesisRequest, DiscardHypothesisResponse
from robocluedo_msgs.srv import RandomRoom, RandomRoomRequest, RandomRoomResponse 

# movement controller
client_name_go_to = "/go_to"
client_go_to = None

# random target generator
client_name_random_target = "/random_room"
client_random_target = None

# oracle channels
subscriber_name_hint = "/hint"
subscriber_hint = None
client_name_check_solution = "/check_solution"
client_check_solution = None

# cluedo_ARMOR interface clients
client_name_add_hint = "/cluedo_armor/add_hint"
client_add_hint = None
client_name_find_consistent_hyp = "/cluedo_armor/find_consistent_h"
client_find_consistent_hyp = None
client_name_wrong_hyp = "/cluedo_armor/wrong_hypothesis"
client_wrong_hyp = None

# global reference to the state machine
robot_sm = None;



# listen for a hint from the oracle, and buffer it
def listen_for_hints( data ):
	pass




class robocluedo_random_target( smach.State ):
	def __init__( self ):
		smach.State.__init__( self, outcomes=['move_robot'],  
			output_keys=['destination'] )
	
	def execute( self, d ):
		global client_random_target, robot_sm
		
		# ask for a random room
		tg = client_random_target( )
		
		# set the string to consume, then go to 'robocluedo_moving'
		d.destination = tg.room
		if( robot_sm.userdata.actual_position == "" )
			robot_sm.userdata.actual_position = tg.room
		
		rospy.loginfo( "[robocluedo_random_target] actual position: %s | random target: %s", robot_sm.userdata.actual_position, obot_sm.userdata.target )
		
		return 'move_robot'



class robocluedo_moving( smach.State ):
	def __init__( self ):
		smach.State.__init__( self, 
			outcomes=['target_reached_no_charge', 'target_reached_ready_for_charge'], 
			input_keys=['target', 'ready_for_charge'], 
			output_keys=['destination', 'actual_position'] )
	
	def execute( self, d ):
		global client_go_to, robot_sm
		
		# reach the target
		client_go_to( d.target )
		d.actual_position = d.target
		d.destination = ""
		
		rospy.loginfo( "[robocluedo_moving] reached position: %s ", robot_sm.userdata.actual_position )
		
		if d.ready_for_charge:
			return 'target_reached_ready_for_charge'
		else:
			return 'target_reached_no_charge'



class robocluedo_listening_for_hints( smach.State ):
	def __init__( self ):
		smach.State.__init__( self, outcomes=['received_a_hint', 'no_hints'], 
			input_keys=['hint_received'] )
	
	def execute( self, d ):
		if d.hint_received:
			rospy.loginfo( "[robocluedo_listening_for_hints] received a hint" )
			return 'received_a_hint'
		else:
			rospy.loginfo( "[robocluedo_listening_for_hints] no new hints" )
			return 'no_hints'



class robocluedo_update_ontology( smach.State ):
	def __init__( self ):
		smach.State.__init__( self, outcomes=['done'], 
			input_keys=['buffered_hint'], 
			output_keys=['buffer', 'buffer_flag'] )
	
	def execute( self, d ):
		global client_add_hint, robot_sm
		
		# add the hint to the ontology
		hint = AddHintRequest( )
		hint.HintID = buffered_hint.HintID
		hint.property = buffered_hint.HintType
		hint.Aelem = "HP" + hint.HintID
		hint-Belem = buffered_hint.HintContent
		client_add_hint( hint )
		
		# clear the buffer
		d.buffer_flag = False
		d.buffer = None
		
		rospy.loginfo( "[robocluedo_update_ontology] received hint: (%s, %s):%s", hint.Aelem, hint.Aelem, hint.Belem, hint.property  )
		
		return 'done'



class robocluedo_reasoning( smach.State ):
	def __init__( self ):
		smach.State.__init__( self, outcomes=['ready_for_charge', 'not_in_the_right_place', 'no_consistent_hp'], 
			input_keys=['actual_position'], 
			output_keys=['charge_ready', 'charge', 'destination'] )
	
	def execute( self, d ):
		global client_find_consistent_hyp, robot_sm
		
		# check for any consistent hypothesis
		hplist = ( client_find_consistent_hyp( ) ).hyp
		if ( len( hyp ) == 0 ):
			return 'no_consistent_hp'
		
		rospy.loginfo( "[robocluedo_reasoning] complete hypotheses found: %d", len( hyp ) )
		for h in hyp:
			rospy.loginfo( "[robocluedo_reasoning] %s(who:%s, where:%, what:%s)", h.tag, h.who, h.where, h.what )
		
		# choose random one among the complete hypotheses
		to_evaluate = random.choice( hplist )
		
		rospy.loginfo( "[robocluedo_reasoning] selected: %s | actual position: %s", to_evaluate.tag, d.actual_position )
		
		# store the hypothesis
		charge_ready = True
		charge = to_evaluate
		
		# choose the outcome
		if d.actual_position == charge.where
			return 'ready_for_charge'
		else
			d.destination = charge.where
			return 'not_in_the_right_place'



class robocluedo_charge( smach.State ):
	def __init__( self ):
		smach.State.__init__( self, outcomes=['case_solved', 'wrong_hp'], 
			input_keys=['charge_to_confirm'], 
			output_keys=['charge', 'charge_ready'] )
	
	def execute( self, d ):
		global client_check_solution, client_wrong_hyp
		
		# send the charge to the oracle 
		req = CheckSolutionRequest( )
		req.who = d.charge_to_confirm.who
		req.what = d.charge_to_confirm.what
		req.where = d.charge_to_confirm.where
		
		rospy.loginfo( "[robocluedo_charge] asking to the oracle: (who:%s, where:%, what:%s)", d.charge_to_confirm.who, d.charge_to_confirm.where, d.charge_to_confirm.what )
		solution = client_check_solution( req )
		
		# success?
		if solution.MysterySolved:
			rospy.loginfo( "[robocluedo_charge] mystery solved!" )
			return 'case_solved'
		
		# explore again
		client_wrong_hyp( d.charge_to_confirm.tag )
		d.charge_ready = False
		d.charge = None
		
		rospy.loginfo( "[robocluedo_charge] exploring again..." )
		return 'wrong_hp'



# create the state machine
def create_state_machine( ):
	global robot_sm
	
	robot_sm = smach.StateMachine( outcomes=[ 'mystery_solved' ] )
	
	# buffer for the messages from the oracle
	robot_sm.userdata.hint_received = False
	robot_sm.userdata.hint = None
	
	# target to be 'consumed' by 'robocluedo_moving'
	robot_sm.userdata.target = ""
	robot_sm.userdata.actual_position = ""
	
	# hypohesis for the charge
	robot_sm.userdata.charge_ready = False
	robot_sm.userdata.charge = None
	robot_sm.userdata.hypothesis_tag = ""
	
	with robot_sm:
		robot_sm.StateMachine.add( 'robocluedo_random_target', 
			robocluedo_random_target( ), 
			transitions={'move_robot':'robocluedo_moving'}, 
			remapping={'destination':'target'} )
			
		robot_sm.StateMachine.add( 'robocluedo_moving', 
			robocluedo_moving( ), 
			transitions={'target_reached_no_charge':'robocluedo_listening_for_hints', 
				'target_reached_ready_for_charge':'robocluedo_charge'}, 
			remapping={'target':'target', 
				'ready_for_charge':'charge_ready',
				'destination':'target', 
				'actual_position':'actual_position'} )
		
		robot_sm.StateMachine.add( 'robocluedo_listening_for_hints', 
			robocluedo_listening_for_hints( ), 
			transitions={ 'received_a_hint':'robocluedo_update_ontology', 
				'no_hints':'robocluedo_reasoning' }, 
			remapping={ 'hint_received':'hint_received' } )
		
		robot_sm.StateMachine.add( 'robocluedo_update_ontology', 
			robocluedo_update_ontology( ), 
			transitions={ 'done':'robocluedo_reasoning' }, 
			remapping={ 'buffered_hint':'hint',
				'buffer':'hint',
				'buffer_flag':'hint_received'} )
		
		robot_sm.StateMachine.add( 'robocluedo_reasoning', 
			robocluedo_reasoning( ), 
			transitions={ 'ready_for_charge':'robocluedo_charge', 
				'not_in_the_right_place':'robocluedo_moving', 
				'no_consistent_hp':'robocluedo_random_target' }, 
			remapping={ 'actual_position':'actual_position',
				'charge_ready':'charge_ready', 
				'charge':'charge', 
				'destination':'target'} )
		
		robot_sm.StateMachine.add( 'robocluedo_charge', 
			robocluedo_charge( ), 
			transitions={ 'case_solved':'mystery_solved', 
				'wrong_hp':'robocluedo_listening_for_hints' }, 
			remapping={ 'charge_to_confirm':'charge',
				'charge':'charge', 
				'charge_ready':'charge_ready'} )




# message on shutdown
def on_shut( ):
	rospy.loginfo( "closing..." )




if __name__ == "__main__":
	rospy.init_node( "robocluedo_main" )
	
	# movement controller
	rospy.loginfo( "calling service %s ...", client_name_go_to )
	rospy.wait_for_service( client_name_go_to )
	client_go_to = rospy.ServiceProxy( client_name_go_to, GoTo )
	rospy.loginfo( "OK" )
	
	# random target generator
	rospy.loginfo( "calling service %s ...", client_name_random_target )
	rospy.wait_for_service( client_name_random_target )
	client_random_target = rospy.ServiceProxy( client_name_random_target, RandomRoom )
	rospy.loginfo( "OK" )
	
	# oracle
	rospy.loginfo( "subscribing to %s ...", subscriber_name_hint )
	rospy.wait_for_message( subscriber_name_hint )
	subscriber_hint = rospy.subscriber( subscriber_name_hint, Hint, listen_for_hints )
	rospy.loginfo( "OK" )
	
	rospy.loginfo( "calling service %s ...", client_name_check_solution )
	rospy.wait_for_service( client_name_check_solution )
	client_check_solution = rospy.ServiceProxy( client_name_check_solution, CheckSolution )
	rospy.loginfo( "OK" )
	
	# aRMOR interfaces
	rospy.loginfo( "calling service %s ...", client_name_add_hint )
	rospy.wait_for_service( client_name_add_hint )
	client_add_hint = rospy.ServiceProxy( client_name_add_hint, AddHint )
	rospy.loginfo( "OK" )
	
	rospy.loginfo( "calling service %s ...", client_name_find_consistent_hyp )
	rospy.wait_for_service( client_name_find_consistent_hyp )
	client_find_consistent_hyp = rospy.ServiceProxy( client_name_find_consistent_hyp, FindConsistentHypotheses )
	rospy.loginfo( "OK" )
	
	rospy.loginfo( "calling service %s ...", client_name_wrong_hyp )
	rospy.wait_for_service( client_name_wrong_hyp )
	client_wrong_hyp = rospy.ServiceProxy( client_name_wrong_hyp, DiscardHypothesis )
	rospy.loginfo( "OK" )
	
	# create the state machine
	create_state_machine( )
	
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	
	# start the system
	rospy.loginfo( "starting the FSM..." )
	robot_sm.execute( )
	
	rospy.spin()
	sis.stop()
