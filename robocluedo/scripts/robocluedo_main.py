#! /usr/bin/env python

"""! 

@file robocluedo_main.py
<div><b>ROS Node Name</b> 
     <ul><li>robocluedo_main</li></ul></div>
@brief The main FSM of RCL
 
@authors Francesco Ganci (S4143910)
@version v1.0

<b>Subscribers:</b> <br>
<ul>
	<li>
			<i>/topic</i> : file.msg <br>
			... description <br><br>
	</li>
</ul>

<b>Clients:</b> <br>
<ul>
    <li>
			<i>/go_to</i> : GoTo.srv <br>
			see the service \ref GoToCallback <br><br>
	</li>
    <li>
			<i>/random_room</i> : RandomRoom.srv <br>
			see the service \ref ChooseRoomRandom <br><br>
	</li>
    <li>
			<i>/check_solution</i> : checkSolution.srv <br>
			see the service \ref checkSolutionCallback <br><br>
	</li>
    <li>
			<i>/cluedo_armor/add_hint</i> : AddHint.srv <br>
			see the service \ref ServiceAddHint <br><br>
	</li>
    <li>
			<i>/cluedo_armor/find_consistent_h</i> : FindConsistentHypotheses.srv <br>
			see the service \ref ServiceFindConsistentHypotheses <br><br>
	</li>
    <li>
			<i>/cluedo_armor/wrong_hypothesis</i> : DiscardHypothesis.srv <br>
			see the service \ref DiscardHypothesis <br><br>
	</li>
    <li>
			<i>/cluedo_armor/backup</i> : <a href="http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html">std_srvs::Trigger</a> <br>
			see the service \ref ServiceBackupOntology <br><br>
	</li>
</ul>

<b>Description:</b> <br>
<p>
...description
</p>

"""

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
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse



# movement controller
## \private
client_name_go_to = "/go_to"
## \private
client_go_to = None

# random target generator
## \private
client_name_random_target = "/random_room"
## \private
client_random_target = None

# oracle channels
## \private
subscriber_name_hint = "/hint"
## \private
subscriber_hint = None
## \private
client_name_check_solution = "/check_solution"
## \private
client_check_solution = None

# cluedo_ARMOR interface clients
## \private
client_name_add_hint = "/cluedo_armor/add_hint"
## \private
client_add_hint = None
## \private
client_name_find_consistent_hyp = "/cluedo_armor/find_consistent_h"
## \private
client_find_consistent_hyp = None
## \private
client_name_wrong_hyp = "/cluedo_armor/wrong_hypothesis"
## \private
client_wrong_hyp = None
## \private
client_name_backup = "/cluedo_armor/backup"
## \private
client_backup = None

# global reference to the state machine
## \private
robot_sm = None;

## is there any message from the oracle?
hint_received = False
## the message from the oracle, if any
hint = None

## target to be 'consumed' by 'robocluedo_moving'
target = ""
## actual position of the robot (string, the room)
actual_position = ""

## is the charge ready?
charge_ready = False
## a possible solution to be confirmed or discarded, if any
#    @todo the buffer should be a list of hints instead of a single value
charge = None
## teh tag of the actual hypothesis
hypothesis_tag = ""



def listen_for_hints( data ):
	"""!
	@brief listen for a hint from the oracle, then buffer it
	
	... details
	
	@param data : Hint.msg the hint received from the Oracle
	"""
	global hint, hint_received
	
	hint_received = True
	hint = data



class robocluedo_random_target( smach.State ):
	"""!
	@brief implementation of the starting state <i>random_target</i>.
	
	... description
	
	<div>
		<b>FSM Evolution:</b> 
		<ul>
			<li>
			<u>move_robot</u> <i>to</i> robocluedo_moving <br>
			.... description <br><br>
			</li>
		</ul>
	</div>
	"""
	
	def __init__( self ):
		smach.State.__init__( self, outcomes=['move_robot'] )
	
	def execute( self, d ):
		global client_random_target
		global actual_position, target
		
		# ask for a random room
		tg = client_random_target( )
		
		# set the string to consume, then go to 'robocluedo_moving'
		target = tg.room
		if( actual_position == "" ):
			actual_position = tg.room
		
		rospy.loginfo( "[state:%s] actual position: %s | random target: %s", "random_target", actual_position, target )
		
		return 'move_robot'



class robocluedo_moving( smach.State ):
	"""!
	@brief implementation of the state <i>moving</i>.
	
	... description
	
	<div>
		<b>FSM Evolution:</b> 
		<ul>
			<li>
			<u>target_reached_no_charge</u> <i>to</i> robocluedo_listening_for_hints <br>
			.... description <br><br>
			</li>
			<li>
			<u>target_reached_ready_for_charge</u> <i>to</i> robocluedo_charge <br>
			.... description <br><br>
			</li>
		</ul>
	</div>
	"""
	
	def __init__( self ):
		smach.State.__init__( self, 
			outcomes=['target_reached_no_charge', 'target_reached_ready_for_charge'] )
	
	def execute( self, d ):
		global client_go_to
		global actual_position, target
		global charge_ready
		
		# reach the target
		client_go_to( target )
		actual_position = target
		target = ""
		
		rospy.loginfo( "[state:%s] reached position: %s ", "moving", actual_position )
		
		if charge_ready:
			return 'target_reached_ready_for_charge'
		else:
			return 'target_reached_no_charge'



class robocluedo_listening_for_hints( smach.State ):
	"""!
	@brief implementation of the state <i>listening_for_hints</i>.
	
	... description
	
	<div>
		<b>FSM Evolution:</b> 
		<ul>
			<li>
			<u>received_a_hint</u> <i>to</i> robocluedo_update_ontology <br>
			.... description <br><br>
			</li>
			<li>
			<u>no_hints</u> <i>to</i> robocluedo_reasoning <br>
			.... description <br><br>
			</li>
		</ul>
	</div>
	"""
	
	def __init__( self ):
		smach.State.__init__( self, outcomes=['received_a_hint', 'no_hints'] )
	
	def execute( self, d ):
		global hint_received
		
		if hint_received:
			rospy.loginfo( "[state:listening_for_hints] received a hint" )
			return 'received_a_hint'
		else:
			rospy.loginfo( "[state:listening_for_hints] no new hints" )
			return 'no_hints'



class robocluedo_update_ontology( smach.State ):
	"""!
	@brief implementation of the state <i>update_ontology</i>.
	
	... description
	
	<div>
		<b>FSM Evolution:</b> 
		<ul>
			<li>
			<u>done</u> <i>to</i> robocluedo_reasoning <br>
			.... description <br><br>
			</li>
		</ul>
	</div>
	"""
	
	def __init__( self ):
		smach.State.__init__( self, outcomes=['done'] )
	
	def execute( self, d ):
		global client_add_hint, client_backup
		global hint, hint_received
		
		rospy.loginfo( "[state:update_ontology] received hint: HP%d(%s:%s)", hint.HintID, hint.HintType, hint.HintContent  )
		
		# add the hint to the ontology
		hintreq = AddHintRequest( )
		hintreq.hypID = hint.HintID
		hintreq.property = hint.HintType
		hintreq.Aelem = "HP" + str(hint.HintID)
		hintreq.Belem = hint.HintContent
		client_add_hint( hintreq )
		
		# clear the buffer
		hint_received = False
		hint = None
		
		# make a backup of the ontology
		rospy.loginfo( "[state:update_ontology] writing backup of the ontology..." )
		client_backup( )
		rospy.loginfo( "[state:update_ontology] ontology saved" )
		
		return 'done'



class robocluedo_reasoning( smach.State ):
	"""!
	@brief implementation of the state <i>reasoning</i>.
	
	... description
	
	<div>
		<b>FSM Evolution:</b> 
		<ul>
			<li>
			<u>ready_for_charge</u> <i>to</i> robocluedo_charge <br>
			.... description <br><br>
			</li>
			<li>
			<u>not_in_the_right_place</u> <i>to</i> robocluedo_moving <br>
			.... description <br><br>
			</li>
			<li>
			<u>no_consistent_hp</u> <i>to</i> robocluedo_random_target <br>
			.... description <br><br>
			</li>
		</ul>
	</div>
	"""
	
	def __init__( self ):
		smach.State.__init__( self, outcomes=['ready_for_charge', 'not_in_the_right_place', 'no_consistent_hp'] )
	
	def execute( self, d ):
		global client_find_consistent_hyp
		global charge_ready, charge
		global actual_position, target
		
		# check for any consistent hypothesis
		hplist = ( client_find_consistent_hyp( ) ).hyp
		if ( len( hplist ) == 0 ):
			return 'no_consistent_hp'
		
		rospy.loginfo( "[state:reasoning] complete hypotheses found: %d", len( hplist ) )
		for h in hplist:
			rospy.loginfo( "[state:reasoning] %s(who:%s, where:%s, what:%s)", h.tag, h.who, h.where, h.what )
		
		# choose random one among the complete hypotheses
		to_evaluate = random.choice( hplist )
		
		rospy.loginfo( "[state:reasoning] selected hyp. with tag: %s | actual position: %s", to_evaluate.tag, actual_position )
		
		# store the hypothesis
		charge_ready = True
		charge = to_evaluate
		
		# choose the outcome
		if ( actual_position == charge.where ):
			return 'ready_for_charge'
		else:
			target = charge.where
			return 'not_in_the_right_place'



class robocluedo_charge( smach.State ):
	"""!
	@brief implementation of the state <i>charge</i>.
	
	... description
	
	<div>
		<b>FSM Evolution:</b> 
		<ul>
			<li>
			<u>case_solved</u> <i>to</i> <b>END STATE</b> <br>
			.... description <br><br>
			</li>
			<li>
			<u>wrong_hp</u> <i>to</i> robocluedo_listening_for_hints <br>
			.... description <br><br>
			</li>
		</ul>
	</div>
	"""
	
	def __init__( self ):
		smach.State.__init__( self, outcomes=['case_solved', 'wrong_hp'] )
	
	def execute( self, d ):
		global client_check_solution, client_wrong_hyp
		global charge, charge_ready
		
		# send the charge to the oracle 
		req = CheckSolutionRequest( )
		req.Who = charge.who
		req.What = charge.what
		req.Where = charge.where
		
		rospy.loginfo( "[state:charge] asking to the oracle: (who:%s, where:%s, what:%s)", charge.who, charge.where, charge.what )
		solution = client_check_solution( req )
		
		# success?
		if solution.MysterySolved:
			rospy.loginfo( "[state:charge] mystery solved!" )
			return 'case_solved'
		
		# explore again
		rospy.loginfo( "[state:charge] WRONG hypothesis with tag: %s", charge.tag )
		client_wrong_hyp( charge.tag )
		charge_ready = False
		charge = None
		
		rospy.loginfo( "[state:charge] exploring again..." )
		return 'wrong_hp'



# create the state machine
def create_state_machine( ):
	"""!
	@brief define the state machine of the robot before starting
	
	... details
	
	"""
	global robot_sm
	
	robot_sm = smach.StateMachine( outcomes=[ 'mystery_solved' ] )
	
	with robot_sm:
		smach.StateMachine.add( 'robocluedo_random_target', 
			robocluedo_random_target( ), 
			transitions={'move_robot':'robocluedo_moving'} )
			
		smach.StateMachine.add( 'robocluedo_moving', 
			robocluedo_moving( ), 
			transitions={'target_reached_no_charge':'robocluedo_listening_for_hints', 
				'target_reached_ready_for_charge':'robocluedo_charge'} )
		
		smach.StateMachine.add( 'robocluedo_listening_for_hints', 
			robocluedo_listening_for_hints( ), 
			transitions={ 'received_a_hint':'robocluedo_update_ontology', 
				'no_hints':'robocluedo_reasoning' } )
		
		smach.StateMachine.add( 'robocluedo_update_ontology', 
			robocluedo_update_ontology( ), 
			transitions={ 'done':'robocluedo_reasoning' } )
		
		smach.StateMachine.add( 'robocluedo_reasoning', 
			robocluedo_reasoning( ), 
			transitions={ 'ready_for_charge':'robocluedo_charge', 
				'not_in_the_right_place':'robocluedo_moving', 
				'no_consistent_hp':'robocluedo_random_target' } )
		
		smach.StateMachine.add( 'robocluedo_charge', 
			robocluedo_charge( ), 
			transitions={ 'case_solved':'mystery_solved', 
				'wrong_hp':'robocluedo_listening_for_hints' } )




# message on shutdown
## @private
def on_shut( ):
	rospy.loginfo( "[robocluedo_main] closing..." )




if __name__ == "__main__":
	rospy.init_node( "robocluedo_main" )
	rospy.on_shutdown( on_shut )
	
	
	# movement controller
	rospy.loginfo( "[robocluedo_main] calling service %s ...", client_name_go_to )
	rospy.wait_for_service( client_name_go_to )
	client_go_to = rospy.ServiceProxy( client_name_go_to, GoTo )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	
	# random target generator
	rospy.loginfo( "[robocluedo_main] calling service %s ...", client_name_random_target )
	rospy.wait_for_service( client_name_random_target )
	client_random_target = rospy.ServiceProxy( client_name_random_target, RandomRoom )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	
	# oracle
	rospy.loginfo( "[robocluedo_main] subscribing to %s ...", subscriber_name_hint )
	subscriber_hint = rospy.Subscriber( subscriber_name_hint, Hint, listen_for_hints )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	rospy.loginfo( "[robocluedo_main] calling service %s ...", client_name_check_solution )
	rospy.wait_for_service( client_name_check_solution )
	client_check_solution = rospy.ServiceProxy( client_name_check_solution, CheckSolution )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	
	# aRMOR interfaces
	rospy.loginfo( "[robocluedo_main] calling service %s ...", client_name_add_hint )
	rospy.wait_for_service( client_name_add_hint )
	client_add_hint = rospy.ServiceProxy( client_name_add_hint, AddHint )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	rospy.loginfo( "[robocluedo_main] calling service %s ...", client_name_find_consistent_hyp )
	rospy.wait_for_service( client_name_find_consistent_hyp )
	client_find_consistent_hyp = rospy.ServiceProxy( client_name_find_consistent_hyp, FindConsistentHypotheses )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	rospy.loginfo( "[robocluedo_main] calling service %s ...", client_name_wrong_hyp )
	rospy.wait_for_service( client_name_wrong_hyp )
	client_wrong_hyp = rospy.ServiceProxy( client_name_wrong_hyp, DiscardHypothesis )
	rospy.loginfo( "[robocluedo_main] OK" )
	
	rospy.loginfo( "[%s] asking for service [%s] ...", "robocluedo_main", client_name_backup )
	rospy.wait_for_service( client_name_backup )
	client_backup = rospy.ServiceProxy( client_name_backup, Trigger )
	rospy.loginfo( "[%s] OK!", "robocluedo_main" )
	
	
	# create the state machine
	create_state_machine( )
	
	## @private
	sis = smach_ros.IntrospectionServer('server_name', robot_sm, '/SM_ROOT')
	sis.start()
	
	# start the system
	rospy.loginfo( "[robocluedo_main] starting the FSM..." )
	robot_sm.execute( )
	rospy.loginfo( "[robocluedo_main] elementary, Whatson. " )
	
	# rospy.spin()
	sis.stop()
