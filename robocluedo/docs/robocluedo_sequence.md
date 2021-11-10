\page robocluedo_sequence RoboCLuedo -- SEQ -- UML Sequence Diagrams
\brief The UML Sequence Diagrams on a practical case

# UML Sequence Diagrams - A practical example

In order to show you how the project works I think it is more useful to present you a possible log from the project, using this test configuration:

- the item lists are reduced to only two elements per list. 
  - PLACE: bathroom, study. 
  - PERSON: plum,  mustard. 
  - WEAPON: revolver, dagger. 
- the maximum number of IDs is 4

You can find both the log and the end ontology from the folder [`/logs/test_20211109`](https://github.com/programmatoroSeduto/ExperimentalRoboticsLab-Assignment-1/tree/main/robocluedo/logs/test_20211109). In this example, the program was launched using the main launch file, that suppresses some logs, for example the ones from the cluedo_armor_interface. Only the logs belonging to the oracle and the robocluedo_main node are visible, since I think they're the most relevant nodes in the architecture. 

Before starting, all the secundary components start running, and the config files are loaded. 

```
[ INFO] [1636487503.092018000]: [cluedo_oracle] reading from fiile [/root/ws_experimental_robotics_lab_assignment_1/src/ExperimentalRoboticsLab-Assignment-1/robocluedo/config/cluedo_items/cluedo_who.txt] 
[ INFO] [1636487503.092848100]: [cluedo_oracle] line[1] READ [plum] 
[ INFO] [1636487503.092901500]: [cluedo_oracle] line[2] READ [mustard] 
[ INFO] [1636487503.092950600]: [cluedo_oracle] closing file ...
[ INFO] [1636487503.093006600]: [cluedo_oracle] reading from fiile [/root/ws_experimental_robotics_lab_assignment_1/src/ExperimentalRoboticsLab-Assignment-1/robocluedo/config/cluedo_items/cluedo_where.txt] 
[ INFO] [1636487503.093062800]: [cluedo_oracle] line[1] READ [study] 
[ INFO] [1636487503.093111100]: [cluedo_oracle] line[2] READ [bathroom] 
[ INFO] [1636487503.093157200]: [cluedo_oracle] closing file ...
[ INFO] [1636487503.093205400]: [cluedo_oracle] reading from fiile [/root/ws_experimental_robotics_lab_assignment_1/src/ExperimentalRoboticsLab-Assignment-1/robocluedo/config/cluedo_items/cluedo_what.txt] 
[ INFO] [1636487503.093260300]: [cluedo_oracle] line[1] READ [dagger] 
[ INFO] [1636487503.093305700]: [cluedo_oracle] line[2] READ [revolver] 
[ INFO] [1636487503.093352000]: [cluedo_oracle] closing file ...
```

In particular the Oracle prepares the case and chooses the solution. In this case, the generated solution has ID zero, and we know what is from the log:

```
		...
[ INFO] [1636487503.093892400]: [cluedo_oracle] found a max number of hypotheses: 4
[ INFO] [1636487503.093940200]: [cluedo_oracle] case generation started 
[ INFO] [1636487503.093992500]: [cluedo_oracle] the solution is (where:study, who:mustard, what:dagger)
[ INFO] [1636487503.094038100]: [cluedo_oracle] the solution has ID:0
[ INFO] [1636487503.094091800]: [cluedo_oracle] hints generation finished. Generated: 19 hints
```

The main node waits for the services and the topics, until all the nodes are ready. 

```
		...
[ INFO] [1636487503.094138700]: [cluedo_oracle] subscribing to the topic [/hint_signal] ...
[ INFO] [1636487503.095897700]: [cluedo_oracle] subscribing to the topic [/hint_signal] ... OK
[ INFO] [1636487503.095975500]: [cluedo_oracle] Creating publisher [/hint] ...
[ INFO] [1636487503.097834100]: [cluedo_oracle] Creating publisher [/hint] ... OK
[ INFO] [1636487503.097998500]: [cluedo_oracle] Advertising service [/check_solution] ...
[ INFO] [1636487503.099094600]: [cluedo_oracle] Advertising service [/check_solution] ... OK
[ INFO] [1636487503.099201000]: [cluedo_oracle] ready!
log4j:WARN No appenders could be found for logger (org.ros.internal.node.client.Registrar).
log4j:WARN Please initialize the log4j system properly.
log4j:WARN See http://logging.apache.org/log4j/1.2/faq.html#noconfig for more info.
[INFO] [1636487503.802970]: [robocluedo_main] calling service /go_to ...
[INFO] [1636487503.804739]: [robocluedo_main] OK
[INFO] [1636487503.805391]: [robocluedo_main] calling service /random_room ...
[INFO] [1636487503.806928]: [robocluedo_main] OK
[INFO] [1636487503.807626]: [robocluedo_main] subscribing to /hint ...
[INFO] [1636487503.809300]: [robocluedo_main] OK
[INFO] [1636487503.810581]: [robocluedo_main] calling service /check_solution ...
[INFO] [1636487503.812781]: [robocluedo_main] OK
[INFO] [1636487503.813683]: [robocluedo_main] calling service /cluedo_armor/add_hint ...
[INFO] [1636487504.719390]: [robocluedo_main] OK
[INFO] [1636487504.720344]: [robocluedo_main] calling service /cluedo_armor/find_consistent_h ...
[INFO] [1636487504.722123]: [robocluedo_main] OK
[INFO] [1636487504.722945]: [robocluedo_main] calling service /cluedo_armor/wrong_hypothesis ...
[INFO] [1636487504.724579]: [robocluedo_main] OK
[INFO] [1636487504.725292]: [robocluedo_main] asking for service [/cluedo_armor/backup] ...
[INFO] [1636487504.726978]: [robocluedo_main] OK!
```

When all the nodes are ready, the robot can start moving.

```
		...
[INFO] [1636487504.730808]: [robocluedo_main] starting the FSM...
[INFO] [1636487504.731578]: State machine starting in initial state 'robocluedo_random_target' with userdata: 
	[]
```

Here is a representation of what happened up to now:

![UML sequence loading](/robocluedo/docs/diagrams/UML_sequence_example_loading.png)

At the beginning, the robot has no informations: the ontology is empty, so it chooses randomly a room asking to the node cluedo_random_room in search of clues. Each time it enteres in one room, it receives *at most* one hint, or also nothing in some cases. The robot keeps behave in this way until at least one hypothesis becomes complete. In the following piece of log, the typical job of the robot while is looking for clues:

```
		...
[INFO] [1636487510.302336]: [state:random_target] actual position: bathroom | random target: bathroom
[INFO] [1636487510.303734]: State machine transitioning 'robocluedo_random_target':'move_robot'-->'robocluedo_moving'
[ INFO] [1636487511.307296200]: [cluedo_oracle] publishing hint (ID:0, PROP:who, VALUE:mustard)
[INFO] [1636487511.307459]: [state:moving] reached position: bathroom 
[INFO] [1636487511.308630]: State machine transitioning 'robocluedo_moving':'target_reached_no_charge'-->'robocluedo_listening_for_hints'
[INFO] [1636487511.309457]: [state:listening_for_hints] received a hint
[INFO] [1636487511.310390]: State machine transitioning 'robocluedo_listening_for_hints':'received_a_hint'-->'robocluedo_update_ontology'
[INFO] [1636487511.311347]: [state:update_ontology] received hint: HP0(who:mustard)
[INFO] [1636487511.379564]: [state:update_ontology] writing backup of the ontology...
[INFO] [1636487511.412380]: [state:update_ontology] ontology saved
[INFO] [1636487511.413624]: State machine transitioning 'robocluedo_update_ontology':'done'-->'robocluedo_reasoning'
[INFO] [1636487511.432874]: State machine transitioning 'robocluedo_reasoning':'no_consistent_hp'-->'robocluedo_random_target'
```

A representation of what is happening in the common cycle above mentioned. The worst case scenario is assumed. 

![UML sequence working cycle](/robocluedo/docs/diagrams/UML_sequence_example_working_cycle.png)

In the example, there are no wrong charges unfortunately, but it is reasonable to suppose that, on a more complex setup (many elements in the list, many IDs, ...) the robot could fail sometimes. Here is the final piece of the log, which shows how the robot behaves when it is ready to present a charge. 

```
		...
[INFO] [1636487535.449865]: State machine transitioning 'robocluedo_reasoning':'no_consistent_hp'-->'robocluedo_random_target'
[INFO] [1636487535.452961]: [state:random_target] actual position: study | random target: bathroom
[INFO] [1636487535.453893]: State machine transitioning 'robocluedo_random_target':'move_robot'-->'robocluedo_moving'
[ INFO] [1636487536.456438300]: [cluedo_oracle] publishing hint (ID:0, PROP:where, VALUE:study)
[INFO] [1636487536.456450]: [state:moving] reached position: bathroom 
[INFO] [1636487536.458016]: State machine transitioning 'robocluedo_moving':'target_reached_no_charge'-->'robocluedo_listening_for_hints'
[INFO] [1636487536.459015]: [state:listening_for_hints] received a hint
[INFO] [1636487536.459973]: State machine transitioning 'robocluedo_listening_for_hints':'received_a_hint'-->'robocluedo_update_ontology'
[INFO] [1636487536.460861]: [state:update_ontology] received hint: HP0(where:study)
[INFO] [1636487536.473777]: [state:update_ontology] writing backup of the ontology...
[INFO] [1636487536.494515]: [state:update_ontology] ontology saved
[INFO] [1636487536.496134]: State machine transitioning 'robocluedo_update_ontology':'done'-->'robocluedo_reasoning'
[INFO] [1636487536.513820]: [state:reasoning] complete hypotheses found: 1
```

If the number of hypotheses would have been 2, the robot have been choosen randomly from the available ones. Here there's only one hypothesis, and it will be so every time because the robot "thinks" every time it enters in a room, and at most one hint can be sent to the robot: if the oracle would send more than one hint at time, the robot have had ore elements to elaborate. 

The actual position is not the one mentioned in the hypothesis, so the robot first moves towards the room, and when is there, makes the charge. The solution is correct, so the program ends. 

```
[INFO] [1636487536.515209]: [state:reasoning] HP0(who:mustard, where:study, what:dagger)
[INFO] [1636487536.516173]: [state:reasoning] selected hyp. with tag: HP0 | actual position: bathroom
[INFO] [1636487536.517029]: State machine transitioning 'robocluedo_reasoning':'not_in_the_right_place'-->'robocluedo_moving'
[ INFO] [1636487537.519778200]: [cluedo_oracle] publishing hint (ID:1, PROP:what, VALUE:revolver)
[INFO] [1636487537.519989]: [state:moving] reached position: study 
[INFO] [1636487537.521567]: State machine transitioning 'robocluedo_moving':'target_reached_ready_for_charge'-->'robocluedo_charge'
[INFO] [1636487537.522649]: [state:charge] asking to the oracle: (who:mustard, where:study, what:dagger)
[ INFO] [1636487537.524821900]: [cluedo_oracle] evaluating the solution WHERE[study]  WHO [mustard]  WHAT [dagger] 
[ INFO] [1636487537.524905900]: [cluedo_oracle] SUCCESS! Found the solution. 
[INFO] [1636487537.525135]: [state:charge] mystery solved!
[INFO] [1636487537.526102]: State machine terminating 'robocluedo_charge':'case_solved':'mystery_solved'
```

The last part of the log can be resumed with this diagram:

![UML sequence working cycle](/robocluedo/docs/diagrams/UML_sequence_example_final.png)

This was a very simple example, aimed at make you understand how this project works. Inside the directory *logs* you can find some more interesting examples with the default setup.
