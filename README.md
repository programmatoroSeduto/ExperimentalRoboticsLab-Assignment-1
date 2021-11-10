# README - Experimental Robotics Lab - Assignment 1 - RoboCLuedo

**Francesco Ganci - 4143910** - Robotics Engineering - A.A. 2021/2022

> Documentation:
> 
> - [Doxygen Documentation Here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/files.html)
> - [UML diagrams](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00369.html)
> 
> For a practical point of view on this project, 
> 
> - See the [practical example](#a-practical-example)

![CLuedo](/robocluedo/docs/img/cluedologo.jpg)

# RCL - Introduction

This project contains a ROS implementation of the control program of a robot which can play to a very limited and simplified version of the well-known game CLuedo. Here is implemented only the logic structure of the system, which can be easily extended and possiblty "mounted" on a real robot. 

A behavioural architecture is implemented here. The robot "moves" among the rooms of a house in order to find who, where, and with what DrBlack was killed. The project also employs artificial intelligence techniques in order to solve the case. The robot interacts with a particular node, called **Oracle**, which is responsible for giving "hints" to the robot. The oracle knows the solution of the case, and checks the solution proposed by the robot each time it is ready to make a charge. If the proposed solution is correct, the program stops. 

# RCL - Packages Structure

The project is distributed into two different packages:

- **robocluedo** : the main folder of the project, containing the implementation
- **robocluedo_msgs** : it contains some useful utilities for the project. I decided to split the project for clarity reasons. 

## Package robocluedo

It contains the main part of the architecture. You can configure how it works from the *config* folder, and test it using the launch files (see below in this document). 

```
robocluedo
├───config                   <> file parameters.launch
│   ├───cluedo_items         <> cluedo items in three text files
│   └───cluedo_owl_ontology  <> base ontology
│       └───last_ontology    <> backup of the ontology from the last execution
|
├───docs                     <> diagrams, examples on the project, adn other pages
│   ├───diagrams
│   ├───examples
│   └───img
|
├───include                  
│   └───armor_tools          <> headers of the classes ArmorTools and ArmorCluedo
|
├───launch                   <> main launch file
│   └───tests                <> other launch files for testing the arch
|
├───logs                     <> log files from previous tests on the complete architecture
|
├───scripts                  <> the main node, and other py nodes for testing the arch
|
└───src                      <> C++ nodes for the project
    └───armor_tools          <> implementation of the classes ArmorTools and ArmorCluedo
```

## Package robocluedo_msgs

This package contains the messages and the services for the project.  

```
robocluedo_msgs
|
├───msg
│       Hint.msg
│       Hypothesis.msg
│
└───srv
        AddHint.srv
        CheckSolution.srv
        DiscardHypothesis.srv
        FindConsistentHypotheses.srv
        GoTo.srv
        RandomRoom.srv
```

# Installing and Running the project

## dependencies

In order to run the project you need to install the followings:

- [ROSJava](http://wiki.ros.org/rosjava)

- [aRMOR](https://github.com/EmaroLab/armor#armor) : a ROS tool, working on ROSJava, able to manipulate .owl fles; please follow the instruction available on the readme of aRMOR in order to install it. 

- [AMOR](https://github.com/EmaroLab/multi_ontology_reference) : required by aRMOR, see the instructions of aRMOR

- [aRMOR msgs](https://github.com/EmaroLab/armor_msgs) : required in order to build the project. 

No Py client is required: the client was re-implemented in C++, see [ArmorTools](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00219.html) in the documentation. 

## Build the project

With the above mentioned depts installed, building the project is straightforward:

0. I always recommend to create a clean workspace before downloading the project.
1. Clone this repository inside the folder `src`
2. then, `catkin_make` on the entire workspace

Sometimes, for some strange reason, `catkin_make` gets stuck and doesn't compile the entire workspace. In order to force catkin to compile everythin, there's a script `compile.sh` attached to this repository. It simply calls *catkin* for each package of the project. 

## Running the project

There are at least two ways to run the architecture.

### First way - Launch file

You can run the entire program from the launch file I provided for you:

```bash
clear && roslaunch robocluedo run_robocluedo.launch
```

### Second way - Slow method

Before starting, remember to launch the ROS master:

```bash
roscore &
```

First of all, you need to set some stuff on the parameter server. You can easily do it from the launch file you can find in `robocluedo/config`:

```bash
roslaunch robocluedo parameters.launch
```

Done this, there are a couple of nodes to start. I suggest you to run aRMOR before the other nodes. Don't load anything: the arch. will load everything after starting. You could notice some warnigs: ignore them. 

```bash
rosrun armor execute it.emarolab.armor.ARMORMainService &
```

Then, launch the auxiliary nodes. The node `cluedo_armor_interface` will set up the aRMOR service loading the ontology file you can find in `robocluedo/config/cluedo_owl_ontology`. This node manages and simplifies the usage of the ontology, and provides services specific for this project.

```bash
rosrun robocluedo cluedo_armor_interface &
```

Now, run the node `cluedo_random_room`, a simple server which chooses one room among the ones in the list you can find at `robocluedo/config/cluedo_items`:

```bash
rosrun robocluedo cluedo_random_room &
```

Run now the movement controller. This is a stub implementation: it should be replaced with a real movement controller, with no need to alter the interface of the architecture. Actually, it is a blocking service which can also cause the Oracle to send a hint to the robot. 

```bash
rosrun robocluedo cluedo_movement_controller &
```

The cluedo oracle is the referee of the game. Its interface should be external to this project: the actual oracle can be easily replaced. The oracle reads three files of items (see the folder `robocluedo/config/cluedo_items`), and prepares the case, choosing the solution in advance. You can retrieve the solution from the output on the console. 

```bash
rosrun robocluedo cluedo_oracle &
```

The last node you should run is the `robocluedo_main`, which implements the FSM, so the center of the architecture. The node makes the whole architecture running: the robot starts to work. 

```bash
rosrun robocluedo robocluedo_main.py
```

That's all folks!

## Configuring the project - settings

All the configuration elements are located in the folder `robocluedo/config`. Here you can find:

- the launch file `parameter.launch` which contains all the parameters to be defined before running the architecture
- a folder `cluedo_items` containing all the entities for the game
- a folder `cluedo_ontology` containing the base ontology (the file *cluedo_ontology.owl*) ...
- .. and another folder `cluedo_ontology/last_ontology` where the node *robocluedo_main*, through the interface *robocluedo_armor_interface*, exports the ontology from the last execution of the robot. For debug purposes, you can inspect them, but first I suggest you to take a look at the documentation [about ArmorCluedo](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00215.html) and in particular the section "DISCARD HYPOTHESIS" from the [example 3](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00249.html) of ArmorCluedo. 

Here is the structure of the folder `config`:

```
robocluedo
├───config
│   │   parameters.launch
│   │
│   ├───cluedo_items
│   │       cluedo_what.txt
│   │       cluedo_where.txt
│   │       cluedo_who.txt
│   │
│   └───cluedo_owl_ontology
│       │   cluedo_ontology.owl
│       │
│       └───last_ontology
│               cluedo_last_ontology.owl
...
```

### Parameters and Item Files

The program need three test files:

- `cluedo_path_where` : the path of the file text containing all the PLACEs
- `cluedo_path_what` : the the path of the file text containing all the WEAPONs
- `cluedo_path_who` : the path of the file text containing all the PERSONs

Each file has a very simple structure: each line corresponds to an item of a given class. No parsing is needed: the program just imports them, reading line per line. 

Regarding the ontology:

- `cluedo_path_owlfile` : the path where the base ontology is located
- `cluedo_path_owlfile_backup` : the path where to export the actual ontology

Items are loaded *dynamically* into the ontology, i.e. when the Oracle gives to the robot a property containing an item which it hasn't seen before. 

Another value is:

- `cluedo_max_hypotheses` : the number of IDs generated by the Oracle

Each hint has an ID; this lets to simplify the code of the node *cluedo_armor_interface*, otherwise the robot should assign a ID and expand the hint in a combinatorial way, making the research of complete hypotheses more difficult to execute.

## Testing the components

During the development of the application, they are been implemented several nodes which have the only purpose to test parts of the applications. Here is the complete list of the launch files and the type of test performed:

- `test_armor.launch` : the C++ node `test_armor.cpp` performs a simple reasoning task on the ontology interacting directly with the aRMOR service. It offers many handy functions which simplify the communication with aRMOR: it is meant to be modified and recompiled several times. If you want to alter it, of course you can: please follow the examples. 
- `test_armor_tools.launch` : the previous test is too much 'direct' and doesn't use ArmorTools neither ArmorCluedo. The node `test_armor_tools.cpp` offers a more rigorous testing using the C++ client (see [the documentation](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00176.html)). Unfortunately it is still in Italian (most of the part). 
- `test_cluedo_armor_interface.launch` : this simple test performs a rapid sequence of operations on `cluedo_armor_interface`, a simple pattern of reasoning. 
- `test_cluedo_oracle.launch` : simple test for the hint request in `cluedo_oracle.cpp`; perform the request of a hint 25 times. 
- `test_cluedo_random_room.launch` : same pattern as the previous, perform the request of a room for 50 times. 
- `test_oracle_plus_interface.launch` : joint test between `cluedo_oracle` and `cluedo_armor_tools`; the node asks to the Oracle a hint for a number of times, and each time the oracle replies. the hint is given to aRMOR through *cluedo_armor_interface*. All the features of both *cluedo_armor_interface* and *cluedo_oracle* are tested here.

# RCL - How RoboCLuedo Works

Here is a short presentation of the work done in this project.

## The components of the project

This is a behavioural architecture without the *sense* part. Here is the UML diagram of the whole project:

![CLuedo](/robocluedo/docs/diagrams/UML_components_arch_sketch.png)

Let's introduce the architecture node by node. If you want to know more about the central node, jump to the next section about the finite state machine. 

### NODE -- cluedo_random_room

> implementation [here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00170.html)

This is a simple node which chooses randomly a place from the PLACEs file. it is used when the robot is searching for clues. It exposes only one service. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_random_room.png)

### NODE -- cluedo_movement_controller

> implementation [here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00164.html)

This is a "stub" node, i.e. it represents a logical role but it does nothing. It represents the *act* part of the behavioural architecture; it can be replaced with a real movement controller easily. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_movement_controller.png)

Note that that the this node has also a channel connected with the Cluedo Oracle. The principle: the robot would receive one hint when it enters in the room, so the movement controller sends a signal every time the robot "reaches" a position. The signal is a message through the topic, which the oracle could accept or not. 

### NODE -- cluedo_oracle

> implementation [here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00167.html)

This node implements a sort of referee for the game. It knows the solution of the case, and can check, through service, the solutions proposed by the node robocluedo_main. Sometimes it can also give hints, i.e. propositions which the robot collects and associates in order to formulate a possible solution to be checked. *The solution could be incorrect*, so every time the robot has a solution, it must check it using a service of the oracle. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_oracle.png)

The solution is generated in this way. First of all, the oracle chooses the ID of the solution. Then, after read the item files, the oracle bulds a shuffled list of [hint items](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00179_source.html). For each ID, the node generates from zero to [MAX_SIZE_HINT](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00167_a4e8fea62d184b59ded0e110e55adc12d.html#a4e8fea62d184b59ded0e110e55adc12d). It should be possible (but very seldom) that one ID has not related hints. 

### NODE -- cluedo_armor_interface

> implementation [here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00161.html)

The node implements a simplified and specific interface which lets other nodes to work with aRMOR without using direct calls to the aRMOR service. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_armor_interface.png)

The interface has four services (all the names of the services are under **/cluedo_armor**):
- **add_hint** : add a proerty to the ontology, adding the implied values if at least one of them doesn't exist. See the [implementation of ServiceAddHint()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00161_a4867095aa20f38de61e518e16253520b.html#a4867095aa20f38de61e518e16253520b) 
- **find\_consistent\_h** : retrieve all the consistet hypotheses from aRMOR. See the [implementation of ServiceFindConsistentHypotheses()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00161_a7ac3004a3cbe78bf0b2c234d11354c43.html#a7ac3004a3cbe78bf0b2c234d11354c43) 
- **wrong_hypothesis** : discards a hypothesis after a negative result from the oracle about one conclusion. See the [implementation of DiscardHypothesis()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00161_a85e8c9731bfed7d2e82df47750b9c19e.html#a85e8c9731bfed7d2e82df47750b9c19e)
- **backup** : the service exports the actual ontology into the .owl backup file. See configurations, and the [implementation of ServiceBackupOntology()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00161_a66c612700258bd528a8d54035044b4ec.html#a66c612700258bd528a8d54035044b4ec)

### C++ CLASS -- ArmorTools

> implementation [here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00219.html)

This class implements the most common mid-level methods for dealing with aRMOR without direct calls to the service. See the implementation for further details, and in particular [the examples](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/examples.html) for more details. 

Other useful documents: 

- [Example 1](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00251.html) : ArmorTools - 001 - Essential usage of ArmorTools
- [Example 2](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00253.html) : ArmorTools - 002 - The quickest way to use ArmorTools

### C++ CLASS -- ArmorCluedo

> implementation [here](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00215.html)

The class extends the class ArmorToold, adding more handy specific mid-level methods for working with CLuedo individuals, properties and hypotheses. It adds also some workarounds needed in order tomake reliable the operations on aRMOR and overcoming some issues in commands DISJOINT and REMOVE. Please refer to the documentation if you want more details about all these aspects. 

Here are some useful documents:

- [Example 1](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00245.html) : ArmorCluedo - 001 - Working on Individuals
- [Example 2](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00247.html) : ArmorCluedo - 002 - Working with properties
- [Example 3](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00249.html) : ArmorCluedo - 003 - Reasoning Workflow
- [test_armor_tools](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00176.html)

## The Finite State Machine -- robocluedo_main

The python ROS node **robocluedo_main** implements the behaviour of the robot. The ROS framework [SMACH](http://wiki.ros.org/smach) is employed here in order to implement the Finite State Machine. 

Here is the FSM diagram:

![CLuedo](/robocluedo/docs/diagrams/UML_FSM_sketch.png)

Note that the robot could receive a message each time it enters in the room, *only one message* because the buffer is actually a single variable instead of a list of hints (see the [todo list](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00206.html) in the documentation). The behavour of the robot can be summarized as follows:

- when no complete hypotheses are available, the robot moves choosing randomly a room where to go
- each time the robot enters in a room, it "could find or not" one clue. If it "finds" something, it adds the hint to the ontology
- each time, the robot checks for any complete hypothesis; if it finds at least one complete hypothesis, it goes into the room and makes the charge
- the case is solved when the oracle confirms the conclusion of the robot, whereas, in case of negative answer, the robot keeps look for clues among the rooms

Clearly it is a oversimplified game dynamic: *the real CLuedo doesn't work in this way...*

# RCL - Other notes on the project

## A practical example

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

## Working Hypotheses

*The robot works in a well-known space, at least from the logical point of view*. Maybe the space would be unknown, but the robot knows in advance at least the names of the rooms. 

*The oracle sends at most one hint each time the robot enters in a room.* For multiple hints, it is necessary to extend the buffering from a single-var buffer type to a list type. The changes to do are few, and the logic can work fine also relaxing this working condition. 

In this implementation *the oracle sends in a precise instant the hint*, but this is not a constraint: the oracle can send the hint when it wants, because there's only one state which needs to check the state of the buffer. If the oracle would be replaced with another one which doesn't use the */hint_signal* but publishes the message in every instant, the architecture should work well. 

*The ID of the hypothesis is provided by the oracle.* Modifying this working hypothesis requires to change a bit the node *robocluedo_main*, but also reimplement the algorithm for adding the hypotheses in the ontology, so non-trivial changes to *cluedo_armor_interface*. It should be implemented an algorithm which, given some properties, can generate all the hypotheses which contains those properties. Providing the ID instead simplifies the workflow, because the interface already knows how to organize the hypotheses, so the problem is reduced to find the right ID among the ones provided by the oracle. 

## Possible improvements

You can find some of the possible improvements directly in the documentation of the code; see the section [todo](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00206.html). 

About the architecture:

- The actual *movement_controller* is just a "socket" for a real movement controller. The specifications require only that the service is *blocking*. 
- the sensing part is completely missing: actually the robot knows always its position without a localization system, and doesn't need to scan any QR code (or image processing, or sensing something) in order to understand the environment. This architecture is heaily focused on the AI part. 
- implementing the cluedo_armor_interface as an action could increase the performances. Actually, the services inside that node are the most time-consuming due to the huge number of needed aRMOR requests. See the time diagram. 

About how the game is managed:

- there's no need to a topic */hint_signal*: it has been introduced in the project only for demonstrating a more linear working situation. 
- the same oracle could manage more than one robot; it should be interesting to start the same architecture twice, and arrange a sort of challenge. It should be done using the same architecture of this project with minimal changes, or possibly with no changes. Other game dinamycs could also be implemented. 

About the Knowledge Base:

- in the real CLuedo, the knowledge is not centralized, but ditributed among the players: each player can exclude a priori some proerties, and hence all the hypotheses which contain them, and can discover, asking to the other players, new *real* clues. In the context of the project instead, it is difficult to apply some interesting reasoning technique such as the one of the real CLuedo. If the oracle indicates the part of the hypothesis not correct, the robot should understand which hypothesis is wrong in advance. 
- the actual .owl file can't distinguish between *inconsistent* hypotheses and *discarded* hypotheses, so the distinction must be implemented inside the architecture; in this case, in *cluedo_armor_interface*. Use DATAPROP in order to mark the hypotheses as discarded, or REMOVE them (see [ArmorCluedo::AddIndiv()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00215_ad29930fec7803aed87b7244e73b98bff.html#ad29930fec7803aed87b7244e73b98bff), [ArmorCluedo::RemoveHypothesis()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00215_a4ae659375e7a6d9564973718ece248e2.html#a4ae659375e7a6d9564973718ece248e2) and the implementation of [ArmorCluedo::ExistsIndiv()](https://programmatoroseduto.github.io/ExperimentalRoboticsLab-Assignment-1/a00215_a2086082362b61607798d6f968c263f9e.html#a2086082362b61607798d6f968c263f9e))

About the quality of the code:

- improve the actual naming convention, introducing a sort os standard more concept-oriented. Unfortunately, this is a tedious, time-consuming operation, despite the the effort to gather the use of messages in small parts of the code. 
- extend the functionalities of the classes ArmorTools and ArmorCluedo by adding the missing getters and setters
- implementation of the nodes as classes instead of using a "procedural" style

# Author and Contacts

Designed and developed by *Francesco Ganci*, S4143910

- **Email** : _s4143910@studenti.unige.it_
