# README - ERL - Assignment 1 - RoboCLuedo

**Francesco Ganci - 4143910** - Robotics Engineering - A.A. 2021/2022

> Documentation:
> - [Doxygen Documentation Here](doxygen.documentation.here)
> UML diagrams of the project:
> - [UML diagrams](foking.uml.diagrams)

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

## Package robobluedo_msgs

This package contains the messages and the services for the project. Please refer to the [documentation](please.read.the.documentation). 

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

# Install and Running the project

## dependencies

In order to run the project you need to install the followings:

- [ROSJava](http://wiki.ros.org/rosjava)

- [aRMOR](https://github.com/EmaroLab/armor#armor) : a ROS tool, working on ROSJava, able to manipulate .owl fles; please follow the instruction available on the readme of aRMOR in order to install it. 

- [AMOR](https://github.com/EmaroLab/multi_ontology_reference) : required by aRMOR, see the instruction of aRMOR

- [aRMOR msgs](https://github.com/EmaroLab/armor_msgs) : required in order to build the project. 

No Py client is required: the client was re-implemented in C++, see [ArmorTools](inserire.link.qui) in the documentation. 

## Build the project

With the above mentioned depts installed, building the project is straightforward:

0. I always recommend to create a clean workspace before download the project. Remember to source the new workspace!
1. Clone this repository inside the folder `src`
2. then, `catkin_make` on the entire workspace

Sometimes `catkin_make` gets stuck and doesn't compile the entire workspace. In order to force catkin to compile everythin, there's a script `compile.sh` attached to this repository.

## Runnung the project

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

Done this, there are a couple of nodes to start. I suggest you to run aRMOR first of all. Don't load anything: the arch. will load everything after starting. You could notice some warnigs: ignore them. 

```bash
rosrun armor execute it.emarolab.armor.ARMORMainService
```

Then, launch the auxiliary nodes. The node `cluedo_armor_interface` will set up the aRMOR service loading the ontology file you can find in `robocluedo/config/cluedo_owl_ontology`. This node manages and simplifies the usage of the ontology, and provides services specific for this project.

```bash
rosrun robocluedo cluedo_armor_interface
```

Now, run the node `cluedo_random_room`, a simple server which chooses one room among the ones in the list you can find at `robocluedo/config/cluedo_items`:

```bash
rosrun robocluedo cluedo_random_room
```

Run now the movement controller. This is a stub implementation, to replace with the real movement controller. Actually, it is a blocking service which can also cause the Oracle to send a hint to the robot. 

```bash
rosrun robocluedo cluedo_movement_controller
```

The cluedo oracle is the referee of the gambe. Its interface should be external to this project: the actual oracle can be easily replaced. The oracle reads three files of items (see the folder `robocluedo/config/cluedo_items`), and prepares the case, choosing the solution in advance. You can retrieve the solution from the output on the console. 

```bash
rosrun robocluedo cluedo_oracle
```

The last node you should run is the `robocluedo_main`, which implements the FSM, so the center of the architecture. The node makes the whole architecture running: the robot starts to work. 

```bash
rosrun robocluedo robocluedo_main.py
```

## Configuring the project - settings

All the configuration elements are located in the folder `robocluedo/config`. Here you an find:

- the launch file `parameter.launch` which contains all the parameters to be defined before running the architecture
- a folder `cluedo_items` containing all the entities for the game
- a folder `cluedo_ontology` containing the base ontology (the file *cluedo_ontology.owl*) ...
- .. and another folder `cluedo_ontology/last_ontology` where the node *robocluedo_main*, through the interface *robocluedo_armor_interface*, exports the ontology from the last execution of the robot. For debug purposes, you can inspect them, but first I suggest you to take a look at the documentation [about ArmorCluedo](armmor.cluedo-link).

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

Each file has a very simple structure: each line corresponds to an item of a given class. No parsing is needed: the program just imports them reading line per line. 

Regarding the ontology:

- `cluedo_path_owlfile` : the path where the base ontology is located
- `cluedo_path_owlfile_backup` : the path where to export the actual ontology

Items are loaded *dynamically* into the ontology, i.e. when the Oracle gives to the robot a property containing an item which it hasn't seen before. 

Another value is:

- `cluedo_max_hypotheses` : the number of IDs generated by the Oracle

Each hint has an ID; this lets to simplify the code of the node *cluedo_armor_interface*, otherwise the robot should assign a ID and expand the hint in a combinatorial way, making the research of complete hypotheses more difficult to execute.

## Testing the project

During the development of the application, they are been implemented several nodes which have the only purpose to test parts of the applications. Here is the complete list of the launch files and the type of test performed:

- `test_armor.launch` : the C++ node `test_armor.cpp` performs a simple reasoning task on the ontology interacting directly with the aRMOR service. It offers many handy functions which simplify the communication with aRMOR: it is meant to be modified and recompiled several times. If you want to alter it, of course you can: please follow the examples. 
- `test_armor_tools.launch` : the previous test is too much 'direct' and doesn't use ArmorTools neither ArmorCluedo. The node `test_armor_tools.cpp` offers a more rigorous testing using the C++ client (see [the documentation](aror.tools.and.armor.cluedo)). Unfortunately it is still in Italian (most of the part). 
- `test_cluedo_armor_interface.launch` : this simple test performs a rapid sequence of operations on `cluedo_armor_interface`, a simple pattern of reasoning. 
- `test_cluedo_oracle.launch` : simple test for the hint request in `cluedo_oracle.cpp`; perform the request of a hint 25 times. 
- `test_cluedo_random_room.launch` : same pattern as the previous, perform the request of a room for 50 times. 
- `test_oracle_plus_interface.launch` : joint test between `cluedo_oracle` and `cluedo_armor_tools`; the node asks to the Oracle a hint for a number of times, and each time the oracle replies. the hint is given to aRMOR through *cluedo_armor_interface*. All the features of both *cluedo_armor_interface* and *cluedo_oracle* are tested here.

# RCL - How RoboCLuedo Works

Here is a short presentation of the work done in this project. Please refer to the [Doxygen documentation](doxygen.foking.documentation) for further details, and in particular for a more detailed description of the interfaces between the nodes. 

## The components of the project

This is a behavioural architecture without the *sense* part. Here is the UML diagram of the whole project:

![CLuedo](/robocluedo/docs/diagrams/UML_components_arch_sketch.png)

Let's introduce the architecture node by node. If you want to know more about the central node, jump to the next section about the finite state machine. 

### NODE -- cluedo_random_room

> implementation [here](cluedo.random.room.cpp)

This is a simple node which chooses randomly a place from the PLACEs file. it is used when the robot is searching for clues. It exposes only one service. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_random_room.png)

### NODE -- cluedo_movement_controller

> implementation [here](cluedo.movement.controller.cpp)

This is a "stub" node, i.e. it represents a logical role but it does nothing. It represents the *act* part of the behavioural architecture; it can be replaced with a real movement controller easily. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_movement_controller.png)

Note that that the this node has also a channel connected with the Cluedo Oracle. The principle: the robot would receive one hint when it enters in the room, so the movement controller sends a signal every time the robot "reaches" a position. The signal is a message through the topic, which the oracle could accept or not. 

### NODE -- cluedo_oracle

> implementation [here](cluedo.oracle.cpp)

This node implements a sort of referee for the game. It knows the solution of the case, and can check, through service, the solutions proposed by the node robocluedo_main. Sometimes it can also give hints, i.e. propositions which the robot collects and associates in order to formulate a possible solution to be checked. *The solution could be incorrect*, so every time the robot has a solution, it must check it using a service of the oracle. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_cluedo_oracle.png)

The solution is generated in this way. First of all, the oracle chooses the ID of the solution. Then, after read the item files, the oracle bulds a shuffled list of [hint items](see.hint.message). For each ID, the node generates from zero to [MAX_SIZE_HINT](doxygen.MAX_SIZE_HINT.link). It should be possible (but very seldom) that one ID has not related hints. 

### NODE -- cluedo_armor_interface

> implementation [here](cluedo.armor.interface.cpp)

The node implements a simplified and specific interface which lets other nodes to work with aRMOR without using direct calls to the aRMOR service. 

![CLuedo](/robocluedo/docs/diagrams/UML_components_armor_interface.png)

The interface has four services (all the names of the services are under **/cluedo_armor**):
- **add_hint** : add a proerty to the ontology, adding the implied values if at least one of them doesn't exist. See the [implementation of ServiceAddHint()](service.add.hint) 
- **find\_consistent\_h** : retrieve all the consistet hypotheses from aRMOR. See the [implementation of ServiceFindConsistentHypotheses()](service.find_consistent_h) 
- **wrong_hypothesis** : discards a hypothesis after a negative result from the oracle about one conclusion. See the [implementation of DiscardHypothesis()](service.discard.hypothesis)
- **backup** : the service exports the actual ontology into the .owl backup file. See configurations, and the [implementation of ServiceBackupOntology()](service.backup.ontology)

### C++ CLASS -- ArmorTools

> implementation [here](armor.tools.cpp)

This class implements the most common mid-level methods for dealing with aRMOR without direct calls to the service. See the implementation for further details, and in particular [the examples](the.examples) for more details. 

Other useful documents: 

- [example1](example.one)
- [example2](example.second)

### C++ CLASS -- ArmorCluedo

> implementation [here](armor.cluedo.cpp)

The class extends the class ArmorToold, adding more handy specific mid-level methods for working with CLuedo individuals, properties and hypotheses. It adds also some workarounds needed in order tomake reliable the operations on aRMOR and overcoming some issues in commands DISJOINT and REMOVE. Please refer to the documentation if you want more details about all these aspects. 

Here are some useful documents:

- [example1](example.one)
- [example2](example.second)
- [example3](example.three)
- [test_armor_tools](test.armor.tools)

## The Finite State Machine -- robocluedo_main

The python ROS node **robocluedo_main** implements the behaviour of the robot. The ROS framework SMACH is employed here in order to implement the Finite State Machine. 

Here is the FSM diagram:

![CLuedo](/robocluedo/docs/diagrams/UML_FSM_sketch.png)

# RCL - Other notes on the project

... how it works

## Working Hypotheses

... hyps

## Possible improvements

... improvements, link to the todo list

# Author and Contacts

Francesco Ganci, S4143910

- **GitHub** : [here](https://github.com/programmatoroSeduto)
- **Email** : _s4143910@studenti.unige.it_