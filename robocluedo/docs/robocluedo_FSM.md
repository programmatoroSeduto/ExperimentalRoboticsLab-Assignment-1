\page robocluedo_FSM RoboCLuedo -- FMS -- UML Finite State Machine Diagram
\brief The UML FSM diagram

# RoboCLuedo -- UML -- FSM

This diagram represents the behaviour implemented in the ROS node robocluedo_main.py . Please take into account the followings:

- the init state is connected to a ball with label **INIT** and the end state to another ball labeled **SUCCESS**. 

- into each node, you can find the name of the state, and the global data the state needs for the elaboration; you can find also the services one state needs to call, in italic. 

<br>

@image html UML_FSM_sketch.png

<br>