\page robocluedo_arch RoboCLuedo -- COMP -- UML Components Diagram
\brief The UML components diagram of the overall architecture

# RoboCLuedo -- UML -- Components

Here is the UML components diagram of this project, a little different from the standard ROS convention. Please take into account that:

- the "black ball" represents the usage of a channel, whereas the "circular sockek" represents the exposed channel. For example, about ROS services, the black ball indicates a client, and the socket represents the server. 

- each connection is labeled, and each label tells which kind of channel is represented (Topic, Service) and the type of essage exchanged through that channel. 

<br>

@image html UML_components_arch_sketch.png

<br>