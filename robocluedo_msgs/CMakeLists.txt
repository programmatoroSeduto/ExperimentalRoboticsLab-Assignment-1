cmake_minimum_required(VERSION 3.0.2)
project(robocluedo_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Hint.msg
  Hypothesis.msg
)

add_service_files(
  FILES
  CheckSolution.srv
  RandomRoom.srv
  GoTo.srv
  AddHint.srv
  FindConsistentHypotheses.srv
  DiscardHypothesis.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package( )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
