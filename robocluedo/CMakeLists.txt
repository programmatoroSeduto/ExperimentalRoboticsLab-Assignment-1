cmake_minimum_required(VERSION 3.0.2)
project(robocluedo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  armor_msgs
  robocluedo_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES armor_tools
 CATKIN_DEPENDS roscpp rospy std_msgs armor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( armor_tools src/armor_tools/armor_tools.cpp )
target_link_libraries(armor_tools ${catkin_LIBRARIES} )

add_library( armor_cluedo src/armor_tools/armor_cluedo.cpp )
target_link_libraries(armor_cluedo ${catkin_LIBRARIES} armor_tools )

add_executable(cluedo_random_room src/cluedo_random_room.cpp)
target_link_libraries(cluedo_random_room ${catkin_LIBRARIES} )

add_executable(cluedo_movement_controller src/cluedo_movement_controller.cpp)
target_link_libraries(cluedo_movement_controller ${catkin_LIBRARIES} )

add_executable(cluedo_oracle src/cluedo_oracle.cpp)
target_link_libraries(cluedo_oracle ${catkin_LIBRARIES} )

add_executable(cluedo_armor_interface src/cluedo_armor_interface.cpp)
target_link_libraries(cluedo_armor_interface ${catkin_LIBRARIES} armor_tools armor_cluedo)

add_executable(test_armor src/test_armor.cpp)
target_link_libraries(test_armor ${catkin_LIBRARIES} armor_tools armor_cluedo)

add_executable(test_armor_tools src/test_armor_tools.cpp)
target_link_libraries(test_armor_tools ${catkin_LIBRARIES} armor_tools armor_cluedo)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/robocluedo_main.py
  scripts/test_cluedo_random_room.py
  scripts/test_cluedo_armor_interface.py
  scripts/test_cluedo_oracle.py
  scripts/test_oracle_plus_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS armor_tools armor_cluedo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
