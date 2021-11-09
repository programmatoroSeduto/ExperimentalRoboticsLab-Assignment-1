#! /bin/bash

set -e

# roscd robocluedo
cd ./../..
rm -rf devel
rm -rf build
catkin_make --only-pkg-with-deps robocluedo_msgs
catkin_make --only-pkg-with-deps robocluedo
# catkin_make --only-pkg-with-deps robocluedo_testing
catkin_make
#roscd robocluedo
cd ./..
