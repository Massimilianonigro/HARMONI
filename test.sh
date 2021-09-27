#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/harmoni_catkin_ws/devel/setup.bash

roslaunch harmoni_pattern test.launch robot_ip:=$1 activity_type:=$2 area:=$3 level3:=$4 level4:=$5 local_ip:=$6