#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/harmoni_catkin_ws/devel/setup.bash

roslaunch harmoni_pattern test.launch robot_ip:=192.168.100.176 activity_type:=Familiarisation_test area:=Touch level3:=None level4:=None