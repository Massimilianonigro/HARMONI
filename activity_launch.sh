#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/harmoni_catkin_ws/devel/setup.bash
roscore \
 & sleep 1 \
&& rosrun harmoni_pattern robot_ip.py \
 & sleep 10 \
&& roslaunch harmoni_pattern nuc_autostart.launch