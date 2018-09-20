#!/bin/bash

# Realsense wakeup.
xterm -geometry 80x100+0+0   -e "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_camera.launch" &

sleep 10s

# ArUco marker.
xterm -geometry 80x100+400+0   -e "/opt/ros/kinetic/bin/roslaunch aruco_ros marker_publisher.launch" &

sleep 3s

# Master.
xterm -geometry 80x100+800+0   -e "/opt/ros/kinetic/bin/roslaunch o_display_disporsal_2018 display_disporsal.launch" &

# Wheel.
#xterm -geometry 80x100+0+0   -e "/opt/ros/kinetic/bin/roslaunch o_display_disporsal_2018 display_disporsal.launch" &

