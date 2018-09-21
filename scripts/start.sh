#!/bin/bash

# -------------------------------------------------------------------- BSD 3-Clause License
# https://opensource.org/licenses/BSD-3-Clause
#
# Copyright (c) 2018, Shogo Okano
# All rights reserved.
# --------------------------------------------------------------------

showHelp() {
    echo 
    echo " Run it from command line:"
    echo "    executable (chmod +x start.sh)"
    echo 
}
# geometory : (COLSxROWS+X+Y)

# Realsense wakeup.
#xterm -geometry 80x100+0+0   -e "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_camera.launch" &
gnome-terminal.real --geometry 80x100+0+0 --command "/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_camera.launch" &

sleep 10s

# ArUco marker.
#xterm -geometry 80x100+400+0   -e "/opt/ros/kinetic/bin/roslaunch aruco_ros marker_publisher.launch" &
gnome-terminal.real --geometry 80x100+400+0 --command "/opt/ros/kinetic/bin/roslaunch aruco_ros marker_publisher.launch" &

sleep 3s

# Master.
#xterm -geometry 80x100+800+0   -e "/opt/ros/kinetic/bin/roslaunch o_display_disporsal_2018 master.launch" &
gnome-terminal.real --geometry 80x100+800+0 --command "/opt/ros/kinetic/bin/roslaunch o_display_disporsal_2018 master.launch" &
#roslaunch o_display_disporsal_2018 master.launch


# Wheel.
#xterm -geometry 80x100+0+0   -e "/opt/ros/kinetic/bin/roslaunch o_display_disporsal_2018 display_disporsal.launch" &
