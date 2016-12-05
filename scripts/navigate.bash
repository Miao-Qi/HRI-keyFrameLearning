#!/bin/bash 

gnome-terminal -e 'roscore' &
sleep 5 

# For simulation
gnome-terminal -e 'rosrun turtlesim turtlesim_node' &

# TODO
# For Sphero 
# gnome-terminal -e 'rosrun sphero_node sphero.py' &
# sleep 75

gnome-terminal -e 'rqt_graph' &

gnome-terminal -e './data_process.py     ' & 
