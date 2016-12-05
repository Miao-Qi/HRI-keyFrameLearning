#!/bin/bash 

# gnome-terminal -e 'roscore' &
# sleep 5 

# TODO
# For simulation
# gnome-terminal -e 'rosrun turtlesim turtlesim_node' &

# For Sphero 
# gnome-terminal -e 'rosrun sphero_node sphero.py' &
# sleep 10

gnome-terminal -e 'rqt_graph' &

gnome-terminal -e './data_process.py     ' & 
