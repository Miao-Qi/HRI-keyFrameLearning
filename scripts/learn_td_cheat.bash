#!/bin/bash 

# gnome-terminal -e 'roscore' &
# sleep 5 

# TODO
# For simulation
# gnome-terminal -e 'rosrun turtlesim turtlesim_node' &

# For Sphero 
# gnome-terminal -e 'rosrun sphero_node sphero.py' &
# sleep 10

# gnome-terminal -e 'rqt_graph' &

gnome-terminal -e './t_controller.py       ' & 
gnome-terminal -e './pose_rec_cheat.py     ' & 

sleep 3
# TODO
gnome-terminal -e './phrase_rec_cheat.py ' & 
# gnome-terminal -e './phrase_recognizer.py' & 
