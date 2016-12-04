#!/bin/bash 

gnome-terminal -e 'roscore' &

# For simulation
gnome-terminal -e 'rosrun turtlesim turtlesim_node' &

# TODO
# For Sphero 
# gnome-terminal -e 'rosrun sphero_node sphero.py' &
# sleep 75

gnome-terminal -e './phrase_recognizer.py' & 
gnome-terminal -e './t_controller.py     ' & 
gnome-terminal -e './keyframe_gen.py     ' & 
gnome-terminal -e './keyframe_recorder.py' & 
