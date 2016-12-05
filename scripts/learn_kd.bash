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

# TODO
gnome-terminal -e './phrase_rec_cheat.py ' & 
# gnome-terminal -e './phrase_recognizer.py' & 
gnome-terminal -e './t_controller.py     ' & 
gnome-terminal -e './keyframe_gen.py     ' & 
gnome-terminal -e './keyframe_recorder.py' & 
