#!/usr/bin/python

import math 
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

######################################################################
# ------------------------------------------------------------------ #
# Sphero Control 
# 
#                           ---------------
#  Work mode           ---> |             | ---> Sphero motion signals
#  Heading from speech ---> | Motion Ctrl | ---> Shpero LED signals   
#  Key frames          ---> |             | ---> Other control signals
#                           ---------------
# 
# Author         Chunheng Luo
# ------------------------------------------------------------------ #
######################################################################

# Constants 
PI        = 3.1415927
SPD       = 50 

# Node initialization
rospy.init_node('sphero_ctr')

# Publishers 
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Subscribers
# TODO: topic types not decided
heading_sub   = rospy.Subscriber('heading', Float32, SetHeading) 
work_mode_sub = rospy.Subscriber('work_mode', Bool, SwitchMode) 
key_frame_sub = rospy.Subscriber('key_frame', Float32MultiArray, CalcHeading) 

# Global control variables
# --------------------------------------------------------------------
# WorkMode: work mode of Sphero
# LRN: learning mode 
# EXE: executing mode 
WorkMode = 'LRN'
# --------------------------------------------------------------------
# Heading: angle in radius 
# Initially zero, for 'forward'
Heading  = 0
# --------------------------------------------------------------------
# Velocity vector: [x, y, z] 
# [SPD, 0, 0] if Heading == 0
VelVec   = [SPD, 0, 0]
# --------------------------------------------------------------------
# Current coord
CurCoord = [0, 0] 
# --------------------------------------------------------------------
# Next key frame coord 
NextKeyFrame = [0, 0] 

# Helper function
# Calculate heading according to src and dst coords
def DoCalcHeading (src, dst) : 
  height = dst[1] - src[1] 
  base   = dst[0] = src[0] 

  return math.atan(height/base) 
  

# Heading (from speech) call-back function 
# TODO: topic type assumed std_msgs/Float32
def SetHeading (msg) : 
  global Heading
  global VelVec
  
  if ( WorkMode == 'LRN' ) :
    Heading   = msg.data
    VelVec[0] = SPD * math.cos(Heading) 
    VelVec[1] = SPD * math.sin(Heading) 

# Work mode call-back function 
# TODO: topic type assumed std_msgs/Bool
# Ture for learning 
# False for executing
def SwitchMode (msg) : 
  global WorkMode 

  if (msg.data == True) : 
    WorkMode = 'LRN' 
  else : 
    WorkMode = 'EXE' 

# Key frame call-back function 
# TODO: topic type assumed std_msgs/Float32MultiArray
# Calculate Heading and VelVec according to the next key frame
def CalcHeading (msgs) : 
  global Heading
  global VelVec

  if ( WorkMode == 'LRN' ) :
    NextKeyFrame = msgs.data
    Heading = DoCalcHeading(CurCoord, NextKeyFrame) 
    VelVec[0] = SPD * math.cos(Heading) 
    VelVec[1] = SPD * math.sin(Heading) 

# Clock frequency: 10 Hz 
rate = rospy.Rate(10)

# Main loop
while not rospy.is_shutdown():
  
  # Create a Twist 
  OutTwist = Twist() 

  # Set speed 
  OutTwist.linear.x = VelVec[0] 
  OutTwist.linear.y = VelVec[1] 
  OutTwist.linear.z = VelVec[2] 

  # Publish the Twist 
  cmd_vel_pub.publish(OutTwist)

  # Wait for the next clock cycle 
  rate.sleep()

