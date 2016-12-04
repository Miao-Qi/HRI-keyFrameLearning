#!/usr/bin/python

import rospy
import sys

from nav_msgs.msg      import Odometry

########################################################################
# keyframe_recorder.py 
# 
# Keeps record of key frames into a keyframe.txt 
#
# Input:  Topic keyframe published by node PhraseRecognizer
# Output: keyframe.txt
# 
# Chunheng Luo 
# 2016-12-03
########################################################################

# ----------------------------------------------------------------------
# Node initialization
# ----------------------------------------------------------------------
rospy.init_node('keyframe_recorder') 

# ----------------------------------------------------------------------
# Subscribers and publishers
# ----------------------------------------------------------------------
Sub = rospy.Subscriber('keyframe', Odometry, Callback) 

# ----------------------------------------------------------------------
# Callback functions 
# ----------------------------------------------------------------------
def Callback (msg) : 
  # Get parameters from Odometry message
  curPosX = msg.pose.pose.position.x
  curPosY = msg.pose.pose.position.y
  curVelX = msg.twist.twist.linear.x
  curVelY = msg.twist.twist.linear.y
  curDirc = math.atan2(curVelY, curVelX)
  # Write to file
  fp = open('keyframe.txt', 'a')
  print fp.write(str(curPosX) + ',' + str(curPosY) + ',' + str(curDirc) + '\n')

# ----------------------------------------------------------------------
# Main routine 
# ----------------------------------------------------------------------
rate = rospy.Rate(10) 
while not rospy.is_shutdown() : 
  rate.sleep() 

