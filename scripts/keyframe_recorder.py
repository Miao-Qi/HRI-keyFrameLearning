#!/usr/bin/python

import rospy
import math
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

f = open('frames.txt', 'w+')

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
  newData = str(curPosX) + ',' + str(curPosY) + ',' + str(curDirc)
  print(newData)
  f = open('frames.txt', 'a')
  f.write(newData)
  f.write("\n")
  print("Write to file: \n") 
  print(msg)

# ----------------------------------------------------------------------
# Main routine 
# ----------------------------------------------------------------------
if __name__ == '__main__':
  rospy.init_node('keyframe_recorder') 
  print('keyframe_recorder') 
  rospy.Subscriber('keyframe', Odometry, Callback) 
  rospy.spin()
  f.close()

