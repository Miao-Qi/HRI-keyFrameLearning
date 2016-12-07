#!/usr/bin/python

import rospy
import math
import time
import datetime
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

init_time = 0
ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
# f = open('./data/path-' + str(st) + '.txt', 'w+')
f = open('./data/frames.txt', 'w+')

# ----------------------------------------------------------------------
# Callback functions 
# ----------------------------------------------------------------------
def Callback (msg) : 
  global st
  global f
  # Get parameters from Odometry message
  curPosX = msg.pose.pose.position.x
  curPosY = msg.pose.pose.position.y
  curVelX = msg.twist.twist.linear.x
  curVelY = msg.twist.twist.linear.y
  curDirc = math.atan2(curVelY, curVelX)
  # Write to file
  newData = str(curPosX) + ',' + str(curPosY) + ',' + str(msg.twist.twist.angular.x) + ',' + str(curDirc)
  print(newData)
  # f = open('./data/path-' + str(st) + '.txt', 'a')
  f = open('./data/frames.txt', 'a')
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

