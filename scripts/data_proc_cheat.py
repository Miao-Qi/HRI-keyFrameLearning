#!/usr/bin/env python

import rospy
import sys
import math
import time
import numpy                 as     np

from   numpy                 import genfromtxt
from   sklearn.cluster       import KMeans
from   sklearn               import mixture
from   geometry_msgs.msg     import Twist
from   turtlesim.msg         import Pose
from   std_msgs.msg          import String
from   nav_msgs.msg          import Odometry

# ----------------------------------------------------------------------
# Configuration Constants 
# ----------------------------------------------------------------------
# Sphero speed 
SPD = 75 
# Position error threshold 
THR = 0.1

# ----------------------------------------------------------------------
# Publishers
# ----------------------------------------------------------------------
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# ----------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------
def createTwist(lx, ly): 
    newTwist = Twist()
    newTwist.linear.x = lx
    newTwist.linear.y = ly
    newTwist.linear.z = 0
    newTwist.angular.x = 0
    newTwist.angular.y = 0
    newTwist.angular.z = 0
    return newTwist

def CalcHeading (src, dst) : 
  height = dst[1] - src[1] 
  base   = dst[0] - src[0] 

  return math.atan2(height, base) 

def IsCloseEnough (src, dst) : 
  global THR
  height = dst[1] - src[1] 
  base   = dst[0] - src[0] 
  dist   = math.sqrt(math.pow(height, 2) + math.pow(base, 2))

  return (dist < THR) 

# ----------------------------------------------------------------------
# Global states
# ----------------------------------------------------------------------
curTheta = 0
curTwist = Twist()
counter  = 0
pos = []
timestamps = [] 
dirc = []
slept = False 

# ----------------------------------------------------------------------
# Callback functions 
# ----------------------------------------------------------------------

def init_model():
    global pos 
    global timestamps 
    global dirc 

    # data preprocess
    # my_data = genfromtxt('./data_file/path-2016-12-06_19:10:35.txt', delimiter=',')
    my_data = genfromtxt('./data_file/frames.txt', delimiter=',')
    dataSet = np.array(my_data)
    pos, timestamps, dirc = np.split(my_data, [2, 3], axis = 1)

def myCallback(data):
    global SPD
    global pub
    global curTheta
    global curTwist
    global pos 
    global counter
    global slept

    if counter >= len(pos) : 
      print("Navigation completed\n") 
      curTwist = createTwist(0, 0) 
      pub.publish(curTwist) 
      return 

    curPosX = data.pose.pose.position.x
    curPosY = data.pose.pose.position.y
    dstPosX = pos[counter][0] 
    dstPosY = pos[counter][1]
    print ("Current position: " + str(curPosX) + ", " + str(curPosY) + "\n") 
    print ("Heading position: " + str(dstPosX) + ", " + str(dstPosY) + "\n") 

    if IsCloseEnough([curPosX, curPosY], [dstPosX, dstPosY]) : 
      counter  = counter + 1 
      curTwist = createTwist(0, 0) 
      print("We are arriving at point # " + str(counter) + "\n") 
      pub.publish(curTwist)
      if not slept : 
        rospy.sleep(1)
      slept = True 
    else : 
      curTheta = CalcHeading([curPosX, curPosY], [dstPosX, dstPosY]) 
      curTwist = createTwist(SPD * math.cos(curTheta), SPD * math.sin(curTheta))  
      pub.publish(curTwist)
      slept = False


def DataProcess():
    init_model()
    rospy.init_node('data_process', anonymous=True)
    rospy.Subscriber('odom', Odometry, myCallback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    DataProcess()

