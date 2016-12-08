#!/usr/bin/python

import rospy
import math
import time
import datetime

from turtlesim.msg import Pose
from nav_msgs.msg  import Odometry

# ----------------------------------------------------------------------
# Constant configs
# ----------------------------------------------------------------------
# For nav_cheat.bash
# SampleRate = 50
# For navigation.bash
SampleRate = 0

counter = 0
init_time = 0
ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
# f = open('./data/path-' + str(st) + '.txt', 'w+')
f = open('./data/frames.txt', 'w+')

last_theta = 0.0
first_sample = True

# write new pose information of robot into log file
# data format: 
#      x,y,delta_time_stamp,theta
# file name:
#      "path-%Y-%m-%d_%H:%M:%S.txt"
# file location:
#      under catkin workspace root directory
def callback(data):
    global SampleRate
    global counter
    global init_time
    global last_theta
    global first_sample

    delta_time = time.time() - init_time
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    if counter == 0 : 

      # If Sphero is not moving, use the theta of the previous callback
      twistX = data.twist.twist.linear.x
      twistY = data.twist.twist.linear.y
      if first_sample : 
        last_theta = math.atan2(twistY, twistX)
        first_sample = False
      if math.fabs(twistX) < 0.1 and math.fabs(twistY) < 0.1 : 
        theta = last_theta 
      else : 
        theta = math.atan2(twistY, twistX)

      # Seems that Sphero self-tracking error is more than 0.01 
      # For better clustering results, round off the inaccurate digits
      posX = round(data.pose.pose.position.x, 2)  
      posY = round(data.pose.pose.position.y, 2)  
      newData = str(posX) + "," + str(posY) + "," + str(delta_time) + "," + str(theta)  

      f = open('./data/frames.txt', 'a')
      f.write(newData)
      f.write("\n")
      print "Write to file"

    if counter == SampleRate : 
      counter = 0
    else : 
      counter = counter + 1


def listener():
    rospy.init_node('listener', anonymous=True)
    # TODO
    # For simulation -----------------------------------------------------
    # rospy.Subscriber('/turtle1/pose', Pose, callback)
    # For Sphero ---------------------------------------------------------
    rospy.Subscriber('odom', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    f.close()

if __name__ == '__main__':
    listener()

