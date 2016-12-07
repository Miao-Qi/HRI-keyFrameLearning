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
SampleRate = 50

counter = 0
init_time = 0
ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
# f = open('./data/path-' + str(st) + '.txt', 'w+')
f = open('./data/frames.txt', 'w+')

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
    delta_time = time.time() - init_time
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    if counter == 0 : 
      theta = math.atan2(data.twist.twist.linear.y, data.twist.twist.linear.x)
      newData = str(data.pose.pose.position.x) + "," + str(data.pose.pose.position.y) + "," + str(delta_time) + "," + str(theta)  
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

