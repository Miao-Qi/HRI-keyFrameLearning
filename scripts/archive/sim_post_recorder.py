#!/usr/bin/python

import rospy
import math
from turtlesim.msg import Pose
from nav_msgs.msg  import Odometry
import time
import datetime

init_time = 0
ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
f = open('path-' + str(st) + '.txt', 'w+')

# write new pose information of robot into log file
# data format: 
#      x,y,delta_time_stamp,theta
# file name:
#      "path-%Y-%m-%d_%H:%M:%S.txt"
# file location:
#      under catkin workspace root directory
def callback(data):
    global init_time
    delta_time = time.time() - init_time
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    newData = str(data.x) + "," + str(data.y) + ","  + str(delta_time) + ',' + str(data.theta)

    f.write(newData)
    f.write("\n")
    print "Write to file"

def listener():
    global init_time
    init_time = time.time()
    rospy.init_node('listener', anonymous=True)
    # listening to the turtle pose topic
    rospy.Subscriber('/turtle1/pose', Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    f.close()

if __name__ == '__main__':
    listener()

