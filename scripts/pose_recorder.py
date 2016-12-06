#!/usr/bin/python

import rospy
import math
import time
import datetime

from turtlesim.msg import Pose
from nav_msgs.msg  import Odometry

#counter = 0
init_time = 0
ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
f = open('./data_file/path-' + str(st) + '.txt', 'w+')

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
    #newData = str(counter) + "," + str(data.x) + "," + str(data.y) + "," + str(data.theta) + "," + str(data.linear_velocity) + "," + str(data.angular_velocity)

    # TODO
    # For simulation -----------------------------------------------------
    # newData = str(data.x) + "," + str(data.y) + "," + str(data.theta)
    # For Sphero ---------------------------------------------------------
    theta = math.atan2(data.twist.twist.linear.y, data.twist.twist.linear.x)
    newData = str(data.pose.pose.position.x) + "," + str(data.pose.pose.position.y) + "," + str(delta_time) + "," + str(theta)  

    f.write(newData)
    f.write("\n")
    #counter = counter + 1
    print "Write to file"

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

