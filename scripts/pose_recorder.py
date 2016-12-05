#!/usr/bin/python

import rospy
import math
from turtlesim.msg import Pose
from nav_msgs.msg  import Odometry

#counter = 0
f = open('frames.txt', 'w+')

def callback(data):
    global counter
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    #newData = str(counter) + "," + str(data.x) + "," + str(data.y) + "," + str(data.theta) + "," + str(data.linear_velocity) + "," + str(data.angular_velocity)

    # TODO
    # For simulation -----------------------------------------------------
    # newData = str(data.x) + "," + str(data.y) + "," + str(data.theta)
    # For Sphero ---------------------------------------------------------
    theta = math.atan2(data.twist.twist.linear.y, data.twist.twist.linear.x)
    newData = str(data.pose.pose.position.x) + "," + str(data.pose.pose.position.y) + "," + str(theta)  

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

