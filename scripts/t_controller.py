#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

#last_instruction = ""
target_theta = 0

def myCallback(data):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    print (rospy.get_caller_id() + 'Current instruction: ' + data.data + "\n")
    newTwist = createTwist(0, 0)
    if data.data == "go straight":
        newTwist = createTwist(1, 0)
    #if data.data == "right" and data.data == last_instruction:
    if data.data == "turn right":
        newTwist = createTwist(1, -math.pi/4)
        last_instruction = data.data
    #if data.data == "left" and data.data == last_instruction:
    if data.data == "turn left":
        newTwist = createTwist(1, math.pi/4)
    pub.publish(newTwist)
    print ("Publish new twist: ")
    print (newTwist)
    print ("\n")

def createTwist(lx, az):
    newTwist = Twist()
    newTwist.linear.x = lx
    newTwist.linear.y = 0
    newTwist.linear.z = 0
    newTwist.angular.x = 0
    newTwist.angular.y = 0
    newTwist.angular.z = az
    return newTwist

def turtle_controller():
    rospy.init_node('turtle_controller', anonymous=True)
    rospy.Subscriber('verbalInstruction', String, myCallback)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    turtle_controller()

