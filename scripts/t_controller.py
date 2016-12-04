#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

#last_instruction = ""
target_theta = 0

# ----------------------------------------------------------------------
# Configuration Constants 
# ----------------------------------------------------------------------
# Sphero speed 
SPD = 50 

# ----------------------------------------------------------------------
# Global states
# ----------------------------------------------------------------------
CurTheta = 0

# TODO: Modified
# Use linear velocity instead of angular velocity to control direction
def myCallback(data):
    global SPD
    global CurTheta 
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    print (rospy.get_caller_id() + 'Current instruction: ' + data.data + "\n")
    newTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
    if data.data == "go straight":
        newTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
    #if data.data == "right" and data.data == last_instruction:
    if data.data == "turn right":
        CurTheta -= math.pi/2
        # newTwist = createTwist(1, -math.pi/4)
        newTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
        last_instruction = data.data
    #if data.data == "left" and data.data == last_instruction:
    if data.data == "turn left":
        CurTheta += math.pi/2
        # newTwist = createTwist(1, math.pi/4)
        newTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
    pub.publish(newTwist)
    print ("Publish new twist: ")
    print (newTwist)
    print ("\n")

# TODO: Modified
# Use linear velocity instead of angular velocity to control direction
def createTwist(lx, ly):
    newTwist = Twist()
    newTwist.linear.x = lx
    newTwist.linear.y = ly
    newTwist.linear.z = 0
    newTwist.angular.x = 0
    newTwist.angular.y = 0
    newTwist.angular.z = 0
    return newTwist

def turtle_controller():
    rospy.init_node('turtle_controller', anonymous=True)
    rospy.Subscriber('verbalInstruction', String, myCallback)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    turtle_controller()

