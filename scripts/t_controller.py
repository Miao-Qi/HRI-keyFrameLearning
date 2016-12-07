#!/usr/bin/python

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
# TODO
# Sphero speed 
SPD = 75 

# ----------------------------------------------------------------------
# Global states
# ----------------------------------------------------------------------
CurTheta = 0
CurTwist = Twist()

# ----------------------------------------------------------------------
# Publishers
# ----------------------------------------------------------------------
# TODO
# For simulation ---------------------------------------------------
# pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
# For Sphero -------------------------------------------------------
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)



# Use linear velocity instead of angular velocity to control direction
def myCallback(data):
    global SPD
    global CurTheta 
    global CurTwist 
    global pub
    print (rospy.get_caller_id() + 'Current instruction: ' + data.data + "\n")
    if data.data == "go":
        CurTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
    elif data.data == "turn right":
        CurTheta -= math.pi/2
        CurTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
    elif data.data == "turn left":
        CurTheta += math.pi/2
        CurTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
    elif data.data == "stop":
        CurTwist = createTwist(0, 0)
    print ("Publish new twist: ")
    print (CurTwist)
    print ("\n")

# Use linear velocity instead of angular velocity to control direction
def createTwist(lx, ly):
    CurTwist = Twist()
    CurTwist.linear.x = lx
    CurTwist.linear.y = ly
    CurTwist.linear.z = 0
    CurTwist.angular.x = 0
    CurTwist.angular.y = 0
    CurTwist.angular.z = 0
    return CurTwist

def turtle_controller():
    rospy.init_node('turtle_controller', anonymous=True)
    rospy.Subscriber('verbalInstruction', String, myCallback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(CurTwist)
        rate.sleep()

if __name__ == '__main__':
    turtle_controller()

