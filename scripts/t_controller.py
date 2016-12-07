#!/usr/bin/python

import rospy
import math

from turtlesim.msg     import Pose
from std_msgs.msg      import String
from std_msgs.msg      import ColorRGBA
from geometry_msgs.msg import Twist

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
pub      = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pubColor = rospy.Publisher('set_color', ColorRGBA, queue_size=10) 

def makeColor(r, g, b) : 
  result = ColorRGBA() 
  result.r = r 
  result.g = g 
  result.b = b 
  result.a = 0.3
  return result 

# Use linear velocity instead of angular velocity to control direction
def myCallback(data):
    global SPD
    global CurTheta 
    global CurTwist 
    global pub

    # Use LED light to show respond 
    # default    white         (255, 255, 255) 
    # go         green         (  0, 255,   0) 
    # turn left  yellow        (255, 255,   0)    
    # turn right blue          (  0,   0, 255)
    # stop       red           (255,   0,   0)
    # keep frame cyan          (  0, 255, 255)
    print (rospy.get_caller_id() + 'Current instruction: ' + data.data + "\n")

    if data.data == "go":
        CurTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
        pubColor.publish(makeColor(  0, 255,   0))
        rospy.sleep(0.2)
        pubColor.publish(makeColor(255, 255, 255))
    elif data.data == "turn right":
        CurTheta -= math.pi/2
        CurTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
        pubColor.publish(makeColor(  0,   0, 255))
        rospy.sleep(0.2)
        pubColor.publish(makeColor(255, 255, 255))
    elif data.data == "turn left":
        CurTheta += math.pi/2
        CurTwist = createTwist(SPD * math.cos(CurTheta), SPD * math.sin(CurTheta))
        pubColor.publish(makeColor(255, 255,   0))
        rospy.sleep(0.2)
        pubColor.publish(makeColor(255, 255, 255))
    elif data.data == "stop":
        CurTwist = createTwist(0, 0)
        pubColor.publish(makeColor(255,   0,   0))
        rospy.sleep(0.2)
        pubColor.publish(makeColor(255, 255, 255))
    elif data.data == "keep frame": 
        pubColor.publish(makeColor(  0, 255, 255))
        rospy.sleep(0.2)
        pubColor.publish(makeColor(255, 255, 255))

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
    pubColor.publish(makeColor(255, 255, 255))
    while not rospy.is_shutdown():
        pub.publish(CurTwist)
        rate.sleep()

if __name__ == '__main__':
    turtle_controller()

