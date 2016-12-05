#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import String

pub = rospy.Publisher('verbalInstruction', String, queue_size=10)

def PhraseRecognizer():
    rospy.init_node('PhraseRecognizer', anonymous=True)
    while not rospy.is_shutdown():
      cmd = raw_input("Enter a command: ")
      pub.publish(cmd)


if __name__ == '__main__':
    try:
        PhraseRecognizer()
    except rospy.ROSInterruptException:
        pass

