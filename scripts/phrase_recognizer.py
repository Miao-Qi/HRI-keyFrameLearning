#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
import speech_recognition as sr

# called in background thread
def callback(recognizer, audio):
    pub = rospy.Publisher('verbalInstruction', String, queue_size=10)
    try:
        command = recognizer.recognize_google(audio)
        #command = recognizer.recognize_sphinx(audio)
        pub.publish(command)
        print("Google Speech Recognition thinks you said " + command)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def PhraseRecognizer():
    rospy.init_node('PhraseRecognizer', anonymous=True)

    # Microphone setup
    r = sr.Recognizer()
    m = sr.Microphone()

    # microphone calibrate
    with m as source:
        r.adjust_for_ambient_noise(source)

    # start listening in the background 
    stop_listen = r.listen_in_background(m, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        PhraseRecognizer()
    except rospy.ROSInterruptException:
        pass

