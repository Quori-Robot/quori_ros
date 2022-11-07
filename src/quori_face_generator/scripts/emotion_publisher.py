#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('emotion', Float64MultiArray, queue_size=10)
    rospy.init_node('emotion_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #joy, sadness, anger, disgust, fear, surprise
    emotion = [1, 0, 0.5, 0, 0.25, 0.5]
    data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
    data_to_send.data = emotion
    while not rospy.is_shutdown():
        pub.publish(data_to_send)
        rate.sleep()

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass