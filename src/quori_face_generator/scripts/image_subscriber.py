#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import time

pub = rospy.Publisher('sensor_msgs/Image', Image, queue_size=10)

def callback(data):
    bridge = CvBridge() 
    img = bridge.compressed_imgmsg_to_cv2(data)

    data = img.astype(np.uint8)


    imgMsg = Image()
    imgMsg.height = data.shape[0]
    imgMsg.width = data.shape[1]
    # imgMsg.step = left_image.strides[0]
    imgMsg.encoding = 'bgr8'
    imgMsg.header.frame_id = 'image_rect'
    imgMsg.header.stamp = rospy.Time.from_sec(time.time())
    imgMsg.data = data.flatten().tolist()

    
    pub.publish(imgMsg)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_subscriber', anonymous=True)

    rospy.Subscriber("face_image", CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()