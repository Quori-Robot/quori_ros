#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import time

gazebo_pub = rospy.Publisher('quori/gazebo_face/image', Image, queue_size=10)
robot_pub = rospy.Publisher('quori/face/image', Image, queue_size=10)
def callback(data):
    bridge = CvBridge() 
    img = bridge.compressed_imgmsg_to_cv2(data)
    img2 = cv2.copyMakeBorder(
                 img, 
                 100, #top
                 0, #bottom
                 0, #left
                 0, #right
                 cv2.BORDER_CONSTANT, 
                 value=[255,255,255]
              )
    img3 = cv2.copyMakeBorder(
                 img2, 
                 0, #top
                 800, #bottom
                 0, #left
                 0, #right
                 cv2.BORDER_CONSTANT, 
                 value=[255,255,255]
              )
    img4 = cv2.copyMakeBorder(
                 img3, 
                 0, #top
                 0, #bottom
                 50, #left
                 50, #right
                 cv2.BORDER_CONSTANT, 
                 value=[255,255,255]
              )
    img5 = cv2.flip(cv2.rotate(img4, cv2.ROTATE_90_CLOCKWISE), 1)
    resized = cv2.resize(img5, (240, 120))
    sharpened_image = unsharp_mask(resized)
    gazebo_pub.publish(bridge.cv2_to_imgmsg(sharpened_image, "bgr8"))
    robot_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=1.0, threshold=0):
    """Return a sharpened version of the image, using an unsharp mask."""
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_subscriber', anonymous=True)

    rospy.Subscriber("quori/face_generator_image", CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()