#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(img_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    cv2.imshow("Image", cv_image)
    cv2.waitKey(1)

def subscriber():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/cam0/cam0', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()