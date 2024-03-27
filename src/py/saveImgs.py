#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime as time

def image_callback(img_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    cv2.imshow("Image", cv_image)

    # Save the image if press space
    if cv2.waitKey(10) == 32:
        cv2.imwrite('src/precision_landing/calib_imgs/' + str(time.now()) +'.png', cv_image)
        rospy.loginfo("Image saved!")

def subscriber():
    rospy.init_node('image_subscriber', anonymous=True)
    # rospy.Subscriber('/tag_detections_image', Image, image_callback)
    rospy.Subscriber('/cam0/cam0', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
