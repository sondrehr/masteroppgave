#!/usr/bin/python3

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

new_topic_publisher = None

def trajectory_callback(msg):
   global new_topic_publisher

   if new_topic_publisher is not None:
       new_topic_publisher.publish(msg)
       new_topic_publisher.publish(msg)

def copy_trajectory_callback(req):
   global new_topic_publisher

   new_topic_publisher = rospy.Publisher('trajectory_forwarder', MultiDOFJointTrajectory, queue_size=10)
   return

def main():
   rospy.init_node('trajectory_forwarder')
   rospy.Subscriber('/april/trajectory', MultiDOFJointTrajectory, trajectory_callback)
   rospy.Service('/forward_trajectory', Empty, copy_trajectory_callback)
   rospy.spin()

if __name__ == '__main__':
   main()