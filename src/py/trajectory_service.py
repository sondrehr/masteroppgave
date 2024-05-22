#!/usr/bin/python3

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from precision_landing.srv import trajectory


def traj_callback(trajectory_msg):
    rospy.wait_for_service('trajectory_service')
    try:
        trajectory_service = rospy.ServiceProxy('trajectory_service', trajectory)
        resp = trajectory_service(trajectory_msg)
        print(resp.flag)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
def subscriber():
    rospy.init_node('trajectory_subscriber', anonymous=True)
    rospy.Subscriber('/april/trajectory', MultiDOFJointTrajectory, traj_callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
