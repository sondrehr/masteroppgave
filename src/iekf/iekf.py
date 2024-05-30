#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose

import numpy as np
import inekf





class iekf:

    def prediction(self, Imu):
        U = np.array(Imu.angular_velocity, Imu.linear_acceleration)
        dt = rospy.Time.now() - self.time
        self.iekf.predict(U, dt)

    def correction(self, AprilPose):
        print("Correction")

    def __init__(self):
        self.time = rospy.Time.now()

        self.processModel = inekf.InertialProcess()
        self.x0 = inekf.SE3[2, 6](0, 0, 0, 0, 0, 0, 0, 0, 0, np.eye(9))
        self.iekf = inekf.INEKF(self.processModel, self.x0, inekf.ERROR.RIGHT)

        b = iekf.SE3(0, 0, 0, 0, 0, 0)
        R = np.eye(6)
        self.pose = inekf.MeasureModel[inekf.SE3[2, 6]](b, R, inekf.ERROR.LEFT)
        self.iekf.addMeasurementModel("pose", self.pose)
        

        rospy.init_node('iekf_node', anonymous=True)
        rospy.Subscriber('/drone/imu', Imu, self.prediction)
        rospy.Subscriber('/april/pose', Pose, self.correction)
        rospy.spin()

if __name__ == '__main__':
    iekf()
