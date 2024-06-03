#!/usr/bin/python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from precision_landing.msg import myAprilTagDetectionArray, myAprilTagDetection

from manifpy import SE3
from scipy.spatial.transform import Rotation as Rot

class parseClass():
    T_mocap_base = SE3(np.array([0,0,0]), np.array([0,0,0,1]))
    T_platform_cam = SE3(np.array([0,0,0]), np.array([0,0,0,1]))

    x = []          # ground truth data
    z = []          # measurement data
    
    start_time = 0
    start_time_april = 0
    elapsed_time = 0
    elapsed_time_april = 0

    def __init__(self):
        self.subOdom = rospy.Subscriber('qualisys/hornbill/odom', Odometry, self.Odometry_callback)
        self.subApril = rospy.Subscriber('tag_detections', myAprilTagDetectionArray, self.April_callback)

        ## Mocap to base/platform transformation
        translation = np.array([-0.5838056466723349, 0.33955547547506265, 0.2059234324968905])
        orientation = np.array([0.000217340308570202, 0.00010564726176245313, -0.00042526692963236457, 0.9999998742783215])
        orientation = orientation / np.linalg.norm(orientation)
        self.T_mocap_base = SE3(translation, orientation)

        ## Camera to base/platform transformation
        translation = np.array([0.0, -0.1, 0.0])
        r = Rot.from_euler('z', np.pi/2)
        orientation = r.as_quat()
        self.T_platform_cam = SE3(translation, orientation)

    def April_callback(self, detectionArray):
        if self.start_time == 0: self.start_time = detectionArray.header.stamp.to_sec()         # Start time of the rosbag
        self.elapsed_time_april = detectionArray.header.stamp.to_sec() - self.start_time

        return                                                                                # Comment this line to start saving data 
        if self.elapsed_time > 18:
            print('Saving data...')
            with open('z3.npy', 'wb') as f:                                                     # Change the filename here to specify which rosbag file it is from: x1.npy, x2.npy, x3.npy
                np.save(f, self.z)
            return
        
        for detection in detectionArray.detections:
            t = np.array([detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z])
            q = np.array([detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w])
            
            T_cam_tag = SE3(t, q)
            T_base_tag = self.T_platform_cam.inverse() * T_cam_tag

            t = T_base_tag.translation()
            q = T_base_tag.quat().flatten()
    
            self.z.append([self.elapsed_time_april, t[0], t[1], t[2], q[0], q[1], q[2], q[3]])


    def Odometry_callback(self, odometry):
        if self.start_time == 0: self.start_time = odometry.header.stamp.to_sec()               # Start time of the rosbag
        self.elapsed_time = odometry.header.stamp.to_sec() - self.start_time        

        return              # Comment this line to start saving data
    
        if self.elapsed_time % 1 == 0 and self.elapsed_time != 0:
            print('Elapsed time: ', self.elapsed_time)
            print('Number of data points: ', len(self.x))
            print('Current position: ', self.x[-1])

        if self.elapsed_time > 18: 
            print('Saving data...')
            with open('x3_gt.npy', 'wb') as f:                          # Change the filename here to specify which rosbag file it is from: x1_gt.npy, x2_gt.npy, x3_gt.npy
                np.save(f, self.x)
            return

        translation = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z])
        orientation = np.array([odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w])
        orientation = orientation / np.linalg.norm(orientation)
        T_mocap_hornbill = SE3(translation, orientation)

        T_base_hornbill = self.T_mocap_base.inverse() * T_mocap_hornbill

        # Save the data
        t = T_base_hornbill.translation()
        q = T_base_hornbill.quat().flatten()
        self.x.append([self.elapsed_time, t[0], t[1], t[2], q[0], q[1], q[2], q[3]])



if __name__ == '__main__':
    rospy.init_node('parser')
    parser = parseClass()

    rospy.spin()
