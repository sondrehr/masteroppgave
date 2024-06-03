#!/usr/bin/python3

import rospy
import numpy as np
import scipy.linalg
from geometry_msgs.msg import PoseWithCovarianceStamped
from precision_landing.srv import estimate

from sensor_msgs.msg import Imu

from manifpy import SE3, SE3Tangent
from scipy.spatial.transform import Rotation as Rot


class estimatorClass():
   prevTime = 0

   x = SE3(np.array([0,0,0]), Rot.from_euler('xyz', [0,0,0]).as_quat())
   P = np.eye(6)*10

   def __init__(self):
      self.srv = rospy.Service('estimate_state', estimate, self.measurement_callback)
      self.sub = rospy.Subscriber('mavros/imu', Imu, self.IMU_callback)
      self.prevTime = rospy.Time.now().to_nsec()

   def IMU_callback(self, IMU):
      dt = (rospy.Time.now().to_nsec() - self.prevTime)/1e9
      self.prevTime = rospy.Time.now().to_nsec()
      
      ## Add IMU data to state estimation and remove prediction step in measurement_callback

      pass

   def measurement_callback(self, Pose):      

      dt = (rospy.Time.now().to_nsec() - self.prevTime)/1e9
      self.prevTime = rospy.Time.now().to_nsec()

      #Get measurement z, R
      t = np.array([Pose.poseIn.pose.pose.position.x, Pose.poseIn.pose.pose.position.y, Pose.poseIn.pose.pose.position.z])
      quat = np.array([Pose.poseIn.pose.pose.orientation.x, Pose.poseIn.pose.pose.orientation.y, Pose.poseIn.pose.pose.orientation.z, Pose.poseIn.pose.pose.orientation.w])

      z = SE3(t, quat)
      R = np.diag([1.14e-4**2, 9.76e-5**2, 2.41e-4**2, 5.01e-2**2, 4.99e-2**2, 1.4e-3**2])      # Constant R

      # polynomial regression for R wrt. t[2]
      a = np.array([0.000160825, 0.000157487, 0.00276888, 0, 0, 0])
      b = np.array([-0.000413986, -0.00056063, -0.00813063, 0.0198355, 0.0217262, 0.000515574])
      c = np.array([0.000248354, 0.000385233, 0.00508156, -0.00421625, 0.00300534, 0.0000290943])

      R_diag = np.array([a[0]*t[2]**2 + b[0]*t[2] + c[0], a[1]*t[2]**2 + b[1]*t[2] + c[1], a[2]*t[2]**2 + b[2]*t[2] + c[2],
                                        b[3]*t[2] + c[3],                b[4]*t[2] + c[4],                b[5]*t[2] + c[5]])
      
      for i in range(6):
         if R_diag[i] < 5e-5:
            R_diag[i] = 5e-5

      R = np.diag(R_diag**2)

      print("R: ", R)

      #Prediction
      ############################################################
      A = np.eye(6)
      Q = np.eye(6)*1e-9

      # matrix exponential
      phi = scipy.linalg.expm(A*dt)

      x_pred = self.x
      P_pred = phi @ self.P @ phi.T + phi @ Q @ phi.T             # P and Q time varying
      # P_pred = phi @ self.P @ phi.T                               # P time varying = time varying K
      # P_pred = self.P*1.5                                         # P not time varying = static K

      #Kalman Gain
      ############################################################
      H = np.eye(6)

      # K = np.eye(6)*0.33                                          # static K
      # alpha = (1 - 0.85*np.exp(-dt/2))
      # K = np.eye(6)*alpha                                         # time varying K
      K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)      # optimal K

      #Correction
      ############################################################
      self.x = x_pred + K @ (z - x_pred)
      self.P = (np.eye(6) - K @ H) @ P_pred
      

      print("K: ", K)
      print("P_pred: ", P_pred)

      #Return x
      poseOut = PoseWithCovarianceStamped()
      poseOut.header.stamp = rospy.Time.now()
      poseOut.pose.pose.position.x = self.x.translation()[0]
      poseOut.pose.pose.position.y = self.x.translation()[1]
      poseOut.pose.pose.position.z = self.x.translation()[2]
      poseOut.pose.pose.orientation.x = self.x.quat()[0]
      poseOut.pose.pose.orientation.y = self.x.quat()[1]
      poseOut.pose.pose.orientation.z = self.x.quat()[2]
      poseOut.pose.pose.orientation.w = self.x.quat()[3]
      poseOut.pose.covariance = self.P.flatten()

      # return Pose.poseIn                                          # if no state estimation
      return poseOut


if __name__ == '__main__':
   rospy.init_node('pose_estimator')
   estimator = estimatorClass()

   rospy.spin()