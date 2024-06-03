
import numpy as np
import scipy.linalg

from manifpy import SE3, SE3Tangent
from scipy.spatial.transform import Rotation as Rot

from funcs_plotting import *


if __name__ == '__main__':
   bag = 3
   variant = 'full'

   # Create the trajectories
   trajectory_gt = parse_data('parse_data/x' + str(bag) + '_gt.npy')
   trajectory_z = parse_data('parse_data/z' + str(bag) + '.npy')


   # State
   X_hat = []
   X = SE3(np.array([0,0,0]), np.array([0,0,0,1]))
   P = np.eye(6)

   # NEES and NIS data
   nees = []
   nis = []

   prev_time = 0

   first_measurement = True

   for i in range(len(trajectory_gt)):
      time = trajectory_gt[i, 0]
      dt = time - prev_time
      prev_time = trajectory_gt[i, 0]

      if (time < trajectory_z[0, 0]):  continue
      if (time > trajectory_z[-1, 0]): 
         print('saving...')
         with open('parse_data/x_hat' + str(bag) + '_' + variant + '.npy', 'wb') as f: np.save(f, X_hat)
         with open('parse_data/NEES' + str(bag) + '_' + variant + '.npy', 'wb') as f:  np.save(f, nees)
         with open('parse_data/NIS' + str(bag) + '_' + variant + '.npy', 'wb') as f:   np.save(f, nis)
         break

      ############################################################
      # Model
      A = np.zeros((6,6))
      #A = np.eye(6)

      # Initial guess
      # Q = np.eye(6)*1e-3
      # Good Q
      Q = np.eye(6)*1e+2



      # Different Q for different states
      # Q_rho = np.eye(3)*1e-2
      # Q_theta = np.eye(3)*1e-1
      # Q = np.block([[Q_rho, np.zeros((3,3))], [np.zeros((3,3)), Q_theta]])
      Q = np.diag([9.83781621e-01, 9.87770938e-01, 1.23086536e-01, 1.27864984e-01, 3.66761454e-02, 9.75984792e-01])

      Phi = scipy.linalg.expm(A*dt)

      # Measurement
      measurement = False
      interval = [time - dt, time]
      for j in range(len(trajectory_z)):
         if (interval[0] < trajectory_z[j, 0] < interval[1]):
            z = SE3(trajectory_z[j, 1:4], trajectory_z[j, 4:])

            # Initial guess
            # R = np.eye(6)*1e-3
            # Good R
            R = np.eye(6)*6e-1

            R_rho = np.diag([3.24521883e-03, 3.07886344e-01, 6.03611589e-02])
            R_theta = np.diag([1.13451345e+00, 2.39189818e-01, 1.03756851e-03])

            R = np.block([[R_rho, np.zeros((3,3))], [np.zeros((3,3)), R_theta]])

            


            # R from linear regression
            ################################
            # t_z = trajectory_z[j, 4]
            # a = np.array([0.000160825, 0.000157487, 0.00276888, 0, 0, 0])
            # b = np.array([-0.000413986, -0.00056063, -0.00813063, 0.0198355, 0.0217262, 0.000515574])
            # c = np.array([0.000248354, 0.000385233, 0.00508156, -0.00421625, 0.00300534, 0.0000290943])

            # R_diag = np.array([a[0]*t_z**2 + b[0]*t_z + c[0], a[1]*t_z**2 + b[1]*t_z + c[1], a[2]*t_z**2 + b[2]*t_z + c[2],
            #                                  b[3]*t_z + c[3],               b[4]*t_z + c[4],               b[5]*t_z + c[5]])
      
            # for i in range(6):
            #    if R_diag[i] < 5e-5:
            #       R_diag[i] = 5e-5

            # R = np.diag(R_diag**2)
            ################################

            measurement = True
            if first_measurement: 
               X = z
               P = R
               first_measurement = False
            break

      H = np.eye(6)

      ############################################################
      # Prediction
      
      X = X
      P = P + Q

      if measurement:
         # Kalman gain
         K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
         #K = np.eye(6)*0.8

         # Correction
         v = z - X
         X = X + K @ v
         P = (np.eye(6) - K @ H) @ P

      t = X.translation()
      quat = X.quat().flatten()
      if measurement:
         if not first_measurement: X_hat.append([time, t[0], t[1], t[2], quat[0], quat[1], quat[2], quat[3]])

      # NEES and NIS
      ############################################################
      X_GT = SE3(trajectory_gt[i, 1:4], trajectory_gt[i, 4:])
      eta = (X - X_GT).coeffs().reshape(-1, 1)
      if measurement:
         if not first_measurement: nees.append(eta.T @ np.linalg.inv(P) @ eta)
      
      if measurement:
         S = H @ P @ H.T + R
         v = v.coeffs().reshape(-1, 1)
         nis.append(v.T @ np.linalg.inv(S) @ v)
      
      
      
      

