
import numpy as np

from tqdm import tqdm

import matplotlib.pyplot as plt
import scipy.stats

from manifpy import SE3, SE3Tangent

from scipy.optimize import minimize

from funcs_plotting import *

printValues = True

def parse_estimated(params):
    bag = 3

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
            return nees, nis

        ############################################################
        # Model
        A = np.zeros((6,6))
        Q = np.diag(params[:6])

        Phi = scipy.linalg.expm(A*dt)

        # Measurement
        measurement = False
        interval = [time - dt, time]
        for j in range(len(trajectory_z)):
            if (interval[0] < trajectory_z[j, 0] < interval[1]):
                z = SE3(trajectory_z[j, 1:4], trajectory_z[j, 4:])
                R_rho = np.diag([params[6], params[7], params[8]])
                R_theta = np.diag([params[9], params[10], params[11]])
                R = np.block([[R_rho, np.zeros((3,3))], [np.zeros((3,3)), R_theta]])
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
        P = Phi @ P @ Phi.T + Phi @ Q @ Phi.T

        if measurement:
            # Kalman gain
            K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)

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


def consistency(params):
    nees, nis = parse_estimated(params)

    # Calculate confidence interval for CHI squared distribution
    confidence_interval = 0.95

    neesDOF = 6
    nisDOF = 6

    aneesDOF = 6*len(nees)
    anisDOF = 6*len(nis)

    # Confidence intervals
    conf_nees = scipy.stats.chi2.interval(confidence_interval, neesDOF)
    conf_nis = scipy.stats.chi2.interval(confidence_interval, nisDOF)

    conf_anees = scipy.stats.chi2.interval(confidence_interval, aneesDOF)
    conf_anis = scipy.stats.chi2.interval(confidence_interval, anisDOF)

    conf_anees = [conf_anees[0]/len(nees), conf_anees[1]/len(nees)]
    conf_anis = [conf_anis[0]/len(nis), conf_anis[1]/len(nis)]

    # Percentage inside
    nees_inside = np.sum((nees > conf_nees[0]) & (nees < conf_nees[1]))/len(nees)
    anees_inside = np.sum((nees > conf_anees[0]) & (nees < conf_anees[1]))/len(nees)

    nis_inside = np.sum((nis > conf_nis[0]) & (nis < conf_nis[1]))/len(nis)
    anis_inside = np.sum((nis > conf_anis[0]) & (nis < conf_anis[1]))/len(nis)

    if printValues:
        print("--------------------")
        print('NEES: ', nees_inside)
        print('ANEES: ', anees_inside)
        print('NIS: ', nis_inside)
        print('ANIS: ', anis_inside)

    return -(nees_inside + 10*anees_inside + nis_inside + 10*anis_inside)



if __name__ == '__main__':
    iter = 100
    if iter > 10: printValues = False

    # good initial values
    Q_var = 2
    R_rho_var = 5e-5
    R_theta_var = 5e-2

    best = -np.inf
    best_params = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    for i in tqdm(range(iter)):

        initial = []
        for i in range (12):
            initial.append(np.random.randint(1, 10)**np.random.randint(-5, 1))
                
        results = minimize(consistency, initial, method='Nelder-Mead', options={'maxiter': 50, 'xatol': 5e-2, 'fatol': 5e-2})

        best = max(best, -results.fun)
        if best == -results.fun:
            best_params = results.x

        if printValues:
            print("initial: ", initial)
            print("results: ", results.x)
            print("results.fun: ", -results.fun)
            print("results.nit: ", results.nit)
            print("results.success: ", results.success)
            print("results.message: ", results.message)

    print('Best: ', best)
    print('Best params: ', best_params)

    



