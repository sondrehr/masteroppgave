
import numpy as np

from manifpy import SE3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def parse_data(path): 
    return np.load(path) 

def add_trajectories(ax, trajectories):

    for i in range(len(trajectories)):
        x, y, z = trajectories[i]
        if i == 0:  ax.plot(x, y, z, 'C0', label='Ground Truth')
        else:       ax.plot(x, y, z, 'C1', label='Estimated')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


def truncate_gt(trajectory_gt, trajectory):
    trajectory_gt_trunc = []

    for i in range(len(trajectory)):
        time = trajectory[i, 0]
        idx = np.argmin(np.abs(trajectory_gt[:, 0] - time))
        trajectory_gt_trunc.append(trajectory_gt[idx])

    return np.array(trajectory_gt_trunc)


def add_lines(ax, trajectory_gt, trajectory):
    if len(trajectory_gt) == 8:     # Single point used in RTE
        ax.plot([trajectory_gt[1], trajectory[1]], [trajectory_gt[2], trajectory[2]], [trajectory_gt[3], trajectory[3]], 'r')
        return
    for i in range(len(trajectory)):
        x = [trajectory_gt[i, 1], trajectory[i, 1]]
        y = [trajectory_gt[i, 2], trajectory[i, 2]]
        z = [trajectory_gt[i, 3], trajectory[i, 3]]
        ax.plot(x, y, z, 'r')

        
def align_trajectories(trajectory_gt, trajectory_z):
    trajectory_z_aligned = []

    t_gt = np.array([trajectory_gt[0, 1], trajectory_gt[0, 2], trajectory_gt[0, 3]])
    q_gt = np.array([trajectory_gt[0, 4], trajectory_gt[0, 5], trajectory_gt[0, 6], trajectory_gt[0, 7]])
    T_gt = SE3(t_gt, q_gt)

    t_z = np.array([trajectory_z[0, 1], trajectory_z[0, 2], trajectory_z[0, 3]])
    q_z = np.array([trajectory_z[0, 4], trajectory_z[0, 5], trajectory_z[0, 6], trajectory_z[0, 7]])
    T_z = SE3(t_z, q_z)

    dT = T_gt - T_z

    for i in range(len(trajectory_z)):
        t_z = np.array([trajectory_z[i, 1], trajectory_z[i, 2], trajectory_z[i, 3]])
        q_z = np.array([trajectory_z[i, 4], trajectory_z[i, 5], trajectory_z[i, 6], trajectory_z[i, 7]])
        T_z = SE3(t_z, q_z)

        T_z_aligned = T_z + dT
        t_z_aligned = T_z_aligned.translation()
        q_z_aligned = T_z_aligned.quat().flatten()

        trajectory_z_aligned.append([trajectory_z[i, 0], t_z_aligned[0], t_z_aligned[1], t_z_aligned[2], q_z_aligned[0], q_z_aligned[1], q_z_aligned[2], q_z_aligned[3]])

    return np.array(trajectory_z_aligned)



def ATE(ax, trajectories_to_plot, trajectory_gt, trajectory_z):
    # Align the trajectories
    trajectory_z_aligned = align_trajectories(trajectory_gt, trajectory_z)
    
    # Add the trajectories to the plot
    trajectories_to_plot.append([trajectory_z_aligned[:, 1], trajectory_z_aligned[:, 2], trajectory_z_aligned[:, 3]])
    add_lines(ax, trajectory_gt, trajectory_z_aligned)

    # Calculate the error
    errors = []
    for i in range(len(trajectory_gt)):
        errors.append(error_func(trajectory_gt[i], trajectory_z_aligned[i]))
    
    print('ATE rms: ', np.sqrt(np.mean(np.square(errors))))
    print('Max error: ', np.max(errors))

    return errors


def RTE(ax, trajectories_to_plot, trajectory_gt, trajectory_z, segment_length, segments):
    
    errors = []
    for i in range(segments):
        start = np.random.randint(0, len(trajectory_gt) - segment_length)

        # Get the segments
        trajectory_gt_segment = trajectory_gt[start:start+segment_length]
        trajectory_z_segment = trajectory_z[start:start+segment_length]

        # Align the segments
        trajectory_z_segment_aligned = align_trajectories(trajectory_gt_segment, trajectory_z_segment)

        # Add the segments to the plot
        trajectories_to_plot.append([trajectory_z_segment_aligned[:, 1], trajectory_z_segment_aligned[:, 2], trajectory_z_segment_aligned[:, 3]])
        add_lines(ax, trajectory_gt_segment[-1], trajectory_z_segment_aligned[-1])

        # Calculate the error
        errors.append(error_func(trajectory_gt_segment[-1], trajectory_z_segment_aligned[-1]))
    
    print('--- Segment length: ', segment_length, ' ---')
    print('Max error: ', np.max(errors))
    print('Min error: ', np.min(errors))
    print('Mean error: ', np.mean(errors))
    print('Std error: ', np.std(errors))
    return errors

def error_func(x, y):
    t_x = np.array([x[1], x[2], x[3]])
    q_x = np.array([x[4], x[5], x[6], x[7]])
    T_x = SE3(t_x, q_x)

    t_y = np.array([y[1], y[2], y[3]])
    q_y = np.array([y[4], y[5], y[6], y[7]])
    T_y = SE3(t_y, q_y)

    dT = (T_x - T_y).coeffs()
    return np.linalg.norm(dT)