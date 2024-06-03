
import numpy as np
from scipy.stats import norm
from matplotlib.pyplot import boxplot

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_interactions import ioff, panhandler, zoom_factory

from geometry_msgs.msg import PoseWithCovarianceStamped

from manifpy import SE3

from funcs_plotting import *




if __name__ == '__main__':
    ### Parameters. Change these to plot different data
    bag = 3
    variant = 'full'        # 'baseline', 'P_growing', 'R_of_time', 'Q', 
    traj = 'estimate'           # 'estimate', 'measurement'
    metric = 'RTE'              # 'RTE', 'ATE', or ''
    removeOutliersATEPlot = False
    segment_length = [5, 10, 20, 30, 50]
    segments = 10

    ### Create the figure
    fig = plt.figure()
    fig.set_size_inches(10, 10*3/4)
    ax = fig.add_subplot(projection='3d')

    ### Create the trajectories
    trajectory_gt = parse_data('parse_data/x' + str(bag) + '_gt.npy')
    if traj == 'estimate':    trajectory = parse_data('parse_data/x_hat' + str(bag) + '_' + variant + '.npy')
    if traj == 'measurement': trajectory = parse_data('parse_data/z' + str(bag) + '.npy')
    trajectory_gt_trunc = truncate_gt(trajectory_gt, trajectory)

    ### Add the trajectories to the plot
    trajectories = []
    trajectories.append([trajectory_gt[:, 1], trajectory_gt[:, 2], trajectory_gt[:, 3]])

    if metric == 'RTE': 
        errors = []
        for i in range(len(segment_length)):
            errors.append(RTE(ax, trajectories, trajectory_gt_trunc, trajectory, segment_length[i], segments))
    if metric == 'ATE': 
        errors = ATE(ax, trajectories, trajectory_gt_trunc, trajectory)
    if metric == '':
        trajectories.append([trajectory[:, 1], trajectory[:, 2], trajectory[:, 3]])
        add_lines(ax, trajectory_gt_trunc, trajectory)

    add_trajectories(ax, trajectories)     
    plt.show()


    #plot histogram of ATE errors
    if metric == 'ATE':
        if removeOutliersATEPlot:
            errors = np.array(errors)
            errors = errors[errors < 0.15]  
    
        mu, std = norm.fit(errors)
        print('Mean: ', mu)
        print('Std: ', std)

        plt.hist(errors, bins=50, density=True, alpha=0.6, color='C0')
        xmin, xmax = plt.xlim()
        x = np.linspace(xmin, xmax, 100)
        p = norm.pdf(x, mu, std)
        plt.plot(x, p, 'k', linewidth=2)
        title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
        plt.axvline(x=mu, color='r', linestyle='--')
        plt.title(title)

        plt.show()
    
    #plot boxplot of RTE errors
    if metric == 'RTE':
        errors = np.array(errors).T
        boxplot(errors, labels=segment_length)
        plt.show()
        


        
