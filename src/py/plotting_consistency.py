import numpy as np

import matplotlib.pyplot as plt
import scipy.stats

bag = 3
variant = 'full'                    # 'baseline', 'bigR', 'full'

# Read NEES and NES.npy files
nees = np.load('parse_data/NEES' + str(bag) + '_' + variant + '.npy').flatten()
nis = np.load('parse_data/NIS' + str(bag) + '_' + variant + '.npy').flatten()

# Calculate confidence interval for CHI squared distribution
confidence_interval = 0.95

neesDOF = 6
nisDOF = 6

aneesDOF = 6*len(nees)
anisDOF = 6*len(nis)

# Subplots
fig, axs = plt.subplots(2)
fig.suptitle('NEES and NIS for bag ' + str(bag) + ' with ' + variant + ' variant')

# Confidence intervals
conf_nees = scipy.stats.chi2.interval(confidence_interval, neesDOF)
conf_nis = scipy.stats.chi2.interval(confidence_interval, nisDOF)

conf_anees = scipy.stats.chi2.interval(confidence_interval, aneesDOF)
conf_anis = scipy.stats.chi2.interval(confidence_interval, anisDOF)

conf_anees = [conf_anees[0]/len(nees), conf_anees[1]/len(nees)]
conf_anis = [conf_anis[0]/len(nis), conf_anis[1]/len(nis)]

# Percentage inside
print('NEES: ', np.sum((nees > conf_nees[0]) & (nees < conf_nees[1]))/len(nees))
print('ANEES: ', np.sum((nees > conf_anees[0]) & (nees < conf_anees[1]))/len(nees))

print('NIS: ', np.sum((nis > conf_nis[0]) & (nis < conf_nis[1]))/len(nis))
print('ANIS: ', np.sum((nis > conf_anis[0]) & (nis < conf_anis[1]))/len(nis))

# Plot NEES
axs[0].plot(nees, label='NEES')
axs[0].plot([0, len(nees)], [conf_nees[0], conf_nees[0]], 'r--', label='Lower bound')
axs[0].plot([0, len(nees)], [conf_nees[1], conf_nees[1]], 'r--', label='Upper bound')
axs[0].plot([0, len(nees)], [conf_anees[0], conf_anees[0]], 'g--', label='Lower bound ANEES')
axs[0].plot([0, len(nees)], [conf_anees[1], conf_anees[1]], 'g--', label='Upper bound ANEES')
axs[0].set_title('NEES')
axs[0].legend()

# Plot NIS
axs[1].plot(nis, label='NIS')
axs[1].plot([0, len(nis)], [conf_nis[0], conf_nis[0]], 'r--', label='Lower bound')
axs[1].plot([0, len(nis)], [conf_nis[1], conf_nis[1]], 'r--', label='Upper bound')
axs[1].plot([0, len(nis)], [conf_anis[0], conf_anis[0]], 'g--', label='Lower bound ANIS')
axs[1].plot([0, len(nis)], [conf_anis[1], conf_anis[1]], 'g--', label='Upper bound ANIS')
axs[1].set_title('NIS')
axs[1].legend()

fig.set_size_inches(10, 10*3/4)

plt.show()





