# -*- coding: UTF-8 -*-

"""
Section the accelerations from the IMU and smartphone according to the starting and ending frames of walking sections of trajectories.
It reads the starting and ends frames stored by the script `extract_walkSection_trajectories.matlab´ in the 'raw_data' folder, and uses these frames to section the IMU accelerations, also stored in 'raw_data' folder.
These walking sections of IMU data are used to extract the walking sections from the smartphone accelerations by correlations analysis.
At the end, this script stores the walking sections from the IMU accelerations and the smartphone's accelerometer readings in the 'processed_data' folder of the 'A multi-sensor and cross-session human gait dataset'.


------- NEEDS TO DEFINE THE LOCAL PATH OF THE DATASET ----------
Set the constant DATASET_PATH


--------------------
Dependencies:
	- Pandas >= 0.25.1
	- Numpy >= 1.17.2
	- Scipy >= 1.3.1
--------------------

"""

import pandas as pd
import numpy as np
from glob import glob
from scipy import stats
import os



DATASET_PATH = ''

users = sorted(glob(DATASET_PATH+'raw_data/*/'))

for user in users:
	idxs = pd.read_csv(user+'idxs_frames.csv').to_numpy()
	for row in idxs:
		data = np.loadtxt(user+row[0]+'_imu.csv', delimiter=',', skiprows=1)
		new_data = data[np.where(data[:, 0]==row[1])[0][0]:np.where(data[:, 0]==row[2])[0][0]]
		df = pd.DataFrame(new_data[:, 1:], index=new_data[:, 0].astype(int), columns=['acc x', 'acc y', 'acc z'])
		df.index.name = 'frame'
		df.to_csv(user.replace('raw_data', 'processed_data')+row[0]+'_imu_walk.csv')


for user in users:
	files_nexus = sorted(glob(user+'*_nexus.csv'))
	for file in files_nexus:
		file_processed = file.replace('raw_data', 'processed_data')
		nexus_data = np.loadtxt(file, delimiter=',')
		imu_data = np.loadtxt(file_processed.replace('nexus', 'imu_walk'), delimiter=',', skiprows=1)
		acc_nexus = nexus_data[:, 2:]
		acc_imu = imu_data[:, 1:]
		idx_init = np.argmax(np.abs([stats.pearsonr(acc_imu[:, -1], acc_nexus[i:i+len(acc_imu), -1])[0] for i in range(len(acc_nexus)-len(acc_imu))]))
		walk_nexus = acc_nexus[idx_init:idx_init+len(acc_imu)]
		df = pd.DataFrame(walk_nexus, columns=['acc x', 'acc y', 'acc z'])
		df.to_csv(file_processed.replace('nexus', 'nexus_walk'), index=False)

