import spatialmath as sm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Question2 import *


data = pd.read_csv('question2_input.csv', header=None,
                   names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])

# Initialize filter variables and inertial references
dt = 0.01
num_steps = len(data)

# initial orientation
current_orientation_quat = sm.UnitQuaternion()

# prev_time = data['t'][0] # Time step from the first entry, this will change dynamically in the loop  TODO decide to keep this or not
times = []
rotation_angles = []

# Process the sensor data and update the naive orientation estimator
for index, row in data.iterrows():

    # Extract current measurements
    t = row['t']
    gyro = np.array([row['gyrox'], row['gyroy'], row['gyroz']])

    # Update the orientation quaternion using our update function
    current_orientation_quat = updateQuaternion(current_orientation_quat, gyro, dt)

    # Store the current time and the rotation angle from the quaternion
    times.append(t)
    rotation_angles.append(quaternion_to_angle(current_orientation_quat))


plot_rotation_data(times, rotation_angles, 'using Naive Orientation Estimator', True, True)