import spatialmath as sm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Question2 import *
from filters import *


from spatialmath.base import skew, tr2angvec
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
import mujoco as mj
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
from filters import MahonyFilter
from spatialmath import SO3


csv_data = pd.read_csv('csv_files\question2_input.csv', header=None,
                   names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])


# csv_data = pd.read_csv('csv_files\charlie_phone_data.csv')
# csv_data = csv_data.rename(columns={"accelerometerAccelerationX(G)": "ax", "accelerometerAccelerationY(G)": "ay", "accelerometerAccelerationZ(G)": "az", "gyroRotationX(rad/s)": "gyrox", "gyroRotationY(rad/s)": "gyroy", "gyroRotationZ(rad/s)": "gyroz", "magnetometerX(µT)": "mx", "magnetometerY(µT)": "my", "magnetometerZ(µT)": "mz", "accelerometerTimestamp_sinceReboot(s)": "t"})
# csv_data[["ax", "ay", "az"]] = csv_data[["ax", "ay", "az"]] * 9.80665# accelerometer data is in G's not m/s^2
# csv_data["az"] = csv_data["az"] * -1# Need to flip z axis to match the coord system for this project. 

dt = 0.01

times = []
rotation_angles = []

triad_estimator = TriadEstimator(dt, kI=.3, kP=2, kA=1.0, kM=.5)
# Perfrom the simulation and get the data
times, rotation_angles, _, _, _, _, _ = simulate_and_visualize_data(csv_data, dt, triad_estimator,  do_3D_vis=True, show_body_coords=False, show_extra_vectors=True, show_spatial_coords=True)

#Plot the data. 
plot_rotation_data(times, rotation_angles, title_suffix=" using Naive Orientation Estimator", data_in_radians=True, convert=False)
