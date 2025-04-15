import spatialmath as sm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Question2 import *
from filters import *
from Question3 import plot_error_estimates, plot_euler_angles



if __name__ == "__main__":
    #Question 4 part d

    # csv_data = pd.read_csv('csv_files\question2_input.csv', header=None,
    #                 names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])

    csv_data = pd.read_csv('csv_files\charlie_phone_data.csv')
    csv_data = csv_data.rename(columns={"accelerometerAccelerationX(G)": "ax", "accelerometerAccelerationY(G)": "ay", "accelerometerAccelerationZ(G)": "az", "gyroRotationX(rad/s)": "gyrox", "gyroRotationY(rad/s)": "gyroy", "gyroRotationZ(rad/s)": "gyroz", "magnetometerX(µT)": "mx", "magnetometerY(µT)": "my", "magnetometerZ(µT)": "mz", "accelerometerTimestamp_sinceReboot(s)": "t"})
    csv_data[["ax", "ay", "az"]] = csv_data[["ax", "ay", "az"]] * 9.80665# accelerometer data is in G's not m/s^2
    csv_data["az"] = csv_data["az"] * -1# Need to flip z axis to match the coord system for this project. 

    dt = 0.01

    times = []
    rotation_angles = []

    triad_estimator = TriadEstimator(dt, kP=2)
    # Perfrom the simulation and get the data
    times, rotation_angles, bias_estimates, error_estimates, roll, pitch, yaw = simulate_and_visualize_data(csv_data, .01, triad_estimator, do_3D_vis=True, show_body_coords=True, show_extra_vectors=True, show_spatial_coords=True)

    # Plot the euler angles
    plot_euler_angles(times, roll, pitch, yaw)

    plot_error_estimates(times, error_estimates)
    
