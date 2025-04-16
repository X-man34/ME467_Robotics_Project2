import spatialmath as sm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Question2 import *
from filters import *
from Question3 import plot_error_estimates, plot_euler_angles
from pathlib import Path
import threading



if __name__ == "__main__":
    #Question 4 part d

    data_path = Path("csv_files") / "charlie_phone_540.csv"
    csv_data = pd.read_csv(str(data_path))
    csv_data = csv_data.rename(columns={"accelerometerAccelerationX(G)": "ax", "accelerometerAccelerationY(G)": "ay", "accelerometerAccelerationZ(G)": "az", "gyroRotationX(rad/s)": "gyrox", "gyroRotationY(rad/s)": "gyroy", "gyroRotationZ(rad/s)": "gyroz", "magnetometerX(µT)": "mx", "magnetometerY(µT)": "my", "magnetometerZ(µT)": "mz", "accelerometerTimestamp_sinceReboot(s)": "t"})
    csv_data[["ax", "ay", "az"]] = csv_data[["ax", "ay", "az"]] * 9.80665# accelerometer data is in G's not m/s^2
    csv_data["az"] = csv_data["az"] * -1# Need to flip z axis to match the coord system for this project. 

    dt = 0.01

    times = []
    rotation_angles = []

    triad_estimator = TriadEstimator(dt, kP=2)
    # Perfrom the simulation and get the data
    times, rotation_angles, bias_estimates, error_estimates, roll, pitch, yaw = simulate_and_visualize_data(csv_data, .01, triad_estimator, do_3D_vis=True, show_body_coords=False, show_extra_vectors=True, show_spatial_coords=True)

    # Plot the euler angles
    plot_euler_angles(times, roll, pitch, yaw)

    plot_error_estimates(times, error_estimates)

    naive_estimator = NaiveEstimator(dt)
    # Perfrom the simulation and get the data
    times, rotation_angles, bias_estimates, error_estimates, roll, pitch, yaw = simulate_and_visualize_data(csv_data, dt, naive_estimator,  do_3D_vis=True, show_body_coords=False, show_extra_vectors=False, show_spatial_coords=True)

    #Plot the data. 
    plot_euler_angles(times, roll, pitch, yaw)

    plot_error_estimates(times, error_estimates)