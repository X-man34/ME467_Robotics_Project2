import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from visualizer import simulate_and_visualize_data

def plot_euler_angles(times, roll, pitch, yaw):
    plt.figure(figsize=(10,6))
    plt.plot(times, roll, label='Roll (deg)')
    plt.plot(times, pitch, label='Pitch (deg)')
    plt.plot(times, yaw, label='Yaw (deg)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Roll, Pitch, Yaw vs Time')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

def plot_error_estimates(times, error_estimates):
    plt.figure(figsize=(10,6))
    plt.plot(times, error_estimates, label='Estimated Error')
    plt.xlabel('Time (s)')
    plt.ylabel("Error")
    plt.title('Estimated Error vs Time')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()


def plot_gyro_bias(times, biases):
    plt.figure(figsize=(10,6))
    plt.plot(times, biases, label='Bias')
    plt.xlabel('Time (s)')
    plt.ylabel("Gyro Bias")
    plt.title('Estimated Gyro Bias vs Time')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":

    # Load pre recorded CSV data
    #TODO load data live and have a real time estimate of what is going on. 
    csv_data = pd.read_csv('csv_files\charlie_phone_data.csv')
    csv_data = csv_data.rename(columns={"accelerometerAccelerationX(G)": "ax", "accelerometerAccelerationY(G)": "ay", "accelerometerAccelerationZ(G)": "az", "gyroRotationX(rad/s)": "gyrox", "gyroRotationY(rad/s)": "gyroy", "gyroRotationZ(rad/s)": "gyroz", "magnetometerX(µT)": "mx", "magnetometerY(µT)": "my", "magnetometerZ(µT)": "mz", "accelerometerTimestamp_sinceReboot(s)": "t"})
    csv_data[["ax", "ay", "az"]] = csv_data[["ax", "ay", "az"]] * 9.80665# accelerometer data is in G's not m/s^2
    csv_data["az"] = csv_data["az"] * -1# Need to flip z axis to match the coord system for this project. 
    

    
    times, rotation_angles, bias_estimates, error_estimates, roll, pitch, yaw = simulate_and_visualize_data(csv_data, .01,kp=1, kI=.3, ka_nominal=1, km_nominal=1, do_3D_vis=True, show_body_coords=True, show_extra_vectors=True, show_spatial_coords=True)
    

    # Plot the euler angles
    plot_euler_angles(times, roll, pitch, yaw)

    plot_error_estimates(times, error_estimates)

    plot_gyro_bias(times, bias_estimates)
    


