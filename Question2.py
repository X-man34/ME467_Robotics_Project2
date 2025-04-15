import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from visualizer import simulate_and_visualize_data
from filters import MahonyFilter

def plot_raw_data(csv_data: pd.DataFrame):
    # Define percent values for each sensor
    percent_value_acc = 0.12   
    percent_value_mag = 0.10  

    # Expected values
    expected_acc = 9.81
    expected_mag = np.linalg.norm([4525.28449, 19699.18982, -47850.850686]) / 1000  # Convert nT to µT

    # Calculate upper and lower bounds for accelerometer
    acc_upper = (1 + percent_value_acc) * expected_acc
    acc_lower = (1 - percent_value_acc) * expected_acc

    # Calculate upper and lower bounds for magnetometer
    mag_upper = (1 + percent_value_mag) * expected_mag
    mag_lower = (1 - percent_value_mag) * expected_mag

    # Compute the norms of the accelerometer and magnetometer readings over time
    times_raw = []
    acc_norms = []
    mag_norms = []

    for index, row in csv_data.iterrows():
        times_raw.append(row['t'])
        a = np.array([row['ax'], row['ay'], row['az']])
        m = np.array([row['mx'], row['my'], row['mz']])
        acc_norms.append(np.linalg.norm(a))
        mag_norms.append(np.linalg.norm(m))
        # print(m)
        # print(np.linalg.norm(m))

    # Create separate plots for accelerometer and magnetometer with default y-limits
    # Accelerometer plot
    plt.figure(figsize=(10, 5))
    plt.plot(times_raw, acc_norms, label='Accelerometer Norm')
    plt.axhline(y=expected_acc, color='red', linestyle='--', label='Expected Acc Norm')
    plt.axhline(y=acc_upper, color='red', linestyle=':', label=f'Acc +{int(percent_value_acc*100)}%')
    plt.axhline(y=acc_lower, color='red', linestyle=':', label=f'Acc -{int(percent_value_acc*100)}%')
    plt.xlabel('Time (s)')
    plt.ylabel('Norm (m/s²)')
    plt.title('Accelerometer Norms vs. Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Magnetometer plot
    plt.figure(figsize=(10, 5))
    plt.plot(times_raw, mag_norms, label='Magnetometer Norm')
    plt.axhline(y=expected_mag, color='green', linestyle='--', label='Expected Mag Norm')
    plt.axhline(y=mag_upper, color='green', linestyle=':', label=f'Mag +{int(percent_value_mag*100)}%')
    plt.axhline(y=mag_lower, color='green', linestyle=':', label=f'Mag -{int(percent_value_mag*100)}%')
    plt.xlabel('Time (s)')
    plt.ylabel('Norm (µT)')
    plt.title('Magnetometer Norms vs. Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    print(times_raw[len(times_raw)-650])

    sigma_a = np.std(acc_norms[len(times_raw)-650:-1])  # standard deviation of accelerometer norms
    sigma_m = np.std(mag_norms[len(times_raw)-650:-1])  # standard deviation of magnetometer norms

    print("sigma_a during rest period:", sigma_a)
    print("sigma_m during rest period:", sigma_m)

    print()

    sigma_a = np.std(acc_norms)  # standard deviation of accelerometer norms
    sigma_m = np.std(mag_norms)  # standard deviation of magnetometer norms

    print("sigma_a during entire period:", sigma_a)
    print("sigma_m during entire period:", sigma_m)

def plot_rotation_data(times, rotation_angles, title_suffix="", data_in_radians=True, convert=False):
    """
    Plots rotation angle vs. time on a single plot and displays the total rotation on the figure
    (outside of the main data plot area), while also printing it to the console.

    Parameters:
        times (array-like): Time data (in seconds).
        rotation_angles (array-like): Rotation angle data.
            The unit of the rotation data is determined by the `data_in_radians` flag.
        title_suffix (str): Suffix to be appended to the plot title.
        data_in_radians (bool): True if the provided rotation_angles are in radians; 
                                False if they are in degrees.
        convert (bool, optional): If True, converts the rotation data to the other unit 
                                  (radians -> degrees or degrees -> radians) before plotting.
                                  Default is False.
    """
    # Determine the original and conversion units
    orig_unit = 'radians' if data_in_radians else 'degrees'
    conv_unit = 'degrees' if data_in_radians else 'radians'
    
    # Choose the data to plot based on whether conversion is needed
    if convert:
        if data_in_radians:
            # Convert from radians to degrees
            rotation_data = np.degrees(rotation_angles)
        else:
            # Convert from degrees to radians
            rotation_data = np.radians(rotation_angles)
        display_unit = conv_unit
    else:
        rotation_data = rotation_angles
        display_unit = orig_unit

    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, rotation_data, label=f'Rotation Angle ({display_unit})')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(f'Rotation Angle ({display_unit})')
    ax.set_title(f'Rotation Angle vs. Time {title_suffix}')
    ax.legend()
    ax.grid(True)

    # Compute total rotation based on the plotted data
    total_rotation = np.sum(rotation_data)
    textstr = f"Total rotation over the recorded period: {total_rotation:.3f} {display_unit}"
    
    # Add figure-level text that appears outside the main axes area (upper-right corner of the entire figure)
    plt.figtext(0.98, 0.98, textstr, horizontalalignment='right', verticalalignment='top',
                fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.5))

    #  Print the total rotation to the console
    print(f"Total rotation over the recorded period: {total_rotation:.3f} {display_unit}")    
    plt.show()

if __name__ == "__main__":
    # Load the CSV file
    csv_data = pd.read_csv('csv_files\question2_input.csv', header=None,
                    names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])
    
    mahony_filter = MahonyFilter(dT=.01, kp=1.0, kI=0.3, ka_nominal=.8, km_nominal=.2)
    # Perfrom the simulation and get the data
    times, rotation_angles, _, _, _, _, _ = simulate_and_visualize_data(csv_data, 0.01, mahony_filter,  do_3D_vis=True, show_body_coords=False, show_extra_vectors=True, show_spatial_coords=True)

    #Plot the data. 
    plot_rotation_data(times, rotation_angles, title_suffix=" using Mahony Filter", data_in_radians=True, convert=False)
    # plot_raw_data(csv_data)