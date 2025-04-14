
from spatialmath.base import skew, tr2angvec
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
import mujoco as mj
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
from filters import MahonyFilter


# for plotting v_a_hat etc
def get_quat_from_vec(v_spatial, negate_z=False)-> np.ndarray:
    # Normalize
    v_spatial = v_spatial / np.linalg.norm(v_spatial)

    # Create a quaternion that rotates z-axis to v_spatial
    if negate_z:
        z_axis = np.array([0, 0, -1])# z is negated because thats how we are defining it. 
    else:
        z_axis = np.array([0, 0, 1])
    
    rot_axis = np.cross(z_axis, v_spatial)
    angle = np.arccos(np.clip(np.dot(z_axis, v_spatial), -1.0, 1.0))

    if np.linalg.norm(rot_axis) < 1e-6:
        # Handle edge cases: already aligned or opposite
        if np.dot(z_axis, v_spatial) > 0:
            quat_v = np.array([1, 0, 0, 0])  # identity
        else:
            quat_v = np.array([0, 1, 0, 0])  # 180 deg around X (arbitrary orthogonal axis)
    else:
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        quat_v = R.from_rotvec(angle * rot_axis).as_quat()  # [x, y, z, w]

    # MuJoCo wants [w, x, y, z]
    return np.roll(quat_v, 1)

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
    
    #Grab the initial values
    row = csv_data.iloc[0]
    raw_mag_vector = np.array([row['mx'], row['my'], row['mz']])
    raw_accel_vector = np.array([row['ax'], row['ay'], row['az']])
    
    #Initialize variables
    times = []
    rotation_angles = []
    time_step = 0.01

    # Initialize the filter. This is where you change the gains. You don't have to pass in initial conditions, but it improves the estimate. 
    # You can also ask it to use the TRIAD initial pose estimatior, but at the time of writing the implementation does not work and its not asked for question 2, so its left disabled. 
    mahony_filter = MahonyFilter(dT=time_step, kp=1, kI=.3, ka_nominal=0, km_nominal=1, use_TRIAD_initial_attitude_estimation=False, init_conditions=(raw_accel_vector, raw_mag_vector))# dont' need to specify these values as I "conveniently" set them as the defaults, but they are here for code readability, except for dT, that is required. 
    
    do_3D_vis = True
    #Set up 3D visualization
    model = mj.MjModel.from_xml_path(r"resources\\phone.xml")
    mujoco_model_data = mj.MjData(model)
    # To not repeat code, if for some reason you don't want to watch the awsome visualization and just want to see the boring plots, it will open the viewer and them immediately close it. 
    # I just don't know how else to do it without repeating the for loop outside the with statement. 
    with mujoco.viewer.launch_passive(model, mujoco_model_data) as viewer:
        if not do_3D_vis:
            viewer.close()
        viewer.cam.distance *= 300.0
        joint_name = "free_joint"
        joint_id = model.joint(joint_name).id
        qpos_addr = model.jnt_qposadr[joint_id]

        v_a_hat_joint_name = "v_a_hat_joint"
        v_a_hat_joint_id = model.joint(v_a_hat_joint_name).id
        qpos_addr_v_a_hat = model.jnt_qposadr[v_a_hat_joint_id]

        v_m_hat_joint_name = "v_m_hat_joint"
        v_m_hat_joint_id = model.joint(v_m_hat_joint_name).id
        qpos_addr_v_m_hat = model.jnt_qposadr[v_m_hat_joint_id]

        start_time = time.time()
        time_simulated = 0.0
        # Process the sensor data and update the Mahony filter
        for index, row in csv_data.iterrows():
            # Extract measurements from csv data. 
            curr_time = row['t']
            raw_mag_vector = np.array([row['mx'], row['my'], row['mz']])
            raw_gyro_vector = np.array([row['gyrox'], row['gyroy'], row['gyroz']])
            raw_accel_vector = np.array([row['ax'], row['ay'], row['az']])

            # Perform the calculations. 
            current_orientation_quat = mahony_filter.time_step(raw_mag_vector, raw_gyro_vector, raw_accel_vector)
            # Save the results. 
            times.append(curr_time)
            rotation_angles.append(tr2angvec(current_orientation_quat.R)[0])
            if do_3D_vis:
                # Update the model with the new orientation
                #phone mesh
                time_simulated += time_step
                mujoco_model_data.qpos[qpos_addr:qpos_addr+3] = [0,0,0]
                mujoco_model_data.qpos[qpos_addr+3:qpos_addr+7] = current_orientation_quat.data[0]


                # TODO more fun visualization, it seems like v_hat_m is not pointing in the right direction. 
                # estimate vectors
                # mujoco_model_data.qpos[qpos_addr_v_a_hat:qpos_addr_v_a_hat+4] = get_quat_from_vec(mahony_filter.v_hat_a)
                mujoco_model_data.qpos[qpos_addr_v_m_hat:qpos_addr_v_m_hat+4] = get_quat_from_vec(mahony_filter.v_hat_m, negate_z=False)
                mujoco_model_data.qpos[qpos_addr_v_a_hat:qpos_addr_v_a_hat+4] = get_quat_from_vec(mahony_filter.m_corrected, negate_z=False)
                mujoco.mj_forward(model, mujoco_model_data)# This is called pre sleep so we use part of our time step to update the viewer, but this wont be been unil viewer.synyc() is called.
                
                if not viewer.is_running():
                    print("Viewer is closed. Exiting...")
                    break
                    
                # Calculate the time to sleep
                elasped_time = time.time() - start_time
                sleep_time = time_simulated - elasped_time# if this is negative it means that the calculations are taking longer than the time step they are simulating so the simulation will be delayed. 
                if sleep_time > 0:
                    time.sleep(sleep_time)# Sleep enough such that the real time elapsed matches the simlated time elapsed. 
                else:
                    print(f"Warning, simulation is delayed: {-sleep_time * 1000:.2f} ms")
                viewer.sync()

    # Compute and print the total rotation at the end of the dataset
    plot_rotation_data(times, rotation_angles, title_suffix=" using Mahony Filter", data_in_radians=True, convert=True)
    # plot_raw_data(csv_data)