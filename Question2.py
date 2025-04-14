import spatialmath as sm
from spatialmath.base import skew, tr2angvec
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
import mujoco as mj
import mujoco.viewer
from scipy.spatial.transform import Rotation as R



# Helper Functions
def quaternion_to_angle(q):
    """
    Compute the rotation angle (in radians) from the UnitQuaternion q.
    We use the rotation matrix representation and SpatialMath’s tr2angvec.
    """
    ang, axis = tr2angvec(q.R)
    return ang

def plot_raw_data(m_corrected_array, csv_data):
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
        m = np.array(m_corrected_array[index])  # Use the corrected magnetometer data
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
    
class MahonyFilter:
    """
    Mahony Filter for attitude estimation.
    """
    def __init__(self, dT, kp=1.0, kI=0.3, ka_nominal=1.0, km_nominal=.5, true_m=np.array([0.087117, 0.37923, -0.92119])):
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kp
        self.kI = kI
        self.ka_nominal = ka_nominal
        self.km_nominal = km_nominal
        self.q = sm.UnitQuaternion()  # Initial orientation quaternion
        # self.q.data = [[.6265, -.0974, .0875, .7684]]
        self.bias = np.zeros(3)  # Initial gyroscope bias
        self.m0 = true_m  # True magnetic field vector (inertial frame)
        self.v_hat_a = np.array([0, 0, 1])  # Initial estimate of gravitational acceleration
        self.v_hat_m = true_m.copy()

    def updateQuaternion(self, q_old: sm.UnitQuaternion, u, **kwargs):
        """
        Update the given orientation quaternion q using the effective angular velocity u over time dt.
        This implements the closed-form update:
            q_{k+1} = exp((1/2)*Omega(u)*dt) * q_k.
        Here, q is a spatialmath UnitQuaternion and u is a 3-element vector.
        """
        norm_u = np.linalg.norm(u)
        theta = 0.5 * kwargs.get('time_step', self.dT)
        if norm_u < 1e-8: # divide by 0 prevention
            update_exp = np.eye(4)
        else:
            Omega_u = self.OMEGA(u)
            update_exp = np.cos(norm_u*theta)*np.eye(4) + (np.sin(norm_u*theta)/norm_u)*Omega_u

        # Multiply update matrix by current quaternion (represented as a 4x1 vector)
        new_q_data = update_exp @ np.array(q_old.data).reshape(4, 1) # need to reshape so matrix mult works correctly
        new_q_data = new_q_data.flatten() # converts back to 1D array
        return sm.UnitQuaternion(new_q_data)

    def OMEGA(self, omega):
        """
        Construct the 4x4 Omega matrix from a 3D angular velocity vector omega.
        """
        omega = np.asarray(omega).flatten() # always ensure it's a 1D array       
        if omega.shape != (3,):
            raise ValueError("Input omega must be a 3-element vector.")
        return np.block([
            [0,           -omega.reshape(1,3)],
            [omega.reshape(3,1), -skew(omega)]
        ])

    def rodrigues(self,u, **kwargs):
        """
        Compute the rotation matrix for a rotation of angle theta = ||u||*dt about axis u/||u||,
        using the Rodrigues formula.
        """
        norm_u = np.linalg.norm(u) 
        if norm_u < 1e-8: # prevent division by zero errors
            return np.eye(3)
        theta = norm_u * kwargs.get('time_step', self.dT)
        u_skew = skew(u)
        R = np.eye(3) + np.sin(theta)*(u_skew / norm_u) + (1 - np.cos(theta))*((u_skew @ u_skew) /( norm_u ** 2))
        return R


    def normalize(self, vector: np.ndarray)-> np.ndarray:
        if np.linalg.norm(vector) != 0:
            return vector / np.linalg.norm(vector)
        else:
            return vector

    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        """
        Perform a single time step of the Mahony filter update.
        This function takes in the current sensor measurements and updates the filter's state.

        Arguments:
            magnetometer_vector: 3D vector from the magnetometer (in inertial frame).
            gyro_vector: 3D vector from the gyroscope (in body frame).
            accel_vector: 3D vector from the accelerometer (in body frame).

        Returns:
            Updated orientation quaternion.
        """       
        time_step = kwargs.get('time_step', self.dT)  # Use the provided time step or the default one            
        v_a = self.normalize(accel_vector)

        # Normalize magnetometer measurements while subtracting gravity component
        #gravity terms
        
        g_body = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body) / (np.linalg.norm(g_body) ** 2)) * g_body
        m_corrected = magnetometer_vector - m_vertical 
        v_m = self.normalize(m_corrected)

        # Compute the error signals from cross products: Innovation:
        error_acc = np.cross(v_a, self.v_hat_a)
        error_mag = np.cross(v_m, self.v_hat_m)
        omega_mes = self.ka_nominal * error_acc + self.km_nominal * error_mag# FIXME we likly need to normalize the values used here, but there is some dought about it. 

        # Compute effective angular velocity for the update: 
        u = gyro_vector - self.bias + self.kp * omega_mes

        # Update the orientation quaternion using our update function
        self.q = self.updateQuaternion(self.q, u, time_step=time_step)

        # Update the gyroscope bias estimate
        self.bias = self.bias - self.kI * omega_mes * time_step

        # Update the estimated reference vectors using the Rodrigues formula
        R_update = self.rodrigues(-u, time_step=time_step)
        self.v_hat_a = R_update @ self.v_hat_a
        self.v_hat_m = R_update @ self.v_hat_m
        return self.q, m_corrected


if __name__ == "__main__":
    do_real_time_visualization = True # Set to True for real-time visualization, False for offline processing
    # Load the CSV file
    # csv_data = pd.read_csv('question2_input.csv', header=None,
    #                 names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])
    # csv_data = pd.read_csv('question3CustomCombined.csv', header=None,
    #                 names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])
    
    csv_data = pd.read_csv('charlie_phone_data.csv')
    csv_data = csv_data.rename(columns={"accelerometerAccelerationX(G)": "ax", "accelerometerAccelerationY(G)": "ay", "accelerometerAccelerationZ(G)": "az", "gyroRotationX(rad/s)": "gyrox", "gyroRotationY(rad/s)": "gyroy", "gyroRotationZ(rad/s)": "gyroz", "magnetometerX(µT)": "mx", "magnetometerY(µT)": "my", "magnetometerZ(µT)": "mz", "accelerometerTimestamp_sinceReboot(s)": "t"})
    csv_data[["ax", "ay", "az"]] = csv_data[["ax", "ay", "az"]] * 9.80665# accelerometer data is in G's not m/s^2
    csv_data["az"] = csv_data["az"] * -1# Need to flip z axis to match the coord system for this project. 

   
    # Expected values
    # expected_acc = 9.81
    # expected_mag = np.linalg.norm([4525.28449, 19699.18982, -47850.850686]) / 1000  # Convert nT to µT


    # prev_time = data['t'][0] # Time step from the first entry, this will change dynamically in the loop  TODO decide to keep this or not
    times = []
    rotation_angles = []
    m_corrected_array = [] # For graphing the corrected magnetometer data
    time_step = 0.01
    mahony_filter = MahonyFilter(dT=time_step, kp=1, kI=.3, ka_nominal=1, km_nominal=1)# dont' need to specify these values as I "conveniently" set them as the defaults, but they are here for code readability, except for dT, that is required. 
    
    

    if do_real_time_visualization:
        model = mj.MjModel.from_xml_path(r"phone.xml")
        mujoco_model_data = mj.MjData(model)
        with mujoco.viewer.launch_passive(model, mujoco_model_data) as viewer:
            viewer.cam.distance *= 300.0
            joint_name = "free_joint"
            joint_id = model.joint(joint_name).id
            qpos_addr = model.jnt_qposadr[joint_id]

            v_hat_name = "v_a_hat_joint"
            v_har_joint_id = model.joint(v_hat_name).id
            v_hat_addr = model.jnt_qposadr[v_har_joint_id]



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
                current_orientation_quat, corrected_mag_vector = mahony_filter.time_step(raw_mag_vector, raw_gyro_vector, raw_accel_vector)

                # Save the results. 
                times.append(curr_time)
                rotation_angles.append(quaternion_to_angle(current_orientation_quat))
                m_corrected_array.append(corrected_mag_vector)

                # Update the model with the new orientation
                time_simulated += time_step
                mujoco_model_data.qpos[qpos_addr:qpos_addr+3] = [0,0,0]
                mujoco_model_data.qpos[qpos_addr+3:qpos_addr+7] = current_orientation_quat.data[0]
              


                mujoco.mj_forward(model, mujoco_model_data)# This is called pre sleep so we use part of our time step to update the viewer, but this wont be been unil viewer.synyc() is called.
                # Calculate the time to sleep
                elasped_time = time.time() - start_time
                sleep_time = time_simulated - elasped_time# if this is negative it means that the calculations are taking longer than the time step they are simulating so the simulation will be delayed. 
                if sleep_time > 0:
                    time.sleep(sleep_time)# Sleep enough such that the real time elapsed matches the simlated time elapsed. 
                else:
                    print(f"Warning: Simulation is running behind schedule by{-sleep_time} seconds")
                viewer.sync()
                

    else:
        # Process the sensor data and update the Mahony filter
        for index, row in csv_data.iterrows():
            # Extract measurements from csv data. 
            curr_time = row['t']
            raw_mag_vector = np.array([row['mx'], row['my'], row['mz']])
            raw_gyro_vector = np.array([row['gyrox'], row['gyroy'], row['gyroz']])
            raw_accel_vector = np.array([row['ax'], row['ay'], row['az']])

            # Perform the calculations. 
            current_orientation_quat, corrected_mag_vector = mahony_filter.time_step(raw_mag_vector, raw_gyro_vector, raw_accel_vector)

            # Save the results. 
            times.append(curr_time)
            rotation_angles.append(quaternion_to_angle(current_orientation_quat))
            m_corrected_array.append(corrected_mag_vector)
   


            

    # Compute and print the total rotation at the end of the dataset
    plot_rotation_data(times, rotation_angles, title_suffix=" using Mahony Filter", data_in_radians=True, convert=True)
    # plot_raw_data(m_corrected_array)