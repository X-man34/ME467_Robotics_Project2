import spatialmath as sm
from spatialmath.base import skew, tr2angvec
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# Load the CSV file
data = pd.read_csv('question2_input.csv', header=None,
                   names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])


# Helper Functions
def updateQuaternion(q, u, dt):
    """
    Update the orientation quaternion q using the effective angular velocity u over time dt.
    This implements the closed-form update:
        q_{k+1} = exp((1/2)*Omega(u)*dt) * q_k.
    Here, q is a spatialmath UnitQuaternion and u is a 3-element vector.
    """
    norm_u = np.linalg.norm(u)
    theta = 0.5 * dt
    if norm_u < 1e-8: # divide by 0 prevention
        update_exp = np.eye(4)
    else:
        Omega_u = OMEGA(u)
        update_exp = np.cos(norm_u*theta)*np.eye(4) + (np.sin(norm_u*theta)/norm_u)*Omega_u

    # Multiply update matrix by current quaternion (represented as a 4x1 vector)
    new_q_data = update_exp @ np.array(q.data).reshape(4, 1) # need to reshape so matrix mult works correctly
    new_q_data = new_q_data.flatten() # converts back to 1D array
    new_q_data = new_q_data / np.linalg.norm(new_q_data)  # Normalize, just in case of small floating point errors. should be a unit quaternion anyway
    return sm.UnitQuaternion(new_q_data)

def OMEGA(omega):
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

def rodrigues(u, dt):
    """
    Compute the rotation matrix for a rotation of angle theta = ||u||*dt about axis u/||u||,
    using the Rodrigues formula.
    """
    norm_u = np.linalg.norm(u) 
    if norm_u < 1e-8: # prevent division by zero errors
        return np.eye(3)
    # theta = norm_u * dt
    # u_normalized = u / norm_u
    # u_skew = skew(u_normalized)
    # R = np.eye(3) + np.sin(theta)*u_skew + (1 - np.cos(theta))*(u_skew @ u_skew)

    #FIXME make sure i am implemented correct
    theta = norm_u * dt
    u_normalized = u / norm_u
    u_skew = skew(u)
    R = np.eye(3) + np.sin(theta)*(u_skew /u_normalized) + (1 - np.cos(theta))*((u_skew @ u_skew) /( u_normalized ** 2))
    return R

def quaternion_to_angle(q):
    """
    Compute the rotation angle (in radians) from the UnitQuaternion q.
    We use the rotation matrix representation and SpatialMath’s tr2angvec.
    """
    ang, axis = tr2angvec(q.R)
    return ang

def plot_raw_data():
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

    for index, row in data.iterrows():
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



# Initialize filter variables and inertial references
dt = 0.01 #TODO decide to keep constant time step or not.
num_steps = len(data)

# initial orientation
q = sm.UnitQuaternion()

# Initial gyroscope bias
b = np.zeros(3)

# Inertial acceleration (gravity, normalized) and magnetic field (given, normalized)
a0 = np.array([0, 0, 1])  
m0 = np.array([0.087117, 0.37923, -0.92119]) #TODO update this number

# Initial Estimate Vectors
v_hat_a = a0.copy() # .copy() copies the content into a new slot in memory
v_hat_m = m0.copy()

# Gains (adjust as needed) TODO adjust these guys to proper values
kp = 1.0     # Proportional gain for orientation correction
kI = 0.3     # Integral gain for bias correction
ka_nominal = 1.0     # Gain for accelerometer error
km_nominal = 1.0     # Gain for magnetometer error

# Expected values
expected_acc = 9.81
expected_mag = np.linalg.norm([4525.28449, 19699.18982, -47850.850686]) / 1000  # Convert nT to µT


# prev_time = data['t'][0] # Time step from the first entry, this will change dynamically in the loop  TODO decide to keep this or not
times = []
rotation_angles = []

#TEMP
m_corrected_array = [] # for graphing
ka_dynamic_list = [] # for graphng
km_dynamic_list = [] # for graphing


# Process the sensor data and update the Mahony filter
for index, row in data.iterrows():
    #update dt to ensure it is consistent with the data TODO decide to keep this or not
    # dt = t - prev_time if index != 0 else t
    # prev_time = t

    # Extract current measurements
    t = row['t']
    m = np.array([row['mx'], row['my'], row['mz']])
    gyro = np.array([row['gyrox'], row['gyroy'], row['gyroz']])
    a = np.array([row['ax'], row['ay'], row['az']])
    
    # Normalize accelerometer measurements
    # If statements prevents divide by zero errors where a = [0, 0, 0] or m = [0, 0, 0]
    if np.linalg.norm(a) != 0:
        v_a = a / np.linalg.norm(a)
    else:
        v_a = a

    # Normalize magnetometer measurements while subtracting gravity component
    #gravity terms
    g_inertial = np.array([0, 0, 1])
    g_body = q.R @ g_inertial
    #projecting m onto g_body, this gives the part of m that is in the gravity direction
    m_vertical = np.dot(m, g_body) # FIXME why multiply by g_body here? remove this? Make sure projection is correct here
    m_corrected = m - m_vertical # FIXME, normalize m before?because m is not normalized here but m_vertical is normalized
    if np.linalg.norm(m_corrected) != 0:
        v_m = m_corrected / np.linalg.norm(m_corrected)
    else:
        v_m = m_corrected

    m_corrected_array.append(m_corrected.copy())  #TEMP

    # #vvv --------------------------------------- Not Used --------------------------------------- vvv#
    # # Calculate dynamic km and ka gains based on gaussian curve
    # # compute sensor norms
    # a_norm = np.linalg.norm(a)
    # m_norm = np.linalg.norm(m_corrected)
    # # Compute confidence measures (example using a Gaussian function)
    # sigma_a = 0.0677649323574827  # adjusted based on expected variation
    # sigma_m = 0.1466190886315767 # adjusted based on expected variation in µT
    # # Confidence measures for accelerometer and magnetometer based on guassian distribution
    # conf_acc = np.exp(-((a_norm - 9.81)**2) / (2 * sigma_a**2))
    # conf_mag = np.exp(-((m_norm - expected_mag)**2) / (2 * sigma_m**2))
    # # Scale the nominal gains dynamically
    # ka_dynamic = ka_nominal * conf_acc
    # km_dynamic = km_nominal * conf_mag
    # ka_dynamic_list.append(ka_dynamic)  #TEMP for graphing
    # km_dynamic_list.append(km_dynamic)  #TEMP for graphing
    # #^^^ --------------------------------------- Not Used --------------------------------------- ^^^#

    
    # Compute the error signals from cross products: Innovation:
    error_acc = np.cross(v_a, v_hat_a)
    error_mag = np.cross(v_m, v_hat_m)
    omega_mes = ka_nominal * error_acc + km_nominal * error_mag

    # Update the gyroscope bias estimate
    b = b - kI * omega_mes * dt
    
    # Compute effective angular velocity for the update: 
    u = gyro - b + kp * omega_mes

    # Update the orientation quaternion using our update function
    q = updateQuaternion(q, u, dt)

    # Update the estimated reference vectors using the Rodrigues formula
    R_update = rodrigues(-u, dt)

    v_hat_a = R_update @ v_hat_a
    v_hat_a = v_hat_a / np.linalg.norm(v_hat_a)

    v_hat_m = R_update @ v_hat_m
    v_hat_m = v_hat_m / np.linalg.norm(v_hat_m)


    # Store the current time and the rotation angle from the quaternion
    times.append(t)
    rotation_angles.append(quaternion_to_angle(q))

rotation_angles_degrees = np.degrees(rotation_angles)
# plot rotation angle vs time for degrees
plt.figure(figsize=(10, 5))
plt.plot(times, rotation_angles_degrees, label='Rotation Angle (degrees)')
plt.xlabel('Time (s)')
plt.ylabel('Rotation Angle (degrees)')
plt.title('Rotation Angle vs. Time from Mahony Filter')
plt.legend()
plt.grid(True)
plt.show()

# Plot the rotation angle vs. time
plt.figure(figsize=(10, 5))
plt.plot(times, rotation_angles, label='Rotation Angle (rad)')
plt.xlabel('Time (s)')
plt.ylabel('Rotation Angle (rad)')
plt.title('Rotation Angle vs. Time from Mahony Filter')
plt.legend()
plt.grid(True)
plt.show()

# Compute and print the total rotation at the end of the dataset
total_rotation = sum(rotation_angles)
print(f"Total rotation over the recorded period: {total_rotation:.3f} radians") 

total_rotation_deg = np.degrees(total_rotation)
print(f"Total rotation over the recorded period: {total_rotation_deg:.3f} degrees") 

