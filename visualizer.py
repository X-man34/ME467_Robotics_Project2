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

def simulate_and_visualize_data(csv_data: pd.DataFrame, time_step: float, kp, kI, ka_nominal, km_nominal, use_TRIAD_initial_attitude_estimation=False, do_3D_vis=True, show_extra_vectors = False, show_spatial_coords=False, show_body_coords=False):
    """
    Performs a simulation on a pandas dataframe with header names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])
    There is the option to not do a full 3D visualization
    Returns data for graphing:
        times-the timestamps
        rotation_angles-the quaterion's rotation angle
        euler_angles-the euler angles at each timesteo
        estimated_error-the filter's estimated error at each time step. 
        gyro_bias_estimate-the magnitude of the bias estimate. 
    """  
    #Grab the initial values
    row = csv_data.iloc[0]
    raw_mag_vector = np.array([row['mx'], row['my'], row['mz']])
    raw_accel_vector = np.array([row['ax'], row['ay'], row['az']])
    
    #Initialize variables
    times = []
    rotation_angles = []
    roll = []
    pitch = []
    yaw = []
    bias_estimates = []
    error_estimates = []
    # Initialize the filter. This is where you change the gains. You don't have to pass in initial conditions, but it improves the estimate. 
    # You can also ask it to use the TRIAD initial pose estimatior, but at the time of writing the implementation does not work and its not asked for question 2, so its left disabled. 
    mahony_filter = MahonyFilter(dT=time_step, kp=kp, kI=kI, ka_nominal=ka_nominal, km_nominal=km_nominal, use_TRIAD_initial_attitude_estimation=use_TRIAD_initial_attitude_estimation, init_conditions=(raw_accel_vector, raw_mag_vector))
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


        # Optinally show or hide things. 
        if not show_extra_vectors:
            model.geom_rgba[model.geom(name="v_m_hat").id][3] = 0.0
            model.geom_rgba[model.geom(name="v_a_hat").id][3] = 0.0

        if not show_spatial_coords:
            model.geom_rgba[model.geom(name="x_axis").id][3] = 0.0
            model.geom_rgba[model.geom(name="y_axis").id][3] = 0.0
            model.geom_rgba[model.geom(name="z_axis").id][3] = 0.0

        if not show_body_coords:
            model.geom_rgba[model.geom(name="body_x_axis").id][3] = 0.0
            model.geom_rgba[model.geom(name="body_y_axis").id][3] = 0.0
            model.geom_rgba[model.geom(name="body_z_axis").id][3] = 0.0
            


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
            euler_angles = SO3.UnitQuaternion(current_orientation_quat).rpy(unit='deg')
            roll.append(euler_angles[0])
            pitch.append(euler_angles[1])
            yaw.append(euler_angles[2])

            error_estimates.append(mahony_filter.estimated_error)
            bias_estimates.append(np.linalg.norm(mahony_filter.bias))



            if do_3D_vis:
                # Update the model with the new orientation
                #phone mesh
                time_simulated += time_step
                mujoco_model_data.qpos[qpos_addr:qpos_addr+3] = [0,0,0]
                mujoco_model_data.qpos[qpos_addr+3:qpos_addr+7] = current_orientation_quat.data[0]


                # TODO more fun visualization, it seems like v_hat_m is not pointing in the right direction. 
                # estimate vectors
                # mujoco_model_data.qpos[qpos_addr_v_a_hat:qpos_addr_v_a_hat+4] = get_quat_from_vec(mahony_filter.v_hat_a)
                mujoco_model_data.qpos[qpos_addr_v_m_hat:qpos_addr_v_m_hat+4] = get_quat_from_vec(mahony_filter.omega_mes, negate_z=False)
                mujoco_model_data.qpos[qpos_addr_v_a_hat:qpos_addr_v_a_hat+4] = get_quat_from_vec(mahony_filter.v_hat_a, negate_z=True)
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
    return times, rotation_angles, bias_estimates, error_estimates, roll, pitch, yaw