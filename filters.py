from spatialmath import SO3
import numpy as np
import spatialmath as sm    
from spatialmath.base import skew


def OMEGA(omega: np.ndarray)-> np.ndarray:
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

def updateQuaternion(q_old: sm.UnitQuaternion, u: np.ndarray, dT: float)-> sm.UnitQuaternion:
    """
    Update the given orientation quaternion q using the effective angular velocity u over time dt.
    This implements the closed-form update:
        q_{k+1} = exp((1/2)*Omega(u)*dt) * q_k.
    Here, q is a spatialmath UnitQuaternion and u is a 3-element vector.
    """
    norm_u = np.linalg.norm(u)
    theta = 0.5 * dT
    if norm_u < 1e-8: # divide by 0 prevention
        update_exp = np.eye(4)
    else:
        Omega_u = OMEGA(u)
        update_exp = np.cos(norm_u*theta)*np.eye(4) + (np.sin(norm_u*theta)/norm_u)*Omega_u

    # Multiply update matrix by current quaternion (represented as a 4x1 vector)
    new_q_data = update_exp @ np.array(q_old.data).reshape(4, 1) # need to reshape so matrix mult works correctly
    new_q_data = new_q_data.flatten() # converts back to 1D array
    return sm.UnitQuaternion(new_q_data)


def normalize(vector: np.ndarray)-> np.ndarray:
    if np.linalg.norm(vector) != 0:
        return vector / np.linalg.norm(vector)
    else:
        return vector
    
def rodrigues(u, dT):
    """
    Compute the rotation matrix for a rotation of angle theta = ||u||*dt about axis u/||u||,
    using the Rodrigues formula.
    """
    norm_u = np.linalg.norm(u) 
    if norm_u < 1e-8: # prevent division by zero errors
        return np.eye(3)
    theta = norm_u * dT
    u_skew = skew(u)
    R = np.eye(3) + np.sin(theta)*(u_skew / norm_u) + (1 - np.cos(theta))*((u_skew @ u_skew) /( norm_u ** 2))
    return R  

class MahonyFilter:
    """
    MahonyFilter: A class implementing the Mahony filter for attitude estimation.
    The Mahony filter is an algorithm used for estimating the orientation of an object in 3D space using sensor data from a gyroscope, accelerometer, and magnetometer. 
    This implementation may include it the future.  the TRIAD algorithm for initializing the attitude estimate based on initial conditions.
    Attributes:
        g_inertial (np.ndarray): Inertial frame gravity vector.
        dT (float): Time step for the filter updates.
        kp (float): Proportional gain for the filter.
        kI (float): Integral gain for the filter.
        ka_nominal (float): Weight for accelerometer-based corrections.
        km_nominal (float): Weight for magnetometer-based corrections.
        q (sm.UnitQuaternion): Current orientation quaternion.
        bias (np.ndarray): Gyroscope bias estimate.
        m0 (np.ndarray): True magnetic field vector in the inertial frame.
        v_hat_a (np.ndarray): Estimated gravitational acceleration vector in the body frame.
        v_hat_m (np.ndarray): Estimated magnetic field vector in the body frame.
    Methods:
        __init__(self, dT, kp=1.0, kI=0.3, ka_nominal=1.0, km_nominal=0.5, true_m=np.array([...]), **kwargs):
            Initializes the Mahony filter with the given parameters and optional initial conditions.
        rodrigues(self, u, **kwargs):
            Computes the rotation matrix for a rotation of angle ||u||*dt about axis u/||u|| using the Rodrigues formula.
        time_step(self, magnetometer_vector, gyro_vector, accel_vector, **kwargs) -> sm.UnitQuaternion:
            Performs a single time step of the Mahony filter update using sensor measurements.
        orthonormalize_matrix(self, matrix):
            Orthonormalizes a given matrix using QR decomposition.
        get_inital_quaterion_TRIAD(self, body_gravity_vector, body_north_vector, actual_gravity_vector, actual_north_vector):
            Estimates the initial orientation quaternion using the TRIAD algorithm based on body and inertial frame vectors.
    Kwargs:
        init_conditions: a tuple with the initial accel and mag data in the body frame of the form (initial_acceleration_vector, initial_north_vector). It not passed [0 0 1] and the true m value are used. 
    """
    def __init__(self, dT, kp=1.0, kI=0.3, ka_nominal=1.0, km_nominal=.5, true_m=np.array([0.087117, 0.37923, -0.92119]), use_TRIAD_initial_attitude_estimation=False, **kwargs):
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kp
        self.kI = kI
        self.ka_nominal = ka_nominal
        self.km_nominal = km_nominal
        self.q = sm.UnitQuaternion()  # Initial orientation quaternion
        self.bias = np.zeros(3)  # Initial gyroscope bias
        self.m0 = true_m  # True magnetic field vector (inertial frame)
        
        if use_TRIAD_initial_attitude_estimation and 'init_conditions' in kwargs:
            self.q = self.get_inital_quaterion_TRIAD(kwargs['init_conditions'][0], kwargs['init_conditions'][1], self.g_inertial, self.m0)
        self.v_hat_a = normalize(kwargs.get("init_conditions", (np.array([0, 0, 1]),))[0])  # Initial estimate of gravitational acceleration
        self.v_hat_m = normalize(kwargs.get("init_conditions", (np.array([0, 0, 1]),true_m.copy()))[1])


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
        v_a = normalize(accel_vector)

        # Normalize magnetometer measurements while subtracting gravity component
        #gravity terms
        
        g_body = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body) / (np.linalg.norm(g_body) ** 2)) * g_body
        m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(m_corrected)

        # Compute the error signals from cross products: Innovation:
        error_acc = np.cross(v_a, self.v_hat_a)
        error_mag = np.cross(v_m, self.v_hat_m)
        omega_mes = self.ka_nominal * error_acc + self.km_nominal * error_mag# FIXME we likly need to normalize the values used here, but there is some dought about it. 

        # Compute effective angular velocity for the update: 
        u = gyro_vector - self.bias + self.kp * omega_mes

        # Update the orientation quaternion using our update function
        self.q = updateQuaternion(self.q, u, time_step)

        # Update the gyroscope bias estimate
        self.bias = self.bias - self.kI * omega_mes * time_step

        # Update the estimated reference vectors using the Rodrigues formula
        R_update = rodrigues(-u, time_step)
        self.v_hat_a = R_update @ self.v_hat_a
        self.v_hat_m = R_update @ self.v_hat_m
        return self.q


    # FIXME: this function currently does not work, or we don't know how to use it. 
    def get_inital_quaterion_TRIAD(self, body_gravity_vector, body_north_vector, actual_gravity_vector, actual_north_vector):
        body_basis_1 = normalize(body_gravity_vector)
        spatial_basis_1 = normalize(actual_gravity_vector)
        body_basis_2 = normalize(body_north_vector)
        spatial_basis_2 = normalize(actual_north_vector)
        body_basis_3 = np.cross(body_basis_1, body_basis_2)
        spatial_basis_3 = np.cross(spatial_basis_1, spatial_basis_2)
        body_matrix = np.vstack((body_basis_1, body_basis_2, body_basis_3))
        spatial_matrix = np.vstack((spatial_basis_1, spatial_basis_2, spatial_basis_3))


        #Applies gram-schmidt optimization to basically massage the matrix to get it as close to orthonormal as possible
        # Shoudln't be nessacary? 
        # SO3() started giving bad argumetn errors when tried to implement algrithm like paper said instead of like chat said. this "fixed" the errors
        # Although the presence of that error is likly indicitive of an error in the method by which the rotation matix was computed it the first place. 
        body_matrix = np.linalg.qr(body_matrix)[0]
        spatial_matrix = np.linalg.qr(spatial_matrix)[0]


        # body_matrix = np.column_stack((body_basis_1, body_basis_2, body_basis_3))
        # spatial_matrix = np.column_stack((spatial_basis_1, spatial_basis_2, spatial_basis_3))
        rotation_matrix = spatial_matrix @ np.linalg.inv(body_matrix)

        # Should be the same or close. 
        print(normalize(actual_north_vector))
        print(rotation_matrix @ normalize(body_north_vector))
        return SO3(rotation_matrix).UnitQuaternion()

