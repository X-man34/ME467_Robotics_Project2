from spatialmath import SO3
import numpy as np
import spatialmath as sm    
from spatialmath.base import skew, tr2angvec
from abc import ABC, abstractmethod

#Following the East North Vertical convention from WWN-2025 at 43.603600◦ N and 116.208710◦ W, elevation 835m, normalized magnetic north is:
magnetic_north_normalized = np.array([0.08635072, 0.3812236, -0.92044126])

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
    """
    Turns the input into a unit vector, not to get confused with finding the magnitude of the vector. 
    """
    if np.linalg.norm(vector) != 0:
        return vector / np.linalg.norm(vector)
    else:
        return vector
    
def rodrigues(u, dT):
    """
    Computes the rotation matrix for a rotation of angle theta = ||u||*dt about axis u/||u||,
    using the Rodrigues formula.
    """
    norm_u = np.linalg.norm(u) 
    if norm_u < 1e-8: # prevent division by zero errors
        return np.eye(3)
    theta = norm_u * dT
    u_skew = skew(u)
    R = np.eye(3) + np.sin(theta)*(u_skew / norm_u) + (1 - np.cos(theta))*((u_skew @ u_skew) /( norm_u ** 2))
    return R  

def TRIAD(body_1: np.ndarray, body_2: np.ndarray, reference_1: np.ndarray, reference_2: np.ndarray, returnRotMatrx=False):
    """
    Computes orientation using the TRIAD method from two vector observations in both
    body and reference frames.

    Parameters:
    - body_1, body_2: 3D vectors in the body frame.
    - reference_1, reference_2: Corresponding 3D vectors in the reference frame.
    - returnRotMatrx (bool): If True, returns a rotation matrix; otherwise, a unit quaternion.

    Returns:
    - 3x3 rotation matrix or unit quaternion representing body-to-reference orientation.

    Notes:
    Constructs orthonormal bases from each vector pair, then aligns them to compute rotation matrix.
    """
    q_body = normalize(body_1)
    r_body = normalize(np.cross(body_1, body_2))
    s_body = np.cross(r_body, q_body)

    q_reference = normalize(reference_1)
    r_reference = normalize(np.cross(reference_1, reference_2))
    s_reference = np.cross(r_reference, q_reference)

    M_reference = np.vstack((s_reference, r_reference, q_reference))
    M_body = np.vstack((s_body, r_body, q_body))

    rotation_matrix = M_reference @ M_body.T
    if returnRotMatrx:
        return rotation_matrix
    else:
        return SO3(rotation_matrix).UnitQuaternion()

class Estimator(ABC):
    """
    Abstract base class defining the interface for attitude estimators using sensor fusion.

    Subclasses must implement:
        - time_step: performs a single update based on current sensor data.
        - set_initial_conditions: sets initial orientation estimates.
        - Properties for estimated error, bias, and reference vectors.

    Methods:
        time_step(magnetometer_vector, gyro_vector, accel_vector, **kwargs) -> UnitQuaternion
        set_initial_conditions(conditions: Tuple[np.ndarray, np.ndarray])
    """
    @abstractmethod
    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        pass

    @abstractmethod
    def set_initial_conditions(conditions: tuple):
        pass
    
    @property
    @abstractmethod
    def get_estimated_error(self):
        pass

    @property
    @abstractmethod
    def get_bias(self):
        pass

    @property
    @abstractmethod
    def get_v_hat_a(self):
        pass

    @property
    @abstractmethod
    def get_v_hat_m(self):
        pass

    @property
    @abstractmethod
    def get_m_corrected(self):
        pass

class MahonyFilter(Estimator):
    """
    Implements the Mahony attitude filter using IMU data (gyro, accel, magnetometer).

    Attributes:
        dT: Time step size.
        kp, kI: Proportional and integral gains.
        ka_nominal, km_nominal: Weights for accelerometer and magnetometer corrections.
        q: Current orientation (as UnitQuaternion).
        bias: Estimated gyroscope bias.
        g_inertial, m0: Known gravity and magnetic field vectors in the inertial frame.

    Behavior:
        Uses error between estimated and measured vectors to refine orientation.
        Includes bias correction via integral feedback.

    Kwargs:
        init_conditions: Tuple of body-frame accel and mag vectors used for initialization.
    """
    def __init__(self, dT, kp=1.0, kI=0.3, ka_nominal=1.0, km_nominal=.5, **kwargs):
        global magnetic_north_normalized
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kp
        self.kI = kI
        self.ka_nominal = ka_nominal
        self.km_nominal = km_nominal
        self.m0 = magnetic_north_normalized  # True magnetic field vector (inertial frame)
        self.q = sm.UnitQuaternion()
        self.v_hat_a = np.array([0,0,1])
        self.v_hat_m = self.m0
        self.bias = np.zeros(3)
        self.m_corrected = magnetic_north_normalized# Expose for graphing
        self.estimated_error = 0# Expose for graphing
        self.omega_mes = 0# Expose for graphing

    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        """
        Perform a single update step of the Mahony filter using sensor data.

        Parameters:
            magnetometer_vector (np.ndarray): 3D magnetic field vector in the body frame.
            gyro_vector (np.ndarray): 3D angular velocity from gyroscope (rad/s, body frame).
            accel_vector (np.ndarray): 3D acceleration vector from accelerometer (body frame).
            time_step (float, optional): Override for internal time step (default is self.dT).

        Returns:
            sm.UnitQuaternion: Updated orientation estimate as a UnitQuaternion.
        """     
        time_step = kwargs.get('time_step', self.dT)  # Use the provided time step or the default one            
        v_a = normalize(accel_vector)

        # Normalize magnetometer measurements and subtract gravity component        
        g_body = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body) / (np.linalg.norm(g_body) ** 2)) * g_body
        self.m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(self.m_corrected)

        # Compute the error signals from cross products: Innovation:
        error_acc = np.cross(v_a, self.v_hat_a)
        error_mag = np.cross(v_m, self.v_hat_m)
        self.omega_mes = self.ka_nominal * error_acc + self.km_nominal * error_mag

        # Compute effective angular velocity for the update: 
        u = gyro_vector - self.bias + self.kp * self.omega_mes

        # Update the orientation quaternion using our update function
        self.q = updateQuaternion(self.q, u, time_step)

        # Update the gyroscope bias estimate
        self.bias = self.bias - self.kI * self.omega_mes * time_step

        # Update the estimated reference vectors using the Rodrigues formula
        R_update = rodrigues(-u, time_step)
        self.v_hat_a = R_update @ self.v_hat_a
        self.v_hat_m = R_update @ self.v_hat_m

        #Update estimate of error for Q3
        self.estimated_error = 1 - np.dot(v_a, self.v_hat_a) + 1 - np.dot(v_m, self.v_hat_m)
        return self.q
    
    def set_initial_conditions(self, conditions):
        self.v_hat_a = normalize(conditions[0])
        self.v_hat_m = normalize(conditions[1])

    @property
    def get_estimated_error(self):
        return self.estimated_error

    @property
    def get_bias(self):
        return self.bias

    @property
    def get_v_hat_a(self):
        return self.v_hat_a

    @property
    def get_v_hat_m(self):
        return self.v_hat_m
    
    @property
    def get_m_corrected(self):
        return self.m_corrected

class NaiveEstimator(Estimator):
    """
    A simplistic estimator using only gyroscope integration.

    Attributes:
        q: Estimated orientation (UnitQuaternion).
        dT: Time step size.

    Behavior:
        Integrates raw gyro measurements directly to estimate orientation.
        Useful for debugging or comparison against more advanced filters.
    """
    def __init__(self, dT):
        self.dT = dT
        self.q = sm.UnitQuaternion()
        self.estimated_error = None
        self.bias = None
        self.v_hat_a = None
        self.v_hat_m = None
        self.m_corrected = None

    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        """
        Perform a simple integration of the gyroscope data to update orientation.

        Parameters:
            magnetometer_vector (np.ndarray): (Unused) Magnetic field vector.
            gyro_vector (np.ndarray): 3D angular velocity from gyroscope (rad/s).
            accel_vector (np.ndarray): (Unused) Acceleration vector.

        Returns:
            sm.UnitQuaternion: Updated orientation estimate.
        """
        self.q = updateQuaternion(self.q, gyro_vector, self.dT)
        return self.q
    
    def set_initial_conditions(self, conditions):
        self.v_hat_a = normalize(conditions[0])
        self.v_hat_m = normalize(conditions[1])
    
    @property
    def get_estimated_error(self):
        return self.estimated_error

    @property
    def get_bias(self):
        return self.bias

    @property
    def get_v_hat_a(self):
        return self.v_hat_a

    @property
    def get_v_hat_m(self):
        return self.v_hat_m
    
    @property
    def get_m_corrected(self):
        return self.m_corrected

class TriadEstimator(Estimator):
    """
    TRIAD-based orientation estimator using gravity and magnetic field vectors.

    Attributes:
        q: Orientation derived directly from TRIAD method at each timestep.
        g_inertial, m0: Known gravity and magnetic field vectors.
        dT: Time step.

    """
    def __init__(self, dT,kP=1.0, **kwargs):
        global magnetic_north_normalized
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kP
        self.m0 = magnetic_north_normalized
        self.q = sm.UnitQuaternion()
        self.v_hat_a = np.array([0,0,1])
        self.v_hat_m = self.m0
        
        self.m_corrected = magnetic_north_normalized
        self.estimated_error = 0
        self.omega_mes = 0
        self.bias = np.zeros(3)
        self.m0_corrected = None


    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        """
        Perform orientation estimation using the TRIAD method.

        Parameters:
            magnetometer_vector (np.ndarray): 3D magnetic field vector (body frame).
            gyro_vector (np.ndarray): (Unused) Angular velocity vector.
            accel_vector (np.ndarray): 3D acceleration vector (body frame).

        Returns:
            sm.UnitQuaternion: Orientation computed from TRIAD method.
        """               

        # Normalize magnetometer measurements while subtracting gravity component  
        g_body_estimated_from_curr_quat = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body_estimated_from_curr_quat) / (np.linalg.norm(g_body_estimated_from_curr_quat) ** 2)) * g_body_estimated_from_curr_quat
        self.m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(self.m_corrected)

        m_vertical = (np.dot(self.m0, g_body_estimated_from_curr_quat) / (np.linalg.norm(g_body_estimated_from_curr_quat) ** 2)) * g_body_estimated_from_curr_quat
        self.m0_corrected = self.m0 - m_vertical

        rotation_matrix = TRIAD(accel_vector, v_m, self.g_inertial, self.m0_corrected, returnRotMatrx=True)
        #Also try and correct the true value to help with creating a basis.
        self.v_hat_a = rotation_matrix.T @ self.g_inertial
        self.v_hat_m = rotation_matrix.T @ self.m0

        #Update estimate of error
        self.estimated_error = 1 - np.dot(normalize(accel_vector), self.v_hat_a) + 1 - np.dot(v_m, self.v_hat_m)
        return SO3(rotation_matrix).UnitQuaternion()

    def set_initial_conditions(self, conditions):
        self.q = TRIAD(conditions[0], conditions[1], self.g_inertial, self.m0)
        self.v_hat_a = normalize(conditions[0])  # Initial measured gravitational acceleration
        self.v_hat_m = normalize(conditions[1])

    @property
    def get_estimated_error(self):
        return self.estimated_error

    @property
    def get_bias(self):
        return self.bias

    @property
    def get_v_hat_a(self):
        return self.v_hat_a

    @property
    def get_v_hat_m(self):
        return self.v_hat_m
    
    @property
    def get_m_corrected(self):
        return self.m_corrected

class MahonyWithTriadEstimator(Estimator):
    """
    Mahony filter enhanced with TRIAD-based corrections.

    Attributes:
        q: Orientation (UnitQuaternion).
        kp: Proportional gain used to correct Mahony error with TRIAD comparison.
        m0: True magnetic field in inertial frame.
        g_inertial: Gravity in inertial frame.

    Behavior:
        TRIAD used to form a reference rotation, and Mahony-style correction is applied
        using the rotational difference between current estimate and TRIAD estimate.
    """
    def __init__(self, dT,kP=1.0, true_m=np.array([0.087117, 0.37923, -0.92119]), **kwargs):
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kP
        self.m0 = true_m  # True magnetic field vector (inertial frame)
        self.q = sm.UnitQuaternion()  # Initial orientation quaternion
        self.v_hat_a = np.array([0,0,1])
        self.v_hat_m = self.m0
        
        self.m_corrected = true_m# expose for graphing. 
        self.estimated_error = 0
        self.omega_mes = 0
        self.bias = np.zeros(3)  # Initial gyroscope bias
        self.m0_corrected = None

    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        """
        Perform an enhanced Mahony filter update using TRIAD-based correction.

        Parameters:
            magnetometer_vector (np.ndarray): 3D magnetic field vector (body frame).
            gyro_vector (np.ndarray): 3D angular velocity (rad/s, body frame).
            accel_vector (np.ndarray): 3D acceleration vector (body frame).
            time_step (float, optional): Override for internal time step.

        Returns:
            sm.UnitQuaternion: Updated orientation estimate.
        """
        time_step = kwargs.get('time_step', self.dT)  # Use the provided time step or the default one            

        # Normalize magnetometer measurements while subtracting gravity component
        g_body_estimated_from_curr_quat = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body_estimated_from_curr_quat) / (np.linalg.norm(g_body_estimated_from_curr_quat) ** 2)) * g_body_estimated_from_curr_quat
        self.m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(self.m_corrected)

        #Also try and correct the true value to help with creating a basis.
        m_vertical = (np.dot(self.m0, g_body_estimated_from_curr_quat) / (np.linalg.norm(g_body_estimated_from_curr_quat) ** 2)) * g_body_estimated_from_curr_quat
        self.m0_corrected = self.m0 - m_vertical

        rot_matr_triad = TRIAD(accel_vector, v_m, self.g_inertial, self.m0_corrected, returnRotMatrx=True)
        angle, vector = tr2angvec(self.q.R @ rot_matr_triad.T)
        self.omega_mes = angle * vector

        # Compute effective angular velocity for the update: 
        u =  gyro_vector - self.kp * self.omega_mes

        # Update the orientation quaternion using our update function
        self.q = updateQuaternion(self.q, u, time_step)
        # self.q = TRIAD(accel_vector, magnetometer_vector, self.g_inertial, self.m0, returnRotMatrx=False)
        R_update = rodrigues(-u, time_step)
        self.v_hat_a = R_update @ self.v_hat_a
        self.v_hat_m = R_update @ self.v_hat_m

        #Update estimate of error for Q3
        self.estimated_error = 1 - np.dot(normalize(accel_vector), self.v_hat_a) + 1 - np.dot(v_m, self.v_hat_m)
        return self.q
    
    def set_initial_conditions(self, conditions):
        self.q = TRIAD(conditions[0], conditions[1], self.g_inertial, self.m0)
        self.v_hat_a = normalize(conditions[0])  # Initial measured gravitational acceleration
        self.v_hat_m = normalize(conditions[1])

    @property
    def get_estimated_error(self):
        return self.estimated_error

    @property
    def get_bias(self):
        return self.bias

    @property
    def get_v_hat_a(self):
        return self.v_hat_a

    @property
    def get_v_hat_m(self):
        return self.v_hat_m
    
    @property
    def get_m_corrected(self):
        return self.m_corrected
    
class HybridMahonyTriadFilter(Estimator):
    """
    A hybrid estimator that dynamically switches between Mahony and TRIAD filters
    based on gyro activity (low vs. high motion scenarios).

    Attributes:
        threshold: Gyro norm threshold to switch behavior.
        All Mahony and TRIAD-related attributes (q, kp, kI, m0, etc.)

    Behavior:
        - Uses Mahony when motion is detected (gyro norm > threshold).
        - Falls back to TRIAD when motion is minimal, improving robustness in static phases.
    """
    def __init__(self, dT, threshold, kp=1.0, kI=0.3, ka_nominal=1.0, km_nominal=.5, true_m=np.array([0.087117, 0.37923, -0.92119]), **kwargs):
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kp
        self.kI = kI
        self.ka_nominal = ka_nominal
        self.km_nominal = km_nominal
        self.m0 = true_m  # True magnetic field vector (inertial frame)
        self.q = sm.UnitQuaternion()  # Initial orientation quaternion
        self.v_hat_a = np.array([0,0,1])
        self.v_hat_m = self.m0
        self.bias = np.zeros(3)  # Initial gyroscope bias
        self.m_corrected = true_m# expose for graphing. 
        self.estimated_error = 0
        self.omega_mes = 0
        self.threshold = threshold  # Threshold for gyroscope magnitude to switch between filter

    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        """
        Perform a hybrid Mahony/TRIAD filter update based on gyroscope activity.

        Parameters:
            magnetometer_vector (np.ndarray): 3D magnetic field vector (body frame).
            gyro_vector (np.ndarray): 3D angular velocity vector (body frame).
            accel_vector (np.ndarray): 3D acceleration vector (body frame).
            time_step (float, optional): Optional override of the default filter timestep.

        Returns:
            sm.UnitQuaternion: Updated orientation estimate.
        """  
        time_step = kwargs.get('time_step', self.dT)  # Use the provided time step or the default one            
        v_a = normalize(accel_vector)

        # Normalize magnetometer measurements while subtracting gravity component
        g_body = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body) / (np.linalg.norm(g_body) ** 2)) * g_body
        self.m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(self.m_corrected)

        #Also try and correct the true value to help with creating a basis.
        m_vertical = (np.dot(self.m0, g_body) / (np.linalg.norm(g_body) ** 2)) * g_body
        self.m0_corrected = self.m0 - m_vertical


        # Switch how innovation is calculated depending on the gyro norm. 
        if np.linalg.norm(gyro_vector) < self.threshold:
            rot_matr_triad = TRIAD(accel_vector, v_m, self.g_inertial, self.m0_corrected, returnRotMatrx=True)
            angle, vector = tr2angvec(self.q.R @ rot_matr_triad.T)
            self.omega_mes = angle * vector
            # Compute effective angular velocity for the update: 
            u =  gyro_vector - self.kp * self.omega_mes
            print("Using TRIAD")

        else:
            # Compute the error signals from cross products: Innovation:
            error_acc = np.cross(v_a, self.v_hat_a)
            error_mag = np.cross(v_m, self.v_hat_m)
            self.omega_mes = self.ka_nominal * error_acc + self.km_nominal * error_mag# FIXME we likly need to normalize the values used here, but there is some dought about it. 
            # Compute effective angular velocity for the update: 
            u = gyro_vector - self.bias + self.kp * self.omega_mes
            print("Using MAHONY")

        # Update the orientation quaternion using our update function
        self.q = updateQuaternion(self.q, u, time_step)

        # Update the gyroscope bias estimate
        self.bias = self.bias - self.kI * self.omega_mes * time_step

        # Update the estimated reference vectors using the Rodrigues formula
        R_update = rodrigues(-u, time_step)
        self.v_hat_a = R_update @ self.v_hat_a
        self.v_hat_m = R_update @ self.v_hat_m

        #Update estimate of error for Q3
        self.estimated_error = 1 - np.dot(v_a, self.v_hat_a) + 1 - np.dot(v_m, self.v_hat_m)
        return self.q
    
    def set_initial_conditions(self, conditions):
        self.q = TRIAD(conditions[0], conditions[1], self.g_inertial, self.m0)
        self.v_hat_a = normalize(conditions[0])  # Initial measured gravitational acceleration
        self.v_hat_m = normalize(conditions[1])

    @property
    def get_estimated_error(self):
        return self.estimated_error

    @property
    def get_bias(self):
        return self.bias

    @property
    def get_v_hat_a(self):
        return self.v_hat_a

    @property
    def get_v_hat_m(self):
        return self.v_hat_m
    
    @property
    def get_m_corrected(self):
        return self.m_corrected