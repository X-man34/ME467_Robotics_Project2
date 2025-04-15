from spatialmath import SO3
import numpy as np
import spatialmath as sm    
from spatialmath.base import skew, tr2angvec
from abc import ABC, abstractmethod


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


def TRIAD(body_1, body_2, reference_1, reference_2, returnRotMatrx=False):
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
        self.m0 = true_m  # True magnetic field vector (inertial frame)
        # These are the default initial conditions and for optimal filter performance they should be set later with set_initial_conditions. 
        self.q = sm.UnitQuaternion()  # Initial orientation quaternion
        self.v_hat_a = np.array([0,0,1])
        self.v_hat_m = self.m0
        self.bias = np.zeros(3)  # Initial gyroscope bias
        self.m_corrected = true_m# expose for graphing. 
        self.estimated_error = 0
        self.omega_mes = 0
        self.use_TRIAD_initial_attitude_estimation = use_TRIAD_initial_attitude_estimation

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
        self.m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(self.m_corrected)

        # Compute the error signals from cross products: Innovation:
        error_acc = np.cross(v_a, self.v_hat_a)
        error_mag = np.cross(v_m, self.v_hat_m)
        self.omega_mes = self.ka_nominal * error_acc + self.km_nominal * error_mag# FIXME we likly need to normalize the values used here, but there is some dought about it. 

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
        if self.use_TRIAD_initial_attitude_estimation:
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


class NaiveEstimator(Estimator):
    def __init__(self, dT):
        self.dT = dT
        self.q = sm.UnitQuaternion()
        self.estimated_error = None
        self.bias = None
        self.v_hat_a = None
        self.v_hat_m = None
        self.m_corrected = None

    def time_step(self, magnetometer_vector: np.ndarray, gyro_vector: np.ndarray, accel_vector: np.ndarray, **kwargs)-> sm.UnitQuaternion:
        self.q = updateQuaternion(self.q, gyro_vector, self.dT)
        return self.q
    
    def set_initial_conditions(self, conditions):
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


class TriadEstimator(Estimator):
    """

    Custom estimator using triad somehow. 
    """
    def __init__(self, dT, kI=.3, kP=1.0, kA=1.0, kM=.5, true_m=np.array([0.087117, 0.37923, -0.92119]), **kwargs):
        self.g_inertial = np.array([0, 0, 9.0665])
        self.dT = dT
        self.kp = kP
        self.kM = kM
        self.kA = kA
        self.kI = kI
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

        # Normalize magnetometer measurements while subtracting gravity component
        #gravity terms
        
        g_body_estimated_from_curr_quat = self.q.R.T @ self.g_inertial
        #projecting m onto g_body, this gives the part of m that is in the gravity direction
        m_vertical = (np.dot(magnetometer_vector, g_body_estimated_from_curr_quat) / (np.linalg.norm(g_body_estimated_from_curr_quat) ** 2)) * g_body_estimated_from_curr_quat
        self.m_corrected = magnetometer_vector - m_vertical 
        v_m = normalize(self.m_corrected)

        #Also try and correct the true value to help with creating a basis.
        m_vertical = (np.dot(self.m0, g_body_estimated_from_curr_quat) / (np.linalg.norm(g_body_estimated_from_curr_quat) ** 2)) * g_body_estimated_from_curr_quat
        self.m0_corrected = self.m0 - m_vertical

       
        rot_matr_triad = TRIAD(accel_vector, v_m, self.g_inertial, self.m0_corrected, returnRotMatrx=True)#where we are?
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
        # self.estimated_error = 1 - np.dot(v_a, self.v_hat_a) + 1 - np.dot(v_m, self.v_hat_m)
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
        