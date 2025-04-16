import spatialmath as sm
from spatialmath.base import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use("TkAgg")
import filters

def step_animation(quaternion, deltaT):
    # Clear the axes to redraw the updated frame.
    ax.clear()
    
    # Define the original coordinate frame axes.
    origin = np.array([0, 0, 0])
    x_axis = np.array([1, 0, 0])
    y_axis = np.array([0, 1, 0])
    z_axis = np.array([0, 0, 1])
    
    # Plot the original coordinate frame.
    ax.quiver(*origin, *x_axis, color='r', label='Original X')
    ax.quiver(*origin, *y_axis, color='g', label='Original Y')
    ax.quiver(*origin, *z_axis, color='b', label='Original Z')
    
    # Get the rotation matrix from the updated quaternion.
    R = quaternion.R
    
    # Compute the transformed axes.
    transformed_x = R @ x_axis
    transformed_y = R @ y_axis
    transformed_z = R @ z_axis
    
    # Plot the transformed coordinate frame.
    ax.quiver(*origin, *transformed_x, color='r', label='Transformed X')
    ax.quiver(*origin, *transformed_y, color='g', label='Transformed Y')
    ax.quiver(*origin, *transformed_z, color='b', label='Transformed Z')
    
    # Set the axis limits and labels.
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    # Draw the updated plot and pause briefly.
    plt.draw()
    plt.pause(deltaT) 

def generate_body_angular_velocities(deltat):
    """
    Generates a time series of body angular velocities.
    
    Parameters:
        deltat (float): Time step between consecutive data points (e.g., 0.001).
    
    Returns:
        np.ndarray: A 2D array with 4 columns:
            - Column 0: time from 0 to 1 second (not inclusive of 1 if 1/deltat is not an integer).
            - Column 1: angular velocity along x.
            - Column 2: angular velocity along y.
            - Column 3: angular velocity along z.
            
    The angular velocity pattern is as follows:
        - For the first 0.25 s: [pi, 0, 0]
        - For the next 0.25 s: [0, 0, pi]
        - For the next 0.25 s: [-pi, 0, 0]
        - For the final 0.25 s: [0, 0, -pi]
    """
    
    # Generate the time vector
    t = np.arange(0, 1, deltat)
    
    # Number of time points
    n = len(t)
    
    # Initialize angular velocity array with zeros
    omega = np.zeros((n, 3))
    
    # Define the value of pi for clarity
    pi_val = np.pi
    
    # Define the time intervals for each quarter-second segment
    mask1 = t < 0.25             # First quarter: [pi, 0, 0]
    mask2 = (t >= 0.25) & (t < 0.5)  # Second quarter: [0, 0, pi]
    mask3 = (t >= 0.5) & (t < 0.75)  # Third quarter: [-pi, 0, 0]
    mask4 = (t >= 0.75)           # Fourth quarter: [0, 0, -pi]
    
    # Assign the values based on the interval masks
    omega[mask1, 0] = pi_val      # x component
    omega[mask2, 2] = pi_val      # z component
    omega[mask3, 0] = -pi_val     # x component
    omega[mask4, 2] = -pi_val     # z component
    
    # Combine the time vector and the angular velocities into one array
    data = np.column_stack((t, omega))
    return data

if __name__ == "__main__":
    deltaT = .01
    quaternion = sm.UnitQuaternion()
    input_data = generate_body_angular_velocities(deltaT)

    # Set up the figure and 3D axis for visualization. and maximize the window
    plt.ion()
    fig = plt.figure()
    manager = plt.get_current_fig_manager()

    ax = fig.add_subplot(111, projection='3d')

    for time, wx, wy, wz in input_data:   
        quaternion = filters.updateQuaternion(quaternion, np.array([wx, wy, wz]).reshape(3,1), deltaT)
        temp = sm.UnitQuaternion()
        temp.data = [np.array(quaternion).reshape(1,4)]
        step_animation(temp, deltaT)

    # generate the final rotation matrix analytically to check answer. 
    one = sm.SO3.Rx(np.pi/4)
    two = sm.SO3.Rz(np.pi/4)
    three = sm.SO3.Rx(-np.pi/4)
    four = sm.SO3.Rz(-np.pi/4)

    final_quaternion = sm.UnitQuaternion()
    final_quaternion.data = [np.array(quaternion).reshape(1,4)]

    correct_quaternion = sm.UnitQuaternion()
    correct_quaternion.data = [np.array(r2q(np.array(one @ two @ three @ four))).reshape(1,4)]

    print("Correct answer: " + str(correct_quaternion))
    print("Our Answer: " + str(final_quaternion))
    if (final_quaternion == correct_quaternion):
        print("Final quaternion is correct!")
    else:
        print("We are wrong, or there is a floating point error in the comparison. ")

    print((np.array(one @ two @ three @ four)))