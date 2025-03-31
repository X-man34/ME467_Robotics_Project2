# Build body angular velocity inputs
# Initial time t0 = 0, q(t0) = 1<0>
deltat = 0.1

import numpy as np

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

# Example usage:
if __name__ == "__main__":
    deltat = 0.001
    data = generate_body_angular_velocities(deltat)
    print(data)
