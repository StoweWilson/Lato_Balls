import math
import numpy as np

# Parameters
g = 9.81  # Acceleration due to gravity (m/s^2)
l = 0.25   # Length of the pendulum (m)
a0 = 0.1  # Coefficient a0
omega= 2 # Angular frequency (rad/s)
b = 0   # Coefficient b
m = 0.020   # Mass of the pendulum bob (kg) 

# Initial conditions
theta0 = 1.5       # Initial angle (rad)
theta_dot0 = 0  # Initial angular velocity (rad/s)

# Time span
t_span = np.linspace(0,30, 150000)
# Initialize arrays to store results
theta_values = np.zeros_like(t_span)
theta_dot_values = np.zeros_like(t_span)

def apply_constraint(theta, theta_dot):
    if theta >= np.pi:
        theta_dot = -theta_dot 
        # Reverse theta_dot when theta reaches pi
    elif theta <= 0:
        theta_dot = -theta_dot  
        # Reverse theta_dot when theta reaches -pi
    return theta_dot

# Iterate through time steps and solve the differential equation
theta = theta0
theta_dot = theta_dot0

for i, t in enumerate(t_span):
    # Apply the constraint
    theta_dot = apply_constraint(theta, theta_dot)
    
    # Update theta_dot and theta using the differential equation
    theta_double_dot = (-(g / l) + a0 * omega**2 * np.cos(omega * t)) * np.sin(theta)
    theta_dot += theta_double_dot * (t_span[1] - t_span[0])
    theta += theta_dot * (t_span[1] - t_span[0])
    
    # Store the results
    theta_values[i] = theta
    theta_dot_values[i] = theta_dot

    print("theta values",theta)
    print("theta dot", theta_dot)