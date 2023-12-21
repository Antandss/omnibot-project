from omnibot.tcp import Connection
import math
import numpy as np
import time

# Insert suitable IP-address
HOST = "130.235.83.171"
PORT = 9006

# PID coefficients
KP = 1  # Start with a smaller value and gradually increase
KI = 0   # Integral part is not used
KD = 0   # Derivative part is not used

# PID variables
error_sum_x = 0
error_sum_y = 0
error_sum_theta = 0

previous_error_vx = 0
previous_error_vy = 0
previous_error_theta = 0

previous_u0 = 0
previous_u1 = 0
previous_u2 = 0

# Set your desired position and orientation
desired_vx = 0.5  # Target X position in meters
desired_vy = -0.5  # Target Y position in meters
desired_theta = 0  # Target orientation in radians

# Define the wheel radius and radius to middle
wheel_radius = 0.025
radius_to_middle = 0.16

max_control_vx = 1000  # Adjust as needed
max_control_vy = 1000 # Adjust as needed

error_vx_list = []
X_dotPrev_list = []


def convert_to_float(value):
    try:
        # Convert to a float
        return float(value)
    except ValueError as e:
        # Handle the exception
        print(f"Exception occurred: {e}")
        return 0.0

def forward_kinematics(control_vx, control_vy, theta):
    # Transformation matrix
    TB = np.array([
        [-np.sin(theta), np.cos(theta), radius_to_middle],
        [-np.sin(theta + ((2 * np.pi) / 3)), np.cos(theta + ((2 * np.pi) / 3)), radius_to_middle],
        [-np.sin(theta + ((4 * np.pi) / 3)), np.cos(theta + ((4 * np.pi) / 3)), radius_to_middle]
    ])

    x_dot_array = np.array([float(control_vx), float(control_vy), 0.0])

    print("x_dot_array:" + str(x_dot_array))
   
    # Use the matrix multiplication "@" operator for dot product
    wheel_speeds = (1 / wheel_radius) * (TB @ x_dot_array)

    # Convert the result to a 1D array
    wheel_speeds = wheel_speeds.flatten()

    return wheel_speeds

def wheelspeeds_to_linSpeed(phi1,phi2,phi3,theta):

    TB = np.array([
        [-np.sin(theta), np.cos(theta), radius_to_middle],
        [-np.sin(theta + ((2 * np.pi) / 3)), np.cos(theta + ((2 * np.pi) / 3)), radius_to_middle],
        [-np.sin(theta + ((4 * np.pi) / 3)), np.cos(theta + ((4 * np.pi) / 3)), radius_to_middle]
    ])

    G = (1/wheel_radius) * np.linalg.inv(TB)
    wheelSpeeds = np.array([phi1, phi2, phi3]).reshape(-1, 1)
    xDot = np.dot(G, wheelSpeeds)

    return [xDot[0], xDot[1]]

with Connection(HOST, PORT) as bot:
    prev_x = prev_y = prev_theta = 0

    for _ in range(10):
    #while True:
        # Current state
        #current_x = bot.get_x()
        #current_y = bot.get_y()
        current_theta = bot.get_theta()

        X_dotPrev = wheelspeeds_to_linSpeed(previous_u0, previous_u1, previous_u2, current_theta)  
        X_dotPrev_list.append(X_dotPrev)
        
        # Calculate errors
        error_vx = X_dotPrev[0] - desired_vx  
        error_vy = X_dotPrev[1] - desired_vy 

        error_vx_list.append(error_vx)
        
        # PID control laws for vx, vy, and theta_dot (only P part is active)
        control_vx = KP * error_vx
        control_vy = KP * error_vy

        # Limit the control signals
        control_vx = min(max_control_vx, max(-max_control_vx, control_vx))
        control_vy = min(max_control_vy, max(-max_control_vy, control_vy))

        # Get wheel speeds from the inverse kinematics
        wheel_speeds = forward_kinematics(control_vx, control_vy, current_theta)
        
        # Set wheel speeds
        bot.set_speeds([int(wheel_speeds[0]), int(wheel_speeds[1]), int(wheel_speeds[2])])

        print('wheel speeds: ' + str(int(wheel_speeds[0])) + ', ' + str(int(wheel_speeds[1])) + ', ' + str(int(wheel_speeds[2])))
    
        # Update previous sensor readings
        previous_u0 = wheel_speeds[0]
        previous_u1 = wheel_speeds[1]
        previous_u2 = wheel_speeds[2]

        time.sleep(2)
print("Error vx over iterations:", error_vx_list)
print("X_dotPrev over iterations:", X_dotPrev_list)
