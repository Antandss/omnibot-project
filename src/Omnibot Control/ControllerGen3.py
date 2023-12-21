from omnibot.tcp import Connection
import math
import numpy as np
import time

# Insert suitable IP-address
HOST = "130.235.83.171"
PORT = 9006

# PID coefficients
KP = 50  # Proportional gain, start with a smaller value and gradually increase
KI = 1  # Integral gain, start with a smaller value
KD = 0.00  # Derivative gain, start with a smaller value

# PID variables
error_sum_x = 0
error_sum_y = 0
error_sum_theta = 0

previous_error_x = 0
previous_error_y = 0
previous_error_theta = 0

# Set your desired position and orientation
desired_x = 0.5  # Target X position in meters
desired_y = -0.5  # Target Y position in meters
desired_theta = 0  # Target orientation in radians

previous_u0 = 0
previous_u1 = 0
previous_u2 = 0

# Define the wheel radius and radius to middle
wheel_radius = 0.025
radius_to_middle = 0.16

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

    print("control_vx" + str(control_vx))
    print("control_vy" + str(control_vy))
    x_dot_array = np.array([control_vx, control_vy, 0])

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
    wheelSpeeds = np.array([phi1, phi2, phi3])
    xDot = G * wheelSpeeds  # x_dot = G * u

    print("xDot" + str(xDot))
    return [xDot[0], xDot[1]]

def calculate_velocities(controlspeed1, controlspeed2, controlspeed3, wheel_radius):
    theta = np.deg2rad(0)  # Robot orientation (in radians)

    # Transformation matrix G
    G = np.array([
        [-np.sin(theta), np.cos(theta), radius_to_middle],
        [-np.sin(theta + (2 * np.pi / 3)), np.cos(theta + (2 * np.pi / 3)), radius_to_middle],
        [-np.sin(theta + (4 * np.pi / 3)), np.cos(theta + (4 * np.pi / 3)), radius_to_middle]
    ])

    # Inverse of the transformation matrix
    G_inverse = np.linalg.inv(G)

    # Control speeds vector
    control_speeds = np.array([controlspeed1, controlspeed2, controlspeed3])

    # Scaling control speeds by the wheel radius
    scaled_speeds = wheel_radius * control_speeds

    # Calculate desired velocities
    desired_velocities = G_inverse @ scaled_speeds

    print(str(desired_velocities))

    return desired_velocities


with Connection(HOST, PORT) as bot:
    prev_x = prev_y = prev_theta = 0

    while True:
        # Current state
        current_x = bot.get_x()
        current_y = bot.get_y()
        current_theta = bot.get_theta()

        # Calculate errors
        error_x = desired_x - current_x
        error_y = desired_y - current_y
        error_theta = desired_theta - current_theta

        # Handle theta wrap-around
        if error_theta > math.pi:
            error_theta -= 2 * math.pi
        elif error_theta < -math.pi:
            error_theta += 2 * math.pi

        # Integrate errors
        error_sum_x += error_x
        error_sum_y += error_y
        error_sum_theta += error_theta

        # Derivative of errors
        d_error_x = error_x - previous_error_x
        d_error_y = error_y - previous_error_y
        d_error_theta = error_theta - previous_error_theta

        # PID control laws for vx, vy, and theta_dot
        control_vx = KP * error_x + KI * error_sum_x + KD * d_error_x
        control_vy = KP * error_y + KI * error_sum_y + KD * d_error_y
        control_theta_dot = KP * error_theta + KI * error_sum_theta + KD * d_error_theta

        # Get wheel speeds from the kinematics function
        wheel_speeds = calculate_velocities(control_vx, control_vy, control_theta_dot, 0.025)
        
        # Set wheel speeds
        bot.set_speeds([int(wheel_speeds[0]), int(wheel_speeds[1]), int(wheel_speeds[2])])

        print('Wheel speeds: ' + str(int(wheel_speeds[0])) + ", " + str(int(wheel_speeds[1])) + ", " + str(int(wheel_speeds[2])))
       
        # Update previous errors
        previous_error_x = error_x
        previous_error_y = error_y
        previous_error_theta = error_theta

        time.sleep(0.5)

        # You can add any additional conditions to break the loop if needed
