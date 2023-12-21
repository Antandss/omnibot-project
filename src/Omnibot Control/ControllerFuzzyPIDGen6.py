from omnibot.tcp import Connection
from time import time, sleep
import numpy as np

# Insert suitable IP-address
HOST = "130.235.83.171"
PORT = 9006

# Desired pose
#Test1
# desired_x = 0.5
# desired_y = -0.5
#Test2
# desired_x = 0.0
# desired_y = -0.0
#Test3
desired_x = 0.0
desired_y = -0.5
#Test4
desired_x = -1.0
desired_y = 0.0

desired_theta = 0.0  # in radians, assuming the robot's API requires radians

# Robot parameters (these should be set to match your robot's specifics)
R = 0.16  # radius of the wheels
r = 0.028  # distance from the center of the robot to the center of each wheel

# Proportional gain for the controller
Kp = 100
Kp_theta = 0
max_speed = 255

# Define waypoints
waypoints = [(0.5, -0.5), (0.0, 0.0), (0.5, -0.5), (0.0, 0.0)]
tolerance = 0.1  # Tolerance for reaching a waypoint


# The inverse kinematics matrix from your system
def inverse_kinematics_matrix(theta):
    return (1 / r) * np.array([
        [-np.sin(theta), np.cos(theta), R],
        [-np.sin(theta + 2 * np.pi / 3), np.cos(theta + 2 * np.pi / 3), R],
        [-np.sin(theta + 4 * np.pi / 3), np.cos(theta + 4 * np.pi / 3), R]
    ])

# Control loop logic
def control_step(current_x, current_y, desired_x, desired_y, desired_theta, Kp, Kp_theta):
    # Compute the errors in x and y
    error_x = desired_x - current_x
    error_y = desired_y - current_y
    error_theta = desired_theta

    # Normalize error_theta to the range [-pi, pi]
    error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

    # Calculate the desired change in position
    delta_x = Kp * error_x
    delta_y = Kp * error_y

    # As orientation is not important, set delta_theta to zero
    delta_theta = Kp_theta * error_theta
    delta_theta = 0
    # Get the inverse kinematics matrix for the current orientation, assumed to be zero
    #ik_matrix = inverse_kinematics_matrix(delta_theta)
    ik_matrix = inverse_kinematics_matrix(0)

    # Calculate the wheel speeds needed to achieve the desired change in position
    wheel_speeds = np.dot(ik_matrix, np.array([delta_x, delta_y, delta_theta]))

    return wheel_speeds

# Function to move to a waypoint
def move_to_waypoint(bot, waypoint):
    desired_x, desired_y = waypoint
    while True:
        current_x, current_y = bot.get_x(), bot.get_y()
        if np.linalg.norm([current_x - desired_x, current_y - desired_y]) < tolerance:
            break  # Close enough to the waypoint
        wheel_speeds = control_step(current_x, current_y, desired_x, desired_y, desired_theta, Kp, Kp_theta)
        wheel_speeds = np.clip(wheel_speeds, -max_speed, max_speed)
        bot.set_speeds(list(wheel_speeds.astype(int)))
        print(f'Wheel speeds: {wheel_speeds}')
        print(f'x: {current_x}, y: {current_y}')
        sleep(0.2)

# Connect to the robot and move through the waypoints
with Connection(HOST, PORT) as bot:
    for waypoint in waypoints:
        move_to_waypoint(bot, waypoint)
        print(f"Reached waypoint {waypoint}, pausing for 3 seconds...")
        sleep(3)