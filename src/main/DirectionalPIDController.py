from omnibot.tcp import Connection
import math
import numpy as np
from time import time, sleep

# Network configuration constants
HOST = "130.235.83.171"  # Replace with appropriate IP address
PORT = 9006

# PID controller coefficients
KP = 0.05 # Proportional coefficient
KI = 0.01   # Integral coefficient (currently not used)
KD = 0   # Derivative coefficient (currently not used)

# # PID controller coefficients WORKING VALUES
# KP = 0.1 # Proportional coefficient
# KI = 0.3  # Integral coefficient (currently not used)
# KD = 0   # Derivative coefficient (currently not used)

# Time and physical constantss
R_VALUE = 0.16
WHEEL_RADIUS = 0.028
DEGREE_OFFSET = 145
MAX_INTEGRAL_TERM = np.pi 

# Desired pose
desired_x = 0
desired_y = 0
desired_theta = 0.0  # in radians, assuming the robot's API requires radians

# Robot parameters (these should be set to match your robot's specifics)
R = 0.16  # radius of the wheels
r = 0.028  # distance from the center of the robot to the center of each wheel

# Proportional gain for the controller
Kp = 20.0


class PID_controller:
    def __init__(self, debugMode=False, windowSize= 50):
        self.PRINT_INFO = debugMode
        if self.PRINT_INFO:
            print("Controller: Debug mode is on.")

        # Initialize previous wheel speeds
        self.prevPos = np.array([0,0])
        self.currPos = np.array([0,0])
        self.integral_error = 0  # Initialize integral error
        
        self.window_size = windowSize
        self.movingAvrgDir = []
    
    def add_value(self, value):
        self.movingAvrgDir.append(value)
       
        # Keep only the last 'window_size' number of elements
        self.movingAvrgDir = self.movingAvrgDir[-self.window_size:]

    def get_average(self):
        if not self.movingAvrgDir:
            return None  # Return None if the list is empty
        return sum(self.movingAvrgDir) / len(self.movingAvrgDir)
    
    def system_dynamics(self, theta):
        return np.array([
            [-np.sin(theta), np.cos(theta), R_VALUE],
            [-np.sin(theta + ((2 * np.pi) / 3)), np.cos(theta + ((2 * np.pi) / 3)), R_VALUE],
            [-np.sin(theta + ((4 * np.pi) / 3)), np.cos(theta + ((4 * np.pi) / 3)), R_VALUE]
        ])

    def calc_prev_omega(self):

        dir = self.currPos - self.prevPos
        angle = np.arctan2(dir[1] , dir[0]) # Calculate the angle and ensure it's within the valid range for arccos
     
        return angle

    def calc_wheel_speed(self, dirDot, TB):
       
        wheelSpeeds = (1 / WHEEL_RADIUS) * np.dot(TB, dirDot)

        if self.PRINT_INFO:
            print(f'Wheel Speeds: {wheelSpeeds}')

        return wheelSpeeds


    def forward_kinematics(self, tataDot,TB):
        
        # Perform forward kinematics to calculate the robot's velocity
        dirDot = WHEEL_RADIUS * np.dot(np.linalg.inv(TB), tataDot)

        if self.PRINT_INFO:
            print(f'Forward Kinematics x_dot: {dirDot}')

        return dirDot
    
    
    def angle_to_direction_vector(self,angle_in_radians):
        # Calculating the x and y components based on the angle
        x_component = np.cos(angle_in_radians)
        y_component = np.sin(angle_in_radians)

        # Creating a numpy array of length 2 with x and y components
        direction_vector = np.array([x_component, y_component])
        return direction_vector
    
    def calculate_control_signal(self, omega, system_dynamics, speed=6):
        
        # Convert angle to direction vector
        direction = self.angle_to_direction_vector(omega) 
        # Ensure 'direction' is a 1D array with 2 elements
        if direction.ndim > 1 and direction.shape[0] == 1:
            direction = direction.flatten()
        elif direction.ndim > 1:
            # Assuming you want to use the first row if direction is 2D
            direction = direction[0]

        # Scale the direction vector by the speed
        direction_scaled = direction * speed

        # Extend direction vector with a zero for 3D system dynamics
        directionExt = np.array([direction_scaled[0], direction_scaled[1], 0])
            
        # Calculate control signals
        controlSignals = (1 / WHEEL_RADIUS) * np.dot(system_dynamics, directionExt)

        return controlSignals.astype(int)

    
    def angle_error(self, desired_angle, actual_angle):
        # Calculate the raw difference
        raw_difference = desired_angle - actual_angle

        # Normalize the difference to be within [-pi, pi]
        normalized_difference = np.mod(raw_difference + np.pi, 2 * np.pi) - np.pi

        # The angle error should indicate the direction of the smallest rotation
        # from the actual angle to the desired angle
        return normalized_difference

    def normalize_angle(self,angle):
        # Normalize the angle to be within the range [-pi, pi]
        normalized_angle = np.mod(angle + np.pi, 2 * np.pi) - np.pi
        return normalized_angle

    def control_step(self, currPos, direction):
        direction += np.deg2rad(DEGREE_OFFSET)
        direction = self.normalize_angle(direction)
        self.currPos = currPos[0:2]
      
        # Convert theta to radians
        #E vi efterblivna finns bara 2st x, y i currpos
        theta = np.deg2rad(60)
        
        system_dynamics = self.system_dynamics(theta) # Insert current orientation of the bot into the system dynamics
        prevDir = self.calc_prev_omega() # Calculate the previous direction and add to the MAF if it is not None
        self.add_value(prevDir)
        avgPrevDir = self.get_average() # Retrieve the average direction

        pError = self.angle_error(direction, avgPrevDir + np.deg2rad(DEGREE_OFFSET)) # Calculate the proportional error using angle_error function

        # Update integral error and guard against windup
        self.integral_error += pError
        #self.integral_error = min(max(self.integral_error, -MAX_INTEGRAL_TERM), MAX_INTEGRAL_TERM)

        # Calculate the final control signal with integral term
        iError = self.integral_error 
        finalAngle = direction + pError * KP + iError* KI

        # Update the previous position
        self.prevPos = self.currPos

        return self.calculate_control_signal(finalAngle, system_dynamics)
    
    # The inverse kinematics matrix from your system
    def fuzzy_inverse_kinematics_matrix(self, theta):
        return (1 / r) * np.array([
            [-np.sin(theta), np.cos(theta), R],
            [-np.sin(theta + 2 * np.pi / 3), np.cos(theta + 2 * np.pi / 3), R],
            [-np.sin(theta + 4 * np.pi / 3), np.cos(theta + 4 * np.pi / 3), R]
        ])

    # Control loop logic
    def fuzzy_control_step(self, current_x, current_y, desired_x, desired_y, Kp):
        # Compute the errors in x and y
        error_x = desired_x - current_x
        error_y = desired_y - current_y

        # Calculate the desired change in position
        delta_x = Kp * error_x
        delta_y = Kp * error_y

        # As orientation is not important, set delta_theta to zero
        
     
        # Get the inverse kinematics matrix for the current orientation, assumed to be zero
        ik_matrix = self.fuzzy_inverse_kinematics_matrix(0.0)

        # Calculate the wheel speeds needed to achieve the desired change in position
        wheel_speeds = np.dot(ik_matrix, np.array([delta_x, delta_y, 0.0]))
        wheel_speeds = wheel_speeds.astype(int)

        return wheel_speeds