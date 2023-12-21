from omnibot.tcp import Connection
from time import time, sleep
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

def calculate_wheel_speeds(vx, vy, theta, R):
    T = np.array([
        [-np.sin(theta), np.cos(theta), R],
        [-np.sin(theta + 2*np.pi/3), np.cos(theta + 2*np.pi/3), R],
        [-np.sin(theta + 4*np.pi/3), np.cos(theta + 4*np.pi/3), R]
    ])
    velocities = np.array([vx, vy, 0])  # Assuming theta_dot is 0 for simplicity
    wheel_speeds = np.dot(T, velocities)
    return wheel_speeds

# PID Controller Setup
pid_x = PIDController(Kp=50, Ki=10, Kd=0.0)  # Tune these parameters
pid_y = PIDController(Kp=50, Ki=10, Kd=0.0)  # Tune these parameters

# Target Position
target_x = 0.5  # Example target x-coordinate
target_y = 0.5  # Example target y-coordinate

# Robot's wheel radius
R = 0.16  # Adjust as per your robot's specification
wheelRadius = 0.023

# Omnibot Connection Settings
HOST = "130.235.83.171"
PORT = 9001

with Connection(HOST, PORT) as bot:
    t0 = time()
    while time() < t0 + 10:  # Run for 10 seconds or until the target is reached
        current_x = bot.get_x()
        current_y = bot.get_y()

        # Calculate errors
        error_x = target_x - current_x
        error_y = target_y - current_y

        # Update PID controllers
        delta_time = 0.1  # Time interval for PID update
        vx = round(pid_x.update(error_x, delta_time))
        vy = round(pid_y.update(error_y, delta_time))

        # Current orientation (theta)
        theta = 0  # Assuming theta is 0 for simplicity, replace with actual value if available

        # Calculate wheel speeds
        wheel_speeds = calculate_wheel_speeds(vx, vy, theta, R)

        print(wheel_speeds)



        speed1 = int (wheel_speeds[0])
        speed2 = int (wheel_speeds[1])
        speed3 = int (wheel_speeds[2])

        # Set wheel speeds
        bot.set_speeds([speed1, speed2, speed3])

        # Print position
        #print('x:', current_x, 'y:', current_y)

        sleep(delta_time)
