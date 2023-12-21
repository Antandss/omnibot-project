from WebcamYOLOProcessor import WebcamYOLOProcessor
#from MPC_controller_CVXPY import ControllerMPC
from DirectionalPIDController import PID_controller
from Omnibot import omnibot
from omnibot.tcp import Connection
from time import time, sleep
from Line import Line
from AvoidLogic import ballAvoidLogic
import numpy as np
import cv2
import threading


# Robot connection parameters
HOST = "130.235.83.171"
PORT = 9006

# Camera feed URL parameters
CAMERA_HOST = "philon-11.control.lth.se"
CAMERA_USER = "labwebcam"
CAMERA_PASSWORD = "omnibot-camera"
CAMERA_PORT = 5000
CAMERA_URL = f"http://{CAMERA_USER}:{CAMERA_PASSWORD}@{CAMERA_HOST}:{CAMERA_PORT}"
MODEL_RELATIVE_PATH = "runs/detect/train7/weights/best.pt"

PRECISION = 0.2
DT = 0.45
CONST_MULT = 50

# Camera Pixel Coordinates
camera_points = np.float32([
    [512.7, 102.53],
    [164.1, 99.074],
    [118.33, 230.45],
    [545.41, 245.54]
])

# Bot Position Coordinates
bot_points = np.float32([
    [-1.5267, -1.3148],
    [1.8153, -1.4312],
    [2.0286, 0.030514],
    [-1.06, 0.38536]
])

# Calculate the transformation matrix using findHomography
transformation_matrix, _ = cv2.findHomography(camera_points, bot_points)

def move_in_dir(phi, currPos, bot, controller):
    wheelSpeeds = controller.control_step(currPos,phi)
        #print("Received! " + str(wheelSpeeds))
    if wheelSpeeds is not None:
        bot.set_speeds(wheelSpeeds)
    else:
        bot.set_speeds([0]) #To be checked
    

def angle_to_vector(angle):
    # Calculate the x and y components of the normalized vector
    x_component = np.cos(angle)
    y_component = np.sin(angle)

    # Create a normalized vector
    normalized_vector = np.array([x_component, y_component])

    return normalized_vector



def process_camera_coordinates(camera_pixel_coordinates, transformation_matrix):
    pixel_coordinates_bot = None
    pixel_coordinates_asteroid = []

    for item in camera_pixel_coordinates:
        if item[0] == 0:  # Robot
            pixel_coordinates_bot = np.array([[item[2], item[3]]], dtype=np.float32)
        elif item[0] == 1:  # Asteroid
            pixel_coordinates_asteroid.append([item[2], item[3]])

    bot_camera_position = None
    asteroids_camera_positions = None

    if pixel_coordinates_bot is not None:
        transformed_points_bot = cv2.perspectiveTransform(np.array([pixel_coordinates_bot]), transformation_matrix)
        bot_camera_position = np.array([transformed_points_bot[0, 0, 0], transformed_points_bot[0, 0, 1]])

    if pixel_coordinates_asteroid:
        pixel_coordinates_asteroid = np.array(pixel_coordinates_asteroid, dtype=np.float32)
        transformed_points_asteroids = cv2.perspectiveTransform(np.array([pixel_coordinates_asteroid]), transformation_matrix)
        asteroids_camera_positions = transformed_points_asteroids.reshape(-1, 2)

    return bot_camera_position, asteroids_camera_positions

def runAsteroidGame():

    print('Started AsteroidGame.py')

    #connection = Connection(HOST, PORT)
    
    with Connection(HOST, PORT) as bot:
        processor = WebcamYOLOProcessor(CAMERA_HOST, CAMERA_USER, CAMERA_PASSWORD, CAMERA_PORT, MODEL_RELATIVE_PATH)
        LineDetector = Line()
        controller = PID_controller(debugMode=False)

        t_start_time = time()

        while True:
            t0 = time()
            bot_position = np.array([bot.get_x(), bot.get_y()])
            camera_pixel_coordinates = processor.getBoxPositions()  # Read bounding boxes from the camera
            bot_camera_position, asteroids_camera_positions = process_camera_coordinates(camera_pixel_coordinates, transformation_matrix)

            print("Bot camera pos is: " + str(bot_camera_position))
            
           
            if asteroids_camera_positions is None:
                print("No ball in the picture")
                #middleDir = ballAvoidLogic.moveMiddle(bot_camera_position)
                #phi = omnibot.vector_to_angle(middleDir)
                #wheelSpeeds = controller.control_step(bot_camera_position,phi)  # Send middle direction to omnibot
                wheel_speeds = controller.fuzzy_control_step(bot_camera_position[0], bot_camera_position[1], desired_x = 1, desired_y = -0.5, Kp=16)
                
                print("wheel_speeds:" + str(wheel_speeds))
                if wheel_speeds is not None:
                    bot.set_speeds(wheel_speeds)
                else:
                    bot.set_speeds([0])
                
            
            else:
                print("Ball detected")
                lineEquation = LineDetector.get_linear_values(asteroids_camera_positions[0][0], asteroids_camera_positions[0][1])
                print("LineEq: " + str(lineEquation))

                if lineEquation is None:
                    bot.set_speeds([0])
                else:
                    moveDir = ballAvoidLogic.avoid(bot_camera_position,lineEquation) # Calculate direction to move in
                    wheelSpeeds = controller.control_step(bot_camera_position, moveDir) # Send direction to the robot
                    
                    if wheelSpeeds is not None:
                        bot.set_speeds(wheelSpeeds)
                    else:
                        bot.set_speeds([0]) #To be checked
        

            t1 = time()
            print("Sample time :" + str(t1 - t0))
            if DT - (t1 - t0) > 0:
                sleep(DT - (t1 - t0))

               