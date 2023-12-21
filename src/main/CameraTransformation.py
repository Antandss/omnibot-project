import requests
from PIL import Image
from io import BytesIO
from WebcamYOLOProcessor import WebcamYOLOProcessor
from Omnibot import omnibot
import numpy as np
from omnibot.tcp import Connection
import cv2

HOST = "130.235.83.171"
PORT = 9006

CAMERA_HOST = "philon-11.control.lth.se"
CAMERA_USER = "labwebcam"
CAMERA_PASSWORD = "omnibot-camera"
CAMERA_PORT = 5000
CAMERA_URL = f"http://{CAMERA_USER}:{CAMERA_PASSWORD}@{CAMERA_HOST}:{CAMERA_PORT}"
MODEL_RELATIVE_PATH = "runs/detect/train7/weights/best.pt"

def convert_to_plane_coordinates(pixelCoordinates):
    matrix_file_path = 'data/transformation_matrix.txt'

    # Read the transformation matrix from the file
    with open(matrix_file_path, 'r') as file:
        matrix_data = file.read()
        matrix = np.fromstring(matrix_data, sep=' ').reshape(3, 3)

    # Convert the pixel coordinates to numpy array
    pixel_point = np.array([pixelCoordinates], dtype=np.float32)

    # Apply the perspective transformation
    mapped_point = cv2.perspectiveTransform(np.array([pixel_point]), matrix)

    # Return the transformed coordinates
    return mapped_point

def transformation(bot_x, bot_y, bot_camera_x, bot_camera_y, theta):
    H_0 = np.array([
        [np.cos(theta), -np.sin(theta), bot_x],
        [np.sin(theta), np.cos(theta), bot_y],
        [0, 0, 1]
    ])

    H_P = np.array([
        [np.cos(theta), -np.sin(theta), bot_camera_x],
        [np.sin(theta), np.cos(theta), bot_camera_y],
        [0, 0, 1]
    ])

    H = np.dot(H_0, np.linalg.inv(H_P))
    return H

def CameraTransformationMatrix(theta, bot_camera_x, bot_camera_y):
    H_P = np.array([
        [np.cos(theta), -np.sin(theta), bot_camera_x],
        [np.sin(theta), np.cos(theta), bot_camera_y],
        [0, 0, 1]
    ])
    return H_P


with Connection(HOST, PORT) as bot:
    processor = WebcamYOLOProcessor(CAMERA_HOST, CAMERA_USER, CAMERA_PASSWORD, CAMERA_PORT, MODEL_RELATIVE_PATH)
   
    camera_pixel_coordinates = processor.getBoxPositions() # For each object in frame
    bot_position = np.array([bot.get_x(), bot.get_y()])
    
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

    print("Transformation Matrix:")
    print(transformation_matrix)

    # Example: Transforming a single point
    if camera_pixel_coordinates[0] == 0 and len(camera_pixel_coordinates) >= 2:
        pixel_coordinates_bot = np.array([[camera_pixel_coordinates[0][2], camera_pixel_coordinates[0][3]]], dtype=np.float32)
        pixel_coordinates_asteroid = np.array([[camera_pixel_coordinates[1][2], camera_pixel_coordinates[1][3]]], dtype=np.float32)
    else:
        pixel_coordinates_bot = np.array([[camera_pixel_coordinates[0][2], camera_pixel_coordinates[0][3]]], dtype=np.float32)
    
    # Apply the perspective transformation
    transformed_points_bot = cv2.perspectiveTransform(np.array([pixel_coordinates_bot]), transformation_matrix)
    transformed_points_asteroids = cv2.perspectiveTransform(np.array([pixel_coordinates_asteroid]), transformation_matrix)

    print("transformed_points_bot: " + str(transformed_points_bot))
    print("transformed_points_asteroids: " + str(transformed_points_asteroids))
       
    bot_camera_position = np.array([])

    # Create the desired numpy array
    point = np.array([x, y])


    print("pixelCoordinates: " + str(pixelCoordinates))
    print("Transformed Point:", point)
    print("bot_pos: " + str(bot_pos))
    
    









    
    
    
