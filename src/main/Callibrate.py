import requests
from io import BytesIO
from omnibot.tcp import Connection
from time import time, sleep
import os
import numpy as np
import cv2
from main.Omnibot import getCoord, move_to_specific_target
from ControllerCVXPY_Gen2 import ControllerMPC
from WebcamYOLOProcessor import WebcamYOLOProcessor

# Robot connection parameters
HOST = "130.235.83.171"
PORT = 9006

# Camera feed URL parameters
CAMERA_HOST = "philon-11.control.lth.se"
CAMERA_USER = "labwebcam"
CAMERA_PASSWORD = "omnibot-camera"
CAMERA_PORT = 5000
CAMERA_URL = f"http://{CAMERA_USER}:{CAMERA_PASSWORD}@{CAMERA_HOST}:{CAMERA_PORT}"
IMAGE_FOLDER = 'data'
MODEL_RELATIVE_PATH = "runs/detect/train7/weights/best.pt"

# Reference positions in the room (in meters)
POS_CORNER = np.array([-1.0, -0.65])
POS_WINDOW_EDGE = np.array([-1.0, 0.4])
POS_TABLE_END = np.array([1.5, 0.4])
POS_WALL = np.array([1.5, -0.65])

def get_point_on_line(start_pos, end_pos, t):
    """
    Calculates a point on a line segment defined by start and end positions.
    :param start_pos: Starting position of the line segment.
    :param end_pos: Ending position of the line segment.
    :param t: Parameter (0 <= t <= 1) indicating position on the line segment.
    :return: Calculated point on the line.
    """
    return (1 - t) * start_pos + t * end_pos

def get_camera_position(bot):
    """
    Returns the pixel coordinates of the camera's current position.
    :param bot: The robot instance.
    :return: Array of pixel coordinates.
    """
    coordinates = getCoord(bot.get_x(), bot.get_y(), bot.get_theta())
    return np.array([coordinates[0], coordinates[1]])

def callibrate():
    # Establish connection with the robot
    with Connection(HOST, PORT) as bot:
        # Setup model path for the YOLO processor
        model_path = os.path.normpath(os.path.join(os.getcwd(), MODEL_RELATIVE_PATH))
        processor = WebcamYOLOProcessor(CAMERA_HOST, CAMERA_USER, CAMERA_PASSWORD, CAMERA_PORT, model_path)
        
        # Create directory for storing images and data
        os.makedirs(IMAGE_FOLDER, exist_ok=True)

        # Initialize the controller
        controller = ControllerMPC()

        # Arrays for storing coordinates
        pixel_coords = np.empty((0, 2), dtype=np.float32)
        robot_coords = np.empty((0, 2), dtype=np.float32)

        for p0 in range(8):
            p10 = p0 / 7
            for p1 in range(4):
                p11 = p1 / 3
                point = get_point_on_line(get_point_on_line(POS_CORNER, POS_WINDOW_EDGE, p10), 
                                        get_point_on_line(POS_WALL, POS_TABLE_END, p10), p11)
                move_to_specific_target(bot, point, controller)

                # Detect robot and get its coordinates
                robot_x, robot_y = processor.detect_robot_coordinates()
                pixel_coords = np.append(pixel_coords, [[robot_x, robot_y]], axis=0)
                robot_position = get_camera_position(bot)
                robot_coords = np.append(robot_coords, [robot_position], axis=0)

                # Capture and save image
                response = requests.get(CAMERA_URL)
                img_data = BytesIO(response.content)
                image_path = os.path.join(IMAGE_FOLDER, f"currPos.jpg")
                with open(image_path, 'wb') as image_file:
                    image_file.write(img_data.getbuffer())

        # Estimate perspective transformation matrix
        transformation_matrix, _ = cv2.findHomography(robot_coords, pixel_coords, cv2.RANSAC)
        print("Homography Transformation Matrix (M):\n", transformation_matrix)

        # Save the transformation matrix to a file
        matrix_file_path = os.path.join(IMAGE_FOLDER, 'transformation_matrix.txt')
        np.savetxt(matrix_file_path, transformation_matrix)
        print(f"Transformation matrix saved to {matrix_file_path}")


        # # Sample pixel point in the camera image
        # pixel_point = np.array([[50, 50]], dtype=np.float32)

        # # Apply perspective transformation to map the pixel to the floor
        # mapped_point = cv2.perspectiveTransform(np.array([pixel_point]), M)
        # print("Mapped floor point:", mapped_point)
        
