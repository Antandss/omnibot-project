import cv2
import numpy as np
import os
from WebcamYOLOProcessor import WebcamYOLOProcessor
from time import time, sleep


def detect_ball():
    # Define camera feed URL parameters
    computer = "philon-11.control.lth.se"
    usr = "labwebcam"
    pwd = "omnibot-camera"
    port = 5000
    camera_url = f"http://{usr}:{pwd}@{computer}:{port}"

    # Define the relative file path for the YOLO model weights
    relative_path = "runs/detect/train7/weights/best.pt"

    # Construct the absolute model path
    model_path = os.path.normpath(os.path.join(os.getcwd(), relative_path))

    # Initialize the WebcamYOLOProcessor with the camera and model parameters
    processor = WebcamYOLOProcessor(computer, usr, pwd, port, model_path)

    # Lists to store the X and Y coordinates of the detected ball
    postListX, postListY = [], []
    robListX, robListY = [], []

    # Set the delay time for processing each frame
    DT = 0.2
    
    def get_ab_values():
        # Perform linear regression on the collected coordinates
        A, B = np.polyfit(postListX, postListY, 1)
        return A, B

    while True:
        t0 = time()

        # Reload the frame for visualization in each iteration
        org_frame= cv2.imread("data/asteriod.jpg")
        
        frame = cv2.resize(org_frame, (640, 450))

        # Append coordinates to the lists
        position = processor.detect_asteroid_coordinates()
        postListX.append(position[0])
        postListY.append(position[1])
        
        robListX.append(position[2])
        robListY.append(position[3])

        if len(postListX) > 1:  # Ensure there are enough points for regression
            
            distance = np.linalg.norm(np.array([postListX[-1], postListY[-1]]) - np.array([postListX[-2], postListY[-2]]))
            
            #change if needed, distance the balls move before drawing a predicted line
            threshold = 3.0
            
            if distance > threshold:
        
            # Perform linear regression on the collected coordinates
                A, B = get_ab_values()

            # Draw the trajectory line across the entire width of the frame
                x_start, x_end = 0, frame.shape[1]  # Start and end X coordinates
                y_start, y_end = int(A * x_start + B), int(A * x_end + B)
                cv2.line(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)

            # Display the positions
            for posX, posY in zip(postListX, postListY):
                cv2.circle(frame, (int(posX), int(posY)), 10, (255, 182, 193), -1)
                
            for robX, robY in zip(robListX, robListY):
                cv2.circle(frame, (int(robX), int(robY)), 17, (255, 100, 50), -1)

        # Display the frame
        cv2.imshow("Frame", frame)

        # Break the loop with the 'Esc' key
        key = cv2.waitKey(200)
        if key == 27:
            break

        # Sleep to maintain the desired frame rate
        t1 = time()
        sleep(max(0, DT - (t1 - t0)))

    # Clean up: Release the video capture object
    cv2.destroyAllWindows()