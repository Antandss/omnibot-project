import requests
from PIL import Image
from io import BytesIO
from ultralytics import YOLO
from time import sleep
import cv2
import numpy as np
import matplotlib.pyplot as plt


# Define a class for processing images from a webcam using the YOLO object detection model.
class WebcamYOLOProcessor:
    def __init__(self, computer, user, password, port, model_path):
        # Initialize the webcam URL and the YOLO model.
        self.url = f"http://{user}:{password}@{computer}:{port}"
        self.model = YOLO(model_path)

    def get_image(self):
        """
        Fetches and returns the webcam image
        """
        response = requests.get(self.url)
        if response.status_code == 200:
            img_data = BytesIO(response.content)
            img = Image.open(img_data)
            return img
        else:
            print(f"Failed to get image. Status code: {response.status_code}")
            return None
    
    def transform_image(self, image, transformation_matrix):
        """
        Applies perspective transformation to the given image.
        :param image: PIL Image to be transformed.
        :param transformation_matrix: The transformation matrix.
        :return: Transformed image.
        """
        if image is not None:
            open_cv_image = np.array(image) 
            open_cv_image = open_cv_image[:, :, ::-1].copy()  # Convert RGB to BGR
            transformed_img = cv2.warpPerspective(open_cv_image, transformation_matrix, (open_cv_image.shape[1], open_cv_image.shape[0]))
            cv2.imshow('Transformed Image', transformed_img)
            return transformed_img
        else:
            return None
        

    def showImage(self):
        """
        Fetches and displays the current webcam image.
        """
        response = requests.get(self.url)
        if response.status_code == 200:
            img_data = BytesIO(response.content)
            img = Image.open(img_data)
            plt.imshow(img)
            plt.axis('off')
            plt.show()
        else:
            print(f"Failed to get image. Status code: {response.status_code}")

    def video_feed(self, transformation_matrix):
        while True:
            raw_image = self.get_image()
            if raw_image is not None:
                try:
                    transformed_image = self.transform_image(raw_image, transformation_matrix)
                    if transformed_image is not None:
                        cv2.imshow('Transformed Video Feed', transformed_image)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                except Exception as e:
                    print(f"Error during image processing: {e}")
            else:
                print("Failed to capture image.")
        cv2.destroyAllWindows()

    def getBoxPositions(self):
        # Get the current webcam image and return detected objects and their positions.
        response = requests.get(self.url)
        if response.status_code == 200:
            img_data = BytesIO(response.content)
            img = Image.open(img_data)
            results = self.model.predict(img, show=False)
            detections = self.extract_detections(results)
        else:
            print(f"Failed to get image. Status code: {response.status_code}")
            detections = []
        return detections

    def run(self):
        # Continuously process images from the webcam until interrupted.
        try:
            while True:
                detections = self.getBoxPositions()
                # Uncomment below to print detections
                # for i, (cls, name, mid_x, mid_y) in enumerate(detections):
                #     print(f"{name} {i + 1}: Middle Position - (X: {mid_x:.2f}, Y: {mid_y:.2f})")
        except KeyboardInterrupt:
            print("Stopped by the user.")
        except Exception as e:
            print(f"An error occurred: {e}")

    def extract_detections(self, results):
        detections = []
        for r in results:
            for i, box in enumerate(r.boxes.xyxy):
                cls = int(r.boxes.cls[i].item())
                name = r.names[cls]
                x1, y1, x2, y2 = box
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2
                detection = [cls, name, mid_x.item(), mid_y.item()]
                detections.append(detection)
    
        #print("Detections: " + str(detections))

        # Check if detections list is empty
        if not detections:
            return []  # Return an empty list if no detections
        else:
            return detections

    
    # Returns the (pixel) coordinates of the first robot it finds.
    def contDetect_robot_coordinates(self):
        """
        Detects the robot in the image and returns its coordinates.

        :param processor: The instance of the WebcamYOLOProcessor.
        :return: Coordinates of the robot if detected, otherwise keeps trying.
        """
        while True:
            detections = self.getBoxPositions()  # This should return the list of detections

            robot_detection = next((d for d in detections if d[0] == 0), None)
            if robot_detection is not None:
                robot_x, robot_y = robot_detection[2], robot_detection[3]
                return robot_x, robot_y
            sleep(0.5)
            
    # Returns the (pixel) coordinates of the first asteroid it finds.
    def contDetect_asteroid_coordinates(self):
        """
        Detects an asteroid in the image and returns its coordinates.

        :param processor: The instance of the WebcamYOLOProcessor.
        :return: Coordinates of the asteroid if detected, otherwise keeps trying.
        """
        while True:
            detections = self.getBoxPositions()  # This should return the list of detections

        # Find the asteroid detection
            asteroid_detection = next((d for d in detections if d[0] == 1), None)
            asteroid_coords = [asteroid_detection[2], asteroid_detection[3]] if asteroid_detection is not None else [None, None]

        # Find the robot detection
            robot_detection = next((d for d in detections if d[0] == 0), None)
            robot_coords = [robot_detection[2], robot_detection[3]] if robot_detection is not None else [None, None]

        # Return the combined list
            combined_coords = asteroid_coords + robot_coords
            return combined_coords
        
    def detect_asteroid_coordinates(self,detections):
        """
        Detects an asteroid in the list of detections and returns its coordinates.

        :param detections: List of detections, where each detection is a tuple or list.
                        The first element is expected to be an identifier, with the second and third elements being the x and y coordinates.
        :return: Tuple of (x, y) coordinates of the asteroid if detected, otherwise None.
        """
        asteroid_detection = next((d for d in detections if d[0] == 1), None)
        if asteroid_detection is not None:
            asteroid_x, asteroid_y = asteroid_detection[2], asteroid_detection[3]
            return self.convert_to_plane_coordinates([asteroid_x, asteroid_y])
        else:
            return None
        
    def detect_robot_coordinates(self,detections):
        """
        Detects the robot in the list of detections and returns its coordinates.

        :param detections: List of detections, where each detection is a tuple or list.
                        The first element is expected to be an identifier, with the second and third elements being the x and y coordinates.
        :return: Tuple of (x, y) coordinates of the robot if detected, otherwise None.
        """
        robot_detection = next((d for d in detections if d[0] == 0), None)
        if robot_detection is not None:
            robot_x, robot_y = robot_detection[2], robot_detection[3]
            return self.convert_to_plane_coordinates([robot_x, robot_y])
        else:
            return None
        
     
    # Converts from the pixel coordinates to the plane coordinates 
    def convert_to_plane_coordinates(self, pixelCoordinates):
        # Adjust the file path to point to the data folder
       
        H = np.array([
            [1, 0, -385.31], 
            [0, 1, -210.91], 
            [0, 0, 1]])
        
        
        H_P = np.array([
            [1, 0, pixelCoordinates[0]], 
            [0, 1, pixelCoordinates[1]], 
            [0, 0, 1]
        ]) 

        H_0 = np.dot(H_P, H)
        #print("H_P: " + str(H_P))
        #print("H_0:" + str(H_0))
        return H_0
    


      
