import requests
from PIL import Image
from io import BytesIO
from omnibot.tcp import Connection
from time import time, sleep
import random
import csv
import os
import numpy as np
import datetime

# Define robot connection parameters
HOST = "130.235.83.171"
PORT = 9001

# Define camera feed URL parameters
computer = "philon-11.control.lth.se"
usr = "labwebcam"
pwd = "omnibot-camera"
port = 5000
camera_url = "http://" + usr + ":" + pwd + "@" + computer + ":" + str(port)

if __name__ == "__main__":
    

    counter = 0
    while True:
        # Capture an image from the camera feed
        response = requests.get(camera_url)
        img_data = BytesIO(response.content)

        # Create the 'data' folder if it doesn't exist
        data_folder = 'data'
        os.makedirs(data_folder, exist_ok=True)

        # Get current date and time
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS

        # Save the image with a filename containing current date and time
        image_path = os.path.join(data_folder, f"dataset_{current_time}_{counter}.jpg")
        with open(image_path, 'wb') as image_file:
            image_file.write(img_data.getbuffer())

        counter += 1  # Increment counter for unique filenames
        input("Press to capture")
    