import requests
from PIL import Image
from io import BytesIO
from omnibot.tcp import Connection
from time import time, sleep
import random
import csv
import os
import numpy as np

# Define robot connection parameters
HOST = "130.235.83.171"
PORT = 9001

# Define camera feed URL parameters
computer = "philon-11.control.lth.se"
usr = "labwebcam"
pwd = "omnibot-camera"
port = 5000
camera_url = "http://" + usr + ":" + pwd + "@" + computer + ":" + str(port)






# Open a CSV file for recording data
with open('robot_dataset.csv', 'w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['x', 'y', 'orientation', 'image_path'])

    # Number of iterations to generate dataset
    num_iterations = 400
    
    with Connection(HOST, PORT) as bot:
        
        timeBetweenPictures = 3
        i = 0
        for _ in range(num_iterations):
            # Generate a random x, y, and orientation
            x = random.uniform(-150, 150)  # Adjust the range as needed
            y = random.uniform(-150, 150) 
            z = random.uniform(-150,150) 
            x_int = int(x)
            y_int = int(y)
            z_int = int(z)   # Adjust the range as needed
           
           
           
            # Retreive current time 
            t0 = time()    
            t1 = t0 + timeBetweenPictures  
            bot.set_speeds([0,0,0])
            
            # Sleep until desired time has passed.
            sleep(t1 - time())
            bot.set_speeds([x_int,y_int,z_int])
            # Move the robot to the generated position and orientation
            # You can adapt this part to your robot's specific control commands
            # Set robot position and orientation
            x_p = bot.get_x()
            y_p = bot.get_y()
            theta = bot.get_theta()
        
            print("Position \n X: " + str(x_p) + " \n Y: " + str(y_p) + "\n Theta: " + str(theta))
           
            
            # Capture an image from the camera feed
            response = requests.get(camera_url)
            img_data = BytesIO(response.content)
            
            print("Photo taken")
            
            # Create the 'data' folder if it doesn't exist
            data_folder = 'data'
            os.makedirs(data_folder, exist_ok=True)

            # Save the image with a unique name
            image_path = os.path.join(data_folder, f"currPos.jpg")
            #image_path = os.path.join(data_folder, f"picture_{i}.jpg")
            with open(image_path, 'wb') as image_file:
                image_file.write(img_data.getbuffer())
                
            print("Im here")

            # Record the x, y, orientation, and image path in the CSV file
            csv_writer.writerow([x_p, y_p, theta, image_path])
            i = i+1

print("Dataset generation completed.")