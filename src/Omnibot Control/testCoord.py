import requests
from io import BytesIO
from omnibot.tcp import Connection
from time import time, sleep
import os
import numpy as np
from main.Omnibot import getCoord

# Define robot connection parameters
HOST = "130.235.83.171"
PORT = 9006

# Define camera feed URL parameters
computer = "philon-11.control.lth.se"
usr = "labwebcam"
pwd = "omnibot-camera"
port = 5000
camera_url = "http://" + usr + ":" + pwd + "@" + computer + ":" + str(port)
timeBetweenPictures = 3

if __name__ == "__main__":
    
    
    with Connection(HOST, PORT) as bot:
        while(True):        
            
            # Retreive current time 
            t0 = time()    
            t1 = t0 + timeBetweenPictures  
            bot.set_speeds([0,0,0])
            
            # Sleep until desired time has passed.
            sleep(t1 - time())
            
            # Set speed
            bot.set_speeds([0,0,0])
            # Read position
            x_p = bot.get_x()
            y_p = bot.get_y()
            theta = bot.get_theta()
        
            getCoord(x_p,y_p,theta)
        
            
            # Capture an image from the camera feed
            response = requests.get(camera_url)
            y_p = bot.get_y()
            theta = bot.get_theta()
        
            img_data = BytesIO(response.content)
            
            print("Photo taken")
            y_p = bot.get_y()
            theta = bot.get_theta()
        
            y_p = bot.get_y()
            theta = bot.get_theta()
        
            
            # Create the 'data' folder if it doesn't exist
            image_folder = 'data'
            os.makedirs(image_folder, exist_ok=True)

            # Save the image with a unique name
            image_path = os.path.join(image_folder, f"currPos.jpg")
            with open(image_path, 'wb') as image_file:
                image_file.write(img_data.getbuffer())
                


 
            y_p = bot.get_y()
            theta = bot.get_theta()
        