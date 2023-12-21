from WebcamYOLOProcessor import WebcamYOLOProcessor
from MPC_controller_CVXPY import ControllerMPC
from BallAvoidance import BallAvoidance
import os
import numpy as np
import omnibotMethods
from omnibot.tcp import Connection
from time import time, sleep

# Robot connection parameters
HOST = "130.235.83.171"
PORT = 9005

# Camera feed URL parameters
CAMERA_HOST = "philon-11.control.lth.se"
CAMERA_USER = "labwebcam"
CAMERA_PASSWORD = "omnibot-camera"
CAMERA_PORT = 5000
CAMERA_URL = f"http://{CAMERA_USER}:{CAMERA_PASSWORD}@{CAMERA_HOST}:{CAMERA_PORT}"
IMAGE_FOLDER = 'data'
MODEL_RELATIVE_PATH = "runs/detect/train7/weights/best.pt"
ABSOLUTE_PATH = "C:\\Users\\ludvi\\OneDrive - Lund University\\LTH\\HT23\\Project\\ProjectGroupF-repo\\group-f\\runs\\detect\\train7\\weights\\best.pt"


PRECISION = 0.2
DT = 0.5
CONST_MULT = 50

if __name__ == "__main__":

    with Connection(HOST, PORT) as bot:

        model_path = os.path.normpath(os.path.join(os.getcwd(), MODEL_RELATIVE_PATH))

        processor = WebcamYOLOProcessor(CAMERA_HOST, CAMERA_USER, CAMERA_PASSWORD, CAMERA_PORT, ABSOLUTE_PATH)
        ballAvoid = BallAvoidance()
        controller = ControllerMPC()

        target=[0.5, 0.5, 0]

        while True:
            t0 = time()
            current_states = [bot.get_x(), bot.get_y(), bot.get_theta()]
            u = controller.get_control_action(current_states, target)
            print('Current_states: ' + str(current_states))

            bot.set_speeds([int(u[0]), int(u[1]), int(u[2])])

            t1 = time()
            if DT - (t1 - t0) > 0:
                sleep(DT - (t1 - t0))