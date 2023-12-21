from ControllerCVXPY_Gen2 import ControllerMPC
from omnibot.tcp import Connection
from time import time, sleep
import numpy as np

# Robot connection parameters
HOST = "130.235.83.171"
PORT = 9006
DT = 0.35

if __name__ == "__main__":

    with Connection(HOST, PORT) as bot:
        controller = ControllerMPC()
        target=[0.0, -0.5, 0]

        while True:
            t0 = time()
            current_states = [bot.get_x(), bot.get_y(), 0]
            u = controller.get_control_action(current_states, target)
            print('Current_states: ' + str(current_states))
            print('Control signal u: ' + str(u))
            
            bot.set_speeds([int(u[0]), int(u[1]), int(u[2])])

            t1 = time()
            if DT - (t1 - t0) > 0:
                sleep(DT - (t1 - t0))