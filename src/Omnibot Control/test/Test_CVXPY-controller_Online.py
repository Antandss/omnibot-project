from archeive.MPC_controller_CVXPY import ControllerMPC
from omnibot.tcp import Connection
from time import time, sleep
import numpy as np

# Insert suitable IP-address
HOST = "130.235.83.171"
PORT = 9001
DT = 0.35
desired_setpoints = [0.5, -0.5, 0.0]
const_mult = 50


def move_to_specific_target(bot, target, controller, const_mult,precision):
    
    while not has_reached_target([bot.get_x(), bot.get_y(), 0.0], target,tolerance=precision):
        t0 = time()
        current_states = [bot.get_x(), bot.get_y(), 0.0]
        u = controller.get_control_action(current_states, target)
        bot.set_speeds([int(u[0]) * const_mult, int(u[1]) * const_mult, int(u[2]) * const_mult])
        
        t1 = time()
        if DT - (t1 - t0) > 0:
                sleep(DT - (t1 - t0))
        

def generate_random_target(previous_target, x_bounds, y_bounds):
    new_target = [np.random.uniform(*x_bounds), np.random.uniform(*y_bounds), 0.0]
    while abs(new_target[0] - previous_target[0]) < 0.2 or abs(new_target[1] - previous_target[1]) < 0.2:
        new_target = [np.random.uniform(*x_bounds), np.random.uniform(*y_bounds), 0.0]
    return new_target

def has_reached_target(current_position, target, tolerance=0.05):
    return abs(current_position[0] - target[0]) < tolerance and \
           abs(current_position[1] - target[1]) < tolerance
           

