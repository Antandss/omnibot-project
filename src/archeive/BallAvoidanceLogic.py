import numpy as np


#parametrar
# - robotPos (x,y,theta)
# - ball_line (k,m)
# 
# returns v_x and v_y speeds for bot

class BallAvoidanceLogic:

    def __init__(self):
        #Some stuff
        print('init done')

    def makeChoice(robotPos, asteroidPos):
        """
        Decides the angle to move towards in order to avoid eventual asteroids. If there is no incoming ateroids, set angle for going to middle.
        
        :param: robotPos [x,y,theta] for the robot
        :param: AsteroidPOs [] for the asteroid, none if there is no ateroid
        :return: phi, angle for the direction to move to avoid asteroids ord going towards middle
        """
