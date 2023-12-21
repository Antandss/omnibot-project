import numpy as np
from DirectionalPIDController import PID_controller
from omnibot.tcp import Connection

class omnibot:
    
    def __init__(self, debugMode = False):
            
            self.controller = PID_controller(debugMode = debugMode)
           
    def set_pos(self,currPos):
        self.currPos = currPos
        self.currPos[2] = self.bot.get_theta()
    
    def get_pos(self):
        return self.currPos
        
    # def get_theta(self):
    #     return self.bot.get_theta()
        

    def stop_robot(self):
        self.bot.set_speeds([0,0,0])
    
    # This method should be removed after programming is done.
    def get_coord(self):
        
        x01 = self.bot.get_x()
        y01 = self.bot.get_y()
        theta01 = self.bot.get_theta()
        # Generate homogeneous transformation matrix.
        # The drone's position and orientation in the 0-frame.
        H01 = np.array([[np.cos(np.deg2rad(theta01)), -np.sin(np.deg2rad(theta01)), x01],
                        [np.sin(np.deg2rad(theta01)), np.cos(np.deg2rad(theta01)), y01],
                        [0, 0, 1]])
        x12 = 0
        y12 = 0.16 

        H12 = np.array([[1, 0, x12],
                        [0, 1, y12],
                        [0, 0, 1]])

        H02 = np.dot(H01, H12)  

        xMiddlePos = H02[0, 2]
        yMiddlePos = H02[1, 2]

        return [xMiddlePos, yMiddlePos,theta01]     
                
    def has_reached_target(self,current_position, target, tolerance=0.05):
        return abs(current_position[0] - target[0]) < tolerance and \
                abs(current_position[1] - target[1]) < tolerance
    
    def vector_to_angle(normalized_vector):
    # Calculate the angle using arctan2 and convert it to the range of -π to π
        angle = np.arctan2(normalized_vector[1], normalized_vector[0])
        
        return angle
            



