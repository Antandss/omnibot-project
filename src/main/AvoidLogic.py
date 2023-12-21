# This python file implements the logic for avoiding a ball. It essentially calculates direction
import numpy as np

class ballAvoidLogic:
    
    def avoid(robotPosition, ballLine):
        print("robotposition: " + str(robotPosition))
        print("ballLine: " + str(ballLine))
        # Extract the slope 'm' and y-intercept 'k' from ballLine
        m, k = ballLine

        # Calculate the slope of the line perpendicular to ballLine
        perpendicular_slope = -1 / k

        # Construct a direction vector [1, perpendicular_slope]
        direction_vector = np.array([1, perpendicular_slope])

        # Normalize the direction vector using NumPy
        normalized_vector = direction_vector / np.linalg.norm(direction_vector)

        return normalized_vector
    
    def moveMiddle(robot_position):
       
        middle_position = np.array([0.5, -0.5])
        #middle_position = np.array([0.0, 0.0])

        robot_position = np.array(robot_position)
        vector = middle_position - robot_position

        # Normalize the vector
        norm = np.linalg.norm(vector)
        if norm != 0:
            normalized_vector = vector / norm
        else:
            # In case the robot is already at the middle, return a zero vector
            normalized_vector = np.array([0, 0])

        return normalized_vector.tolist()