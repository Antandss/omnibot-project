#get x and y positions from asteriodgame
#send none the first iteration since not enough values
#append all in a list (save) and then do linear regression and return A and B values
import numpy as np
class Line:
    
    def __init__(self):
        self.posListX = []
        self.posListY = []
    
    def transform_line_with_file(self,m, k):
        # Read the transformation matrix from the file
        with open('data/transformation_matrix.txt', 'r') as file:
            matrix_data = file.read()
            transformation_matrix = np.fromstring(matrix_data, sep=' ').reshape(3, 3)

        # Create a column vector [m, k, 1]
        line_vector = np.array([m, k, 1])

        # Apply the transformation matrix
        transformed_vector = np.dot(transformation_matrix, line_vector)

        return transformed_vector

    def get_linear_values(self,x, y):
        self.posListX.append(x)
        self.posListY.append(y)
        
        if len(self.posListX) >= 2:
            k, m = np.polyfit(self.posListX, self.posListY, 1)
            #lineEquation = self.transform_line_with_file(m, k)
            return k, m
        else:
            return None

