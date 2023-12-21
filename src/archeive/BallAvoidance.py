import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

STATE_BOUNDS = [-5, 5]
STATE_BOUNDS_X = [-5, 5]
STATE_BOUNDS_Y = [-5, 5]

class BallAvoidance:

    def calculate_polygon_midpoint(self, vertices):
        """
        :param: vertices that midpoint should be calculated for
        :return: centeroids as np-array for input
        """
        polygon = Polygon(vertices)
        centroid = polygon.centroid
        return centroid.x, centroid.y

    def extend_trajectory(self, ball_trajectory, state_bounds_x, state_bounds_y):
        """
        :param ball_trajcetory: traj. of the ball as np-array containing [x,y] coordinates
        :param state_bounds_x_: [min_x, max_x]
        :param state_bounds_y: [min_y, max_y]
        """

        # Define line segments for the square's borders
        square_borders = [
            np.array([state_bounds_x[0], state_bounds_y[0], state_bounds_x[1], state_bounds_y[0]]),  # Bottom border
            np.array([state_bounds_x[1], state_bounds_y[0], state_bounds_x[1], state_bounds_y[1]]),  # Right border
            np.array([state_bounds_x[1], state_bounds_y[1], state_bounds_x[0], state_bounds_y[1]]),  # Top border
            np.array([state_bounds_x[0], state_bounds_y[1], state_bounds_x[0], state_bounds_y[0]])   # Left border
        ]
        
        # Start and end points of the ball trajectory
        p1, p2 = ball_trajectory[0], ball_trajectory[-1]
        # Calculate coefficients A, B, and C for the line equation Ax + By = C
        A = p2[1] - p1[1]
        B = p1[0] - p2[0]
        C = A * p1[0] + B * p1[1]
        
        # Function to calculate the intersection point of two lines
        def line_intersection(line1, line2):
            # Line 1 coefficients
            A1, B1, C1 = line1
            # Line 2 coefficients
            A2, B2, C2 = line2
            determinant = A1 * B2 - A2 * B1
            if determinant != 0:
                x = (C1 * B2 - C2 * B1) / determinant
                y = (A1 * C2 - A2 * C1) / determinant
                return np.array([x, y])
            return None
        
        # Extend the trajectory by finding intersection points with the square's borders
        extended_trajectory = [ball_trajectory[0]]
        for border in square_borders:
            x1, y1, x2, y2 = border
            border_line = [y2 - y1, x1 - x2, (y2 - y1) * x1 + (x1 - x2) * y1]
            intersection = line_intersection([A, B, C], border_line)
            if intersection is not None:
                if min(x1, x2) <= intersection[0] <= max(x1, x2) and min(y1, y2) <= intersection[1] <= max(y1, y2):
                    extended_trajectory.append(intersection)
        extended_trajectory.append(ball_trajectory[-1])
        return np.array(extended_trajectory)

    def get_centeroids(self, ball_trajectory):
        """
        :param: ball_trajectory = [np.array([ [4.0, 5.5], [2.0, 2.5], [1.8, 1.0], [-1, -1] ])]
        :return: [centeroid_A, centeroid_B]
        """

        state_bounds_x = STATE_BOUNDS_X
        state_bounds_y = STATE_BOUNDS_Y

        # Define vertices of the workspace
        # 
        workspace_vertices = np.array([[state_bounds_x[0], state_bounds_y[0]], [state_bounds_x[0], state_bounds_y[1]],
                                        [state_bounds_x[1], state_bounds_y[1]], [state_bounds_x[1], state_bounds_y[0]],
                                        [state_bounds_x[0], state_bounds_y[0]]])  # Close the loop by repeating the first point

        # Plotting
        plt.figure(figsize=(8, 8))
        
        # Plot the workspace vertices
        plt.plot(workspace_vertices[:, 0], workspace_vertices[:, 1], 'bo-')  # Blue color, circle markers, solid lines

        # Extract the ball trajectory array from the list and extend it
        print("Ball Traj. :" + str(ball_trajectory[0]))
        print("StateX :" + str(state_bounds_x))
        print("StateY :" + str(state_bounds_y))
        extended_ball_trajectory = self.extend_trajectory(ball_trajectory[0], state_bounds_x, state_bounds_y)

        # Plot the extended ball trajectory
        plt.plot(extended_ball_trajectory[:, 0], extended_ball_trajectory[:, 1], 'ro-')  # Red color, circle markers, solid lines

        #plt.show()
        
        print("First position of extended trajectory:", str(extended_ball_trajectory[0]))
        print("Last position of extended trajectory:", str(extended_ball_trajectory[-1]))

        # Set plot titles and labels
        plt.title("Workspace Vertices and Ball Trajectory Visualization")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.grid(True)
        plt.xlim([state_bounds_x[0] - 1, state_bounds_x[1] + 1])  # Set limits for x-axis
        plt.ylim([state_bounds_y[0] - 1, state_bounds_y[1] + 1])  # Set limits for y-axis
        
        # Show the plot
        #plt.show()
        
        # Correct the vertices format for area calculation
        vertices_area_A = [(extended_ball_trajectory[0][0], extended_ball_trajectory[0][1]),
                        (extended_ball_trajectory[-1][0], extended_ball_trajectory[-1][1]),
                        (STATE_BOUNDS_X[0], STATE_BOUNDS_Y[0]),
                        (STATE_BOUNDS_X[0], STATE_BOUNDS_Y[1])]

        vertices_area_B = [(extended_ball_trajectory[0][0], extended_ball_trajectory[0][1]),
                        (extended_ball_trajectory[-1][0], extended_ball_trajectory[-1][1]),
                        (STATE_BOUNDS_X[1], STATE_BOUNDS_Y[0]),
                        (STATE_BOUNDS_X[1], STATE_BOUNDS_Y[1])]

        centeroid_A = self.calculate_polygon_midpoint(vertices_area_A)
        centeroid_B = self.calculate_polygon_midpoint(vertices_area_B)
        
        return [centeroid_A, centeroid_B]
