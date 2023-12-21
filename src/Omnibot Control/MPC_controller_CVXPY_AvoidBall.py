import cvxpy as cp
import numpy as np
from time import time,sleep
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

# maybe some config parameters here
HORIZON = 35
DT = 0.35
STATE_COST = 20
CONTROL_COST = 0
STATE_BOUNDS = [-5, 5]
STATE_BOUNDS_X = [-5, 5]
STATE_BOUNDS_Y = [-5, 5]


CONTROL_BOUNDS = [-1022, 1022]
N_STATES = 3
N_CONTROLS = 3

BALL_RADIUS = 0.05
BALL_SAFETY_DIST = 0.1

def calculate_polygon_area(vertices):
    polygon = Polygon(vertices)
    area = polygon.area
    return area

def calculate_polygon_midpoint(vertices):
    polygon = Polygon(vertices)
    centroid = polygon.centroid
    return centroid.x, centroid.y

def extend_trajectory(ball_trajectory, state_bounds_x, state_bounds_y):
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

class ControllerMPC:

    def __init__(self):
        # generate and save input-matrix
        theta = np.deg2rad(0)
        wheelRadius = 0.25
        R = 0.16
        self.TB = np.array([[-np.sin(theta), np.cos(theta), R],
                             [-np.sin(theta + ((2*np.pi)/3)), np.cos(theta + ((2*np.pi)/3)), R],
                             [-np.sin(theta + ((4*np.pi)/3)), np.cos(theta + ((4*np.pi)/3)), R]])

        self.B = wheelRadius * np.linalg.inv(self.TB)

    def get_control_action(self, x0, xr, ball_trajectories):
        """
        :param x0: current state
        :param xr: target state
        :param ball_trajectories: trajectories for balls to avoid
        :return: control action u
        """

        # create trajectory variable
        x = cp.Variable((N_STATES, HORIZON + 1))

        # create control variable
        u = cp.Variable((N_CONTROLS, HORIZON))

        # state initial condition
        constraints = [x[:, 0] == x0]
        const_mult = 1

        # add constraints which describe the dynamics
        for t in range(HORIZON):
            # add one constraint to the list of constraints
            # saying that the next x(t+1) should be equal to the current x(t) plus the dynamics dt*B*u
            constraints += [x[:, t + 1] == x[:, t] + const_mult*DT * (self.B @ u[:, t])]


        # avoid balls
        # The input ball_trajectories should be on the form below.
        # Where each row in ball_trajectory represents 1 ball and each column in the arrays represent one timestep. 
        # Leave variable empty if there are no balls.
        
        # ball_trajectory = [
        # np.array([[1.0, 2.0], [1.2, 2.5], [1.5, 3.0]]),  # Positions of the first ball at different time steps
        # np.array([[-1.5, 0.5], [-1.7, 0.3], [-2.0, 0.0]]) ]  # Positions of the second ball at different time steps
        
        # adding constraints for the minimum distance between omnibot and balls
        # REQUIRE FURTHER DEVELOMENT SINCE CONCAVE (?) ATM

        polygon_centeroids = self.get_centeroids(ball_trajectories)

        print("Polygon centeroids: " + str(polygon_centeroids))

        # add constraints which describe the bounds on the states and controls
        for t in range(HORIZON):
            for i in range(N_STATES-1):
                # upper and lower bounds on all states
                constraints += [x[i, t] >= STATE_BOUNDS[0]]
                constraints += [x[i, t] <= STATE_BOUNDS[1]]

            for j in range(N_CONTROLS):
                # upper and lower bounds on all controls
                constraints += [u[j, t] >= CONTROL_BOUNDS[0]]
                constraints += [u[j, t] <= CONTROL_BOUNDS[1]]

        distA = np.sqrt((polygon_centeroids[0][0] - x0[0])**2 + (polygon_centeroids[0][1] - x0[1])**2)
        distB = np.sqrt((polygon_centeroids[1][0] - x0[0])**2 + (polygon_centeroids[1][1] - x0[1])**2)

        print('CENTER: ' + str(polygon_centeroids))

        if distA > distB:
            target = np.array([polygon_centeroids[0][0], polygon_centeroids[0][1], 0, 0])
        else:
            target = np.array([polygon_centeroids[1][0], polygon_centeroids[1][1], 0, 0])

        # define the minimization problem
        XR = np.tile(target, (HORIZON + 1, 1)).T # Make a fat matrix with xr as columns
        prob = cp.Problem(cp.Minimize(cp.sum_squares(x - XR) * STATE_COST + cp.sum_squares(u) * CONTROL_COST), constraints)
                          
        # solve the problem
        prob.solve()

        if prob.status == cp.OPTIMAL:
            # return control signal
            return u.value[:, 0]
        else:
            print("Optimization problem did not solve successfully. Status:", prob.status)
        return np.zeros(N_CONTROLS)  # Return zeros or handle the failure accordingly

        #return u.value[:, 0]    
    
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
        extended_ball_trajectory = extend_trajectory(ball_trajectory[0], state_bounds_x, state_bounds_y)

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

        Area_A = calculate_polygon_area(vertices_area_A)
        Area_B = calculate_polygon_area(vertices_area_B)


        Area_A = calculate_polygon_area(vertices_area_A)
        Area_B = calculate_polygon_area(vertices_area_B)

        print("Area_A: " + str(Area_A))
        print("Area_B: " + str(Area_B))

        centeroid_A = calculate_polygon_midpoint(vertices_area_A)
        centeroid_B = calculate_polygon_midpoint(vertices_area_B)

        return [centeroid_A, centeroid_B]


# Test class
class TestController:
    def __init__(self):
        self.controller = ControllerMPC()

    def simulate(self, initial_state, target_state, ball_trajectory):
        current_state = initial_state.copy()

        for _ in range(10):  # Simulate for 10 time steps
            control_action = self.controller.get_control_action(current_state, target_state, ball_trajectory)
            current_state = current_state + DT * (self.controller.B @ control_action)
            print("Current State:", current_state)
            sleep(0.4)

# Test
initial_state = np.array([0.0, 0.0, 0.0])
target_state = np.array([0.5, -0.5, 0.0])
#ball_trajectory = [np.array([[1.5, 0.5], [1.25, 0.25], [1, 0], [0.75, -0.25],[0.5, -0.5],[0.25, -0.75],[0, -1]]),  # Positions of the first ball at different time steps]
ball_trajectory = [np.array([[2, -1], [4, 5.2], [3, 2.1]])]


test_controller = TestController()
test_controller.simulate(initial_state, target_state, ball_trajectory)