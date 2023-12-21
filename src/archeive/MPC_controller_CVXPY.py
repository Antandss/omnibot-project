import cvxpy as cp
import numpy as np

# maybe some config parameters here
HORIZON = 10
DT = 0.35
STATE_COST = 10
CONTROL_COST = 0
STATE_BOUNDS = [-2, 2]
CONTROL_BOUNDS = [-1022, 1022]
N_STATES = 3
N_CONTROLS = 3

WHEEL_RADIUS = 0.25
R_VALUE = 0.16

#WHEEL_RADIUS = 0.025
#R_VALUE = 0.16

class ControllerMPC:

    def __init__(self):
        # generate and save input-matrix
        #self.B = np.random.rand(N_STATES, N_CONTROLS)
        theta = np.deg2rad(0)

        self.TB = np.array([[-np.sin(theta), np.cos(theta), R_VALUE],
                             [-np.sin(theta + ((2*np.pi)/3)), np.cos(theta + ((2*np.pi)/3)), R_VALUE],
                             [-np.sin(theta + ((4*np.pi)/3)), np.cos(theta + ((4*np.pi)/3)), R_VALUE]])

        self.B = WHEEL_RADIUS * np.linalg.inv(self.TB)

    #def get_control_action(self, x0, xr, ball_trajectories):
    def get_control_action(self, x0, xr):
        """
        :param x0: current state
        :param xr: target state
        :return: control action u
        """
        # Comment out the two rows below to disregard the theta-pos of the Omnibot. I.e. lock theta = 0
        # theta = np.deg2rad(x0[2]-20)
        # self.update_kinemtaics(theta)

        print('x0 is ' + str(x0))

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

        # define the minimization problem
        XR = np.tile(xr, (HORIZON + 1, 1)).T # Make a fat matrix with xr as columns
        prob = cp.Problem(cp.Minimize(cp.sum_squares(x - XR) * STATE_COST + cp.sum_squares(u) * CONTROL_COST), constraints)
       
        # solve the problem
        #prob.solve()
        prob.solve(verbose=True)

        # return control signal
        return u.value[:, 0]
    
    #def update_kinemtaics(self, theta):
    #    """
    #    :param theta: current theta of Omnibot
    #    """
    #
    #    self.TB = np.array([[-np.sin(theta), np.cos(theta), R_VALUE],
    #                         [-np.sin(theta + ((2*np.pi)/3)), np.cos(theta + ((2*np.pi)/3)), R_VALUE],
    #                         [-np.sin(theta + ((4*np.pi)/3)), np.cos(theta + ((4*np.pi)/3)), R_VALUE]])
    #
    #    self.B = WHEEL_RADIUS * np.linalg.inv(self.TB)


# testing
#controller = ControllerMPC()
#for i in range(100):
#    print(controller.get_control_action(np.random.rand(3), np.random.rand(3), []))

