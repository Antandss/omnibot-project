#constants
CONTROL_SIGNAL_MAX = 200
CONTROL_SIGNAL_MIN = -200

PREDICTION_HORIZON = 10
CONTROL_HORIZON = 2
#SOLVER_TOLERANCE = 1

CONTROL_D_COST = 0.0
STATE_TAU_VAL = 10

X_COST = 5
Y_COST = 5
ALPHA_COST = 1

SOLVER_MAX_ITERATIONS = 50

ALPHA_MIN = 0
ALPHA_MAX = 360
X_MIN = -4
X_MAX = 5
Y_MIN = -2
Y_MAX = 2

STATE_MIN = -5
STATE_MAX = 5

CV_PENALTY = 10

from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt


class MPCController:
    def __init__(self, initPos):
        # Define the system matrices A, B, C, and D
        R = 0.16
        theta = 0
        wheelRadius = 0.05

        self.TB = np.array([
        [-np.sin(theta), np.cos(theta), R],
        [-np.sin(theta + 2*np.pi/3), np.cos(theta + 2*np.pi/3), R],
        [-np.sin(theta + 4*np.pi/3), np.cos(theta + 4*np.pi/3), R]
        ])

        self.TB2 = np.array([[-np.sin(theta), np.cos(theta), R],
                             [-np.sin(theta + ((2*np.pi)/3)), np.cos(theta + ((2*np.pi)/3)), R],
                             [-np.sin(theta + ((4*np.pi)/3)), np.cos(theta + ((4*np.pi)/3)), R]])

        self.TB2INV = wheelRadius * np.linalg.inv(self.TB2)

        self.A = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]])
        
        self.B = np.array([[0.0038, 0.0, -0.0038],
                           [0.0067, 0.0067, 0.0067],
                           [0.0606, -0.1212, 0.0606]])

        self.B = np.array([ [1.54197642309050e-19, -0.000360843918243516, 0.000360843918243516],
                            [0.000416666666666667, -0.000208333333333333, -0.000208333333333333],
                            [0.00138888888888889,	0.00138888888888889,	0.00138888888888889]])

        self.C = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        self.D = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]])

        self.prediction_horizon = PREDICTION_HORIZON
        self.control_horizon = CONTROL_HORIZON

        # GEKKO model
        self.m = GEKKO(remote=False)
        #self.m.options.OTOL = SOLVER_TOLERANCE
        #self.m.options.RTOL = SOLVER_TOLERANCE

        self.n_inputs = 3
        self.n_outputs = 3

        # Define the time points
        self.time_points = np.arange(0, self.prediction_horizon)

        # Define the inputs and outputs as GEKKO variables
        self.u = [self.m.MV(value=0, lb=CONTROL_SIGNAL_MIN, ub=CONTROL_SIGNAL_MAX) for _ in range(self.n_inputs)]
        self.u[0] = self.m.MV(value=0, lb=CONTROL_SIGNAL_MIN, ub=CONTROL_SIGNAL_MAX)
        self.u[1] = self.m.MV(value=0, lb=CONTROL_SIGNAL_MIN, ub=CONTROL_SIGNAL_MAX)
        self.u[2] = self.m.MV(value=0, lb=CONTROL_SIGNAL_MIN, ub=CONTROL_SIGNAL_MAX)

        # Set the manipulated variables
        for i in range(self.n_inputs):
            self.u[i].status = 1
            self.u[i].dcost = CONTROL_D_COST  # Control move penalty

        self.y = [self.m.CV() for _ in range(self.n_outputs)]
        self.y[0] = self.m.CV(value=initPos[0], lb=X_MIN, ub=X_MAX)
        self.y[1] = self.m.CV(value=initPos[1], lb=Y_MIN, ub=Y_MAX)
        self.y[2] = self.m.CV(value=initPos[2], lb=ALPHA_MIN, ub=ALPHA_MAX)

        # Set the controlled variables
        for i in range(self.n_outputs):
            self.y[i].status = 1
            self.y[i].sp = 5
            self.y[i].TR_INIT = 1
            self.y[i].TAU = STATE_TAU_VAL

        # Define the system dynamics
        self.x = [self.m.Var(value=0) for _ in range(self.n_outputs)]
        
        for i in range(self.n_outputs):
            equation = 0
            for j in range(self.n_outputs):
                equation += self.A[i, j] * self.x[j]
            for j in range(self.n_inputs):
                equation += self.TB2INV[i, j] * self.u[j]
            equation += self.D[i, j] * self.u[j]
            self.m.Equation(self.x[i].dt() == equation)
            self.m.Equation(self.y[i] == sum([self.C[i, j] * self.x[j] for j in range(self.n_outputs)]))


        for i in range(self.n_outputs):
            equation = 0
            for j in range(self.n_outputs):
                equation += self.A[i, j] * self.x[j]
            for k in range(self.n_inputs):  # Change j to k
                equation += self.TB2INV[i, k] * self.u[k]  # Change j to k
            equation += self.D[i, j] * self.u[j]  # Fix the indexing issue here
            self.m.Equation(self.x[i].dt() == equation)
            self.m.Equation(self.y[i] == sum([self.C[i, j] * self.x[j] for j in range(self.n_outputs)]))


        # Define the objective function (typically a sum of squared errors)
        #objective = sum([(self.y[i] - self.y[i].sp) ** 2 for i in range(self.n_outputs)]) # Customize this as needed
        #self.m.Obj(objective)

        objective = sum([(self.y[0] - self.y[0].sp)**X_COST + 
                        (self.y[1] - self.y[1].sp)**Y_COST + 
                        (self.y[2] - self.y[2].sp)**2 * ALPHA_COST])  # Adjust the weight as needed
        self.m.Obj(objective)

        # Set the time points for the prediction horizon
        self.m.time = self.time_points
        self.m.options.SOLVER = 1  # Use APOPT solver
        self.m.options.MAX_ITER = SOLVER_MAX_ITERATIONS  # Increase the maximum number of iterations
        self.m.options.CV_TYPE = 2
        self.m.options.IMODE = 6  # Control

    def get_control_signals(self, current_states, setpoints):
        for i in range(self.n_outputs):
            self.y[i].value = current_states[i]

        # Set the controlled variables
        for i in range(self.n_outputs):
            self.y[i].status = 1
            self.y[i].TR_INIT = 1
            self.y[i].TAU = STATE_TAU_VAL

        for i in range(self.n_outputs):
            self.y[i].SP = setpoints[i]

        # Clear and solve the model
        self.m.solve(disp=False, reset=True, clear=True)
        self.m.solve(disp=False, reset=True)

        # Calc control signals
        optimal_controls = [ui.NEWVAL for ui in self.u]

        #print('Controlled variable 1: ' + str(self.u[0].VALUE))
        #print('Controlled variable 2: ' + str(self.u[1].VALUE))
        #print('Controlled variable 3: ' + str(self.u[2].VALUE))

        print('Optimal 1: ' + str(optimal_controls[0]))
        print('Optimal 2: ' + str(optimal_controls[1]))
        print('Optimal 3: ' + str(optimal_controls[2]))

        print('Current SetPoint 1: ' + str(self.y[0].SP))
        print('Current SetPoint 2: ' + str(self.y[1].SP))
        print('Current SetPoint 3: ' + str(self.y[2].SP))

        return optimal_controls
    
    def set_desired_setpoints(self, setpoints):
        for i in range(self.n_outputs):
            self.y[i].SP = setpoints[i]