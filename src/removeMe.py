import numpy as np

theta = np.pi
wheel_radius = 1
radius_to_middle = 1
vx = 1
vy = 1
TB = np.array([[-np.sin(theta), np.cos(theta), radius_to_middle],
                             [-np.sin(theta + ((2*np.pi)/3)), np.cos(theta + ((2*np.pi)/3)), radius_to_middle],
                             [-np.sin(theta + ((4*np.pi)/3)), np.cos(theta + ((4*np.pi)/3)), radius_to_middle]])


B = (1/wheel_radius) * np.matmul(TB, np.array([[vx], [vy], [0]]))

wheel_speeds = B

print(B)