import math

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from pytictoc import TicToc

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X = np.array([[0, 0, 3.5222688000000013, 3.522268800000002, -6.276822599999997, -6.2768225999999965, -6.276822599999997]])
Y = np.array([[0, 0, 15.3747724, 30.7495448, 30.7495448, 40.5486362, 40.5486362]])
Z = np.array([[0, 13.390245, 13.390245, 13.390245, 13.390245, 13.390245, 19.272123]])
WINDOW_SIZE = 100
ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_zlim3d(0, WINDOW_SIZE)

ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')
# robot.shift_body_rotation()

ax.plot_wireframe(X, Y, Z)

plt.show()
