import math

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from pytictoc import TicToc

fig = plt.figure()
ax = Axes3D(fig)

WINDOW_SIZE = 500
ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_zlim3d(0, WINDOW_SIZE)

ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')
# robot.shift_body_rotation()


plt.show()