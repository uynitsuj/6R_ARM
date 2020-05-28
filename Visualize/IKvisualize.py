import matplotlib.pyplot as plt
from numpy import *

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


X = [0, 0, 35.222688000000013, 35.22268800000002, -62.76822599999997, -62.768225999999965, -62.76822599999997]
Y = [0, 0, 153.747724, 307.495448, 307.495448, 405.486362, 405.486362]
Z = [0, 133.90245, 133.90245, 133.90245, 133.90245, 133.90245, 192.72123]


WINDOW_SIZE = 1000
ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_zlim3d(0, WINDOW_SIZE)

ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')

#user input
'''
T1 = float(input("Enter angle 1 (deg): "))
T2 = float(input("Enter angle 2 (deg): "))
T3 = float(input("Enter angle 3 (deg): "))
T4 = float(input("Enter angle 4 (deg): "))
T5 = float(input("Enter angle 5 (deg): "))
T6 = float(input("Enter angle 6 (deg): "))
'''

plt.plot(X,Y,Z)

plt.show()
