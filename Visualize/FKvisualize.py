import matplotlib.pyplot as plt
from numpy import *


# Homogeneous Transformation Matrices
def transform_matrix(i, DHT):
    return [
        [cos(DHT[i][0]), -sin(DHT[i][0]) * cos(DHT[i][1]), sin(DHT[i][0]) * sin(DHT[i][1]),
         DHT[i][2] * cos(DHT[i][0])],
        [sin(DHT[i][0]), cos(DHT[i][0]) * cos(DHT[i][1]), -cos(DHT[i][0]) * sin(DHT[i][1]),
         DHT[i][2] * sin(DHT[i][0])],
        [0, sin(DHT[i][1]), cos(DHT[i][1]), DHT[i][3]],
        [0, 0, 0, 1]]


class SixR:

    def __init__(self, ax,
                 limb_lengths=(133.90245, 23.596092, 153.747724, 58.81878, 153.747724, 97.990914, 97.990914, 58.81878),
                 theta=(90, 0, 0, 90, 90, 0)):
        self.ax = ax
        self.limb_lengths = limb_lengths
        self.theta = theta  # theta angles in degrees

        self.pt = [[theta[0] / 180 * pi, pi / 2, 0, limb_lengths[0]],
                   [theta[1] / 180 * pi, 0, limb_lengths[2], limb_lengths[3] - limb_lengths[1]],
                   [theta[2] / 180 * pi, 0, limb_lengths[4], 0],
                   [theta[3] / 180 * pi, pi / 2, 0, -limb_lengths[5]],
                   [theta[4] / 180 * pi, pi / 2, 0, limb_lengths[6]],
                   [theta[5] / 180 * pi, 0, 0, limb_lengths[7]]]

        self.fk = [transform_matrix(0, self.pt),
                   transform_matrix(1, self.pt),
                   transform_matrix(2, self.pt),
                   transform_matrix(3, self.pt),
                   transform_matrix(4, self.pt),
                   transform_matrix(5, self.pt)]
        self.H0_1 = self.fk[0]
        self.H0_2 = dot(self.fk[0], self.fk[1])
        self.H0_3 = dot(self.H0_2, self.fk[2])
        self.H0_4 = dot(self.H0_3, self.fk[3])
        self.H0_5 = dot(self.H0_4, self.fk[4])
        self.H0_6 = dot(self.H0_5, self.fk[5])
        self.limb_vertecies = [[self.H0_1[0][3], self.H0_1[1][3], self.H0_1[2][3]],
                               [self.H0_2[0][3], self.H0_2[1][3], self.H0_2[2][3]],
                               [self.H0_3[0][3], self.H0_3[1][3], self.H0_3[2][3]],
                               [self.H0_4[0][3], self.H0_4[1][3], self.H0_4[2][3]],
                               [self.H0_5[0][3], self.H0_5[1][3], self.H0_5[2][3]],
                               [self.H0_6[0][3], self.H0_6[1][3], self.H0_6[2][3]],
                               ]
        print("End Effector Position (X,Y,Z): ")
        print(self.limb_vertecies[5])

    def draw_limbs(self):
        limb_vectors = [[0, 0, 0]]
        for i, limb, in enumerate(self.limb_vertecies):
            limb_vectors.append(self.limb_vertecies[i])
        # limb_vectors = [[3, 2, 2], [14, 13, 51], [43, 53, 75]]
        x_data = [vector[0] for vector in limb_vectors]
        y_data = [vector[1] for vector in limb_vectors]
        z_data = [vector[2] for vector in limb_vectors]
        self.ax.plot(x_data, y_data, z_data)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
'''
X = [0, 0, 3.5222688000000013, 3.522268800000002, -6.276822599999997, -6.2768225999999965, -6.276822599999997]
Y = [0, 0, 15.3747724, 30.7495448, 30.7495448, 40.5486362, 40.5486362]
Z = [0, 13.390245, 13.390245, 13.390245, 13.390245, 13.390245, 19.272123]

'''
WINDOW_SIZE = 1000
ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
ax.set_zlim3d(0, WINDOW_SIZE)

ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')
# robot.shift_body_rotation()
T1 = float(input("Enter angle 1 (deg): "))
T2 = float(input("Enter angle 2 (deg): "))
T3 = float(input("Enter angle 3 (deg): "))
T4 = float(input("Enter angle 4 (deg): "))
T5 = float(input("Enter angle 5 (deg): "))
T6 = float(input("Enter angle 6 (deg): "))

robot = SixR(ax, theta=(90+T1, T2, T3, 90+T4, 90+T5, T6))
robot.draw_limbs()

plt.show()
