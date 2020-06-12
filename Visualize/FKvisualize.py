import matplotlib.pyplot as plt
from numpy import *


# Homogeneous Transformation Matrices via Denavit-Hartenberg Method
def transform_matrix(i, DHT):
    return [
        [cos(DHT[i][0]), -sin(DHT[i][0]) * cos(DHT[i][1]), sin(DHT[i][0]) * sin(DHT[i][1]),
         DHT[i][2] * cos(DHT[i][0])],
        [sin(DHT[i][0]), cos(DHT[i][0]) * cos(DHT[i][1]), -cos(DHT[i][0]) * sin(DHT[i][1]),
         DHT[i][2] * sin(DHT[i][0])],
        [0, sin(DHT[i][1]), cos(DHT[i][1]), DHT[i][3]],
        [0, 0, 0, 1]]


def transform_matrix2(ti, alpha_i_m1, a_i_m1, di):
    return [[cos(ti), -sin(ti), 0, a_i_m1],
            [sin(ti) * cos(alpha_i_m1), cos(ti) * cos(alpha_i_m1), -sin(alpha_i_m1), -sin(alpha_i_m1) * di],
            [sin(ti) * sin(alpha_i_m1), cos(ti) * sin(alpha_i_m1), cos(alpha_i_m1), cos(alpha_i_m1) * di],
            [0, 0, 0, 1]]


# class defines robot joint vertices and stores & plots vertex definition
# in cartesian space relative to the origin of the base frame
# those vertices get plotted within MPL's 3d visualization plotter using their plot function.
class SixR:

    def __init__(self, ax, theta=(90, 0, 0, 90, 90, 0)):
        self.ax = ax
        self.d1 = 133.90245
        self.a2 = 102.947724
        self.a3 = 102.947724
        self.d4 = 62.768226
        self.d5 = 97.990914
        self.d6 = 58.81878
        self.theta = theta  # theta angles in degrees
        # self.pt determines Denavit-Hartenberg Table using joint angle, rotational link offset, link offset,
        # and link length, respectively
        self.pt = [[theta[0], 0, 0, self.d1],
                        [theta[1], pi / 2, 0, 0],
                        [theta[2], 0, -self.a2, 0],
                        [theta[3], 0, -self.a3, self.d4],
                        [theta[4], pi / 2, 0, self.d5],
                        [theta[5], -pi / 2, 0, self.d6]]
        # self.fk determines link i frame and link i-1 frame transformation matrices
        self.fk = []
        for i in range(6):
            self.fk.append(transform_matrix2(self.pt[i][0]/180*pi,self.pt[i][1],self.pt[i][2],self.pt[i][3]))

        # determination of transformation matrix for link i frame relative to base frame
        self.H0_1 = self.fk[0]
        self.H0_2 = dot(self.fk[0], self.fk[1])
        self.H0_3 = dot(self.H0_2, self.fk[2])
        self.H0_4 = dot(self.H0_3, self.fk[3])
        self.H0_5 = dot(self.H0_4, self.fk[4])
        self.H0_6 = dot(self.H0_5, self.fk[5])

        # stores limb vertex locations in x,y,z relative to base frame
        self.limb_vertecies = [[self.H0_1[0][3], self.H0_1[1][3], self.H0_1[2][3]],
                               [self.H0_2[0][3], self.H0_2[1][3], self.H0_2[2][3]],
                               [self.H0_3[0][3], self.H0_3[1][3], self.H0_3[2][3]],
                               [self.H0_4[0][3], self.H0_4[1][3], self.H0_4[2][3]],
                               [self.H0_5[0][3], self.H0_5[1][3], self.H0_5[2][3]],
                               [self.H0_6[0][3], self.H0_6[1][3], self.H0_6[2][3]],
                               ]
        print("End Effector Position (X,Y,Z): ")
        print(self.limb_vertecies[5])
        print("Joint 5 Position (X,Y,Z): ")
        print(self.limb_vertecies[4])
        print("Joint 4 Position (X,Y,Z): ")
        print(self.limb_vertecies[3])
        print(self.H0_6)
    # takes stored limb vertices and plots the data points using .plot() function, generating a wireframe representation
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

# user input
T1 = float(input("Enter angle 1 (deg): "))
T2 = float(input("Enter angle 2 (deg): "))
T3 = float(input("Enter angle 3 (deg): "))
T4 = float(input("Enter angle 4 (deg): "))
T5 = float(input("Enter angle 5 (deg): "))
T6 = float(input("Enter angle 6 (deg): "))

robot = SixR(ax, theta=(T1, T2, T3, T4, T5, T6))
robot.draw_limbs()

plt.show()
