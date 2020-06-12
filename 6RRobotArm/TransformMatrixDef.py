from numpy import *  # imports all function so we don't have to use np.function()
import math
import numpy as np

# DH Parameter Table for 6 DOF in mm
d1 = 133.90245
a2 = 102.947724
a3 = 102.947724
d4 = 62.768226
d5 = 97.990914
d6 = 58.81878

PT = [[0, 0, 0, d1],
      [0, pi / 2, 0, 0],
      [0, 0, -a2, 0],
      [0, 0, -a3, d4],
      [0, pi / 2, 0, d5],
      [0, -pi / 2, 0, d6]]


def transform_matrix2(ti, alpha_i_m1, a_i_m1, di):
    return [[cos(ti), -sin(ti), 0, a_i_m1],
            [sin(ti) * cos(alpha_i_m1), cos(ti) * cos(alpha_i_m1), -sin(alpha_i_m1), -sin(alpha_i_m1) * di],
            [sin(ti) * sin(alpha_i_m1), cos(ti) * sin(alpha_i_m1), cos(alpha_i_m1), cos(alpha_i_m1) * di],
            [0, 0, 0, 1]]


H0_1 = transform_matrix2(PT[0][0] / 180 * pi, PT[0][1], PT[0][2], PT[0][3])
H1_2 = transform_matrix2(PT[1][0] / 180 * pi, PT[1][1], PT[1][2], PT[1][3])
H2_3 = transform_matrix2(PT[2][0] / 180 * pi, PT[2][1], PT[2][2], PT[2][3])
H3_4 = transform_matrix2(PT[3][0] / 180 * pi, PT[3][1], PT[3][2], PT[3][3])
H4_5 = transform_matrix2(PT[4][0] / 180 * pi, PT[4][1], PT[4][2], PT[4][3])
H5_6 = transform_matrix2(PT[5][0] / 180 * pi, PT[5][1], PT[5][2], PT[5][3])

H0_2 = dot(H0_1, H1_2)
H0_3 = dot(H0_2, H2_3)
H0_4 = dot(H0_3, H3_4)
H0_5 = dot(H0_4, H4_5)
H0_6 = dot(H0_5, H5_6)

Md6 = [[0],
       [0],
       [-d6],
       [1]]
P0_5 = dot(H0_6, Md6)

print("H0_4")
print(H0_4)
Md4 = [[0],
       [0],
       [0],
       [1]]

Tt1 = [[cos(pi/3), sin(pi/3), 0, 0],
       [-sin(pi/3), cos(pi/3), 0, 0],
       [0, 0, 1, -d1]]
print("matrix(H4_5)")
print(matrix(H4_5))
print("matrix(H5_6)")
print(matrix(H5_6))
print("H0_6")
print(H0_6)
P0_4 = dot(dot(H0_6, linalg.inv(dot(H4_5, H5_6))), Md4)
print(P0_4)
H0_4 = dot(H0_6, linalg.inv(dot(H4_5, H5_6)))
H1_4 = dot(linalg.inv(H0_1), H0_4)
H2_4 = dot(linalg.inv(H1_2), H1_4)
H3_4 = dot(linalg.inv(H2_3), H2_4)
P1_4 = dot(Tt1, P0_4)
print(P1_4)
H1_4 = dot(linalg.inv(H0_1), H0_4)
print(H3_4)

print("H0_4")
print(H0_4)
print("H2_3")
print(H2_3)
print("H1_4")
print(H1_4)
print("H2_4")
print(H2_4)
print("H3_4")
print(H3_4)
print("H1_2")
print(H1_2)
print(H0_6)