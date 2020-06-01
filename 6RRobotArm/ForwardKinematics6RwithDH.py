from numpy import *  # imports all function so we don't have to use np.function()

# Link lengths
# a = [5.27175, 0.92898, 6.05306, 2.31570, 6.05306, 3.85791, 3.85791, 2.3157]
a1 = 13.390245  # length of link a1 in cm
a2 = 2.3596092  # length of link a2 in cm
a3 = 15.3747724  # length of link a3 in cm
a4 = 5.881878  # length of link a4 in cm
a5 = 15.3747724  # length of link a5 in cm
a6 = 9.7990914  # length of link a6 in cm
a7 = 9.7990914  # length of link a7 in cm
a8 = 5.881878  # length of link a8 in cm

# Angles
theta_1 = 90  # theta 1 angle in degrees
theta_2 = 0  # theta 2 angle in degrees
theta_3 = 0  # theta 3 angle in degrees
theta_4 = 90  # theta 4 angle in degrees
theta_5 = 90  # theta 5 angle in degrees
theta_6 = 0  # theta 6 angle in degrees

theta_1 = (theta_1 / 180) * pi  # theta 1 in radians
theta_2 = (theta_2 / 180) * pi  # theta 2 in radians
theta_3 = (theta_3 / 180) * pi  # theta 3 in radians
theta_4 = (theta_4 / 180) * pi  # theta 4 in radians
theta_5 = (theta_5 / 180) * pi  # theta 5 in radians
theta_6 = (theta_6 / 180) * pi  # theta 6 in radians

# DH Parameter Table for 6 DOF
PT = [[theta_1, pi / 2, 0, a1],
      [theta_2, 0, a3, a4 - a2],
      [theta_3, 0, a5, 0],
      [theta_4, pi / 2, 0, -a6],
      [theta_5, pi / 2, 0, a7],
      [theta_6, 0, 0, a8]]


# Homogeneous Transformation Matrices
def transform_matrix(i):
    return [[cos(PT[i][0]), -sin(PT[i][0]) * cos(PT[i][1]), sin(PT[i][0]) * sin(PT[i][1]), PT[i][2] * cos(PT[i][0])],
            [sin(PT[i][0]), cos(PT[i][0]) * cos(PT[i][1]), -cos(PT[i][0]) * sin(PT[i][1]), PT[i][2] * sin(PT[i][0])],
            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
            [0, 0, 0, 1]]


H0_1 = transform_matrix(0)
H1_2 = transform_matrix(1)
H2_3 = transform_matrix(2)
H3_4 = transform_matrix(3)
H4_5 = transform_matrix(4)
H5_6 = transform_matrix(5)


print("H0_1 =")
print(matrix(H0_1))
print("H1_2 =")
print(matrix(H1_2))
print("H2_3 =")
print(matrix(H2_3))
print("H3_4 =")
print(matrix(H3_4))
print("H4_5 =")
print(matrix(H4_5))
print("H5_6 =")
print(matrix(H5_6))

H0_2 = dot(H0_1, H1_2)
H0_3 = dot(H0_2, H2_3)
H0_4 = dot(H0_3, H3_4)
H0_5 = dot(H0_4, H4_5)
H0_6 = dot(H0_5, H5_6)

print("H0_1 =")
print(matrix(H0_1))
print("H0_2 =")
print(matrix(H0_2))
print("H0_3 =")
print(matrix(H0_3))
print("H0_4 =")
print(matrix(H0_4))
print("H0_5 =")
print(matrix(H0_5))
print("H0_6 =")
print(matrix(H0_6))


print(H0_1[0][3])
print(H0_1[1][3])
print(H0_1[2][3])
print("\n")
print(H0_2[0][3])
print(H0_2[1][3])
print(H0_2[2][3])
print("\n")
print(H0_3[0][3])
print(H0_3[1][3])
print(H0_3[2][3])
print("\n")
print(H0_4[0][3])
print(H0_4[1][3])
print(H0_4[2][3])
print("\n")
print(H0_5[0][3])
print(H0_5[1][3])
print(H0_5[2][3])
print("\n")
print(H0_6[0][3])
print(H0_6[1][3])
print(H0_6[2][3])