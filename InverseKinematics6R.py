from numpy import *  # imports all function so we don't have to use np.function()
import math

# DH Parameter Table for 6 DOF in mm
d1 = 133.90245
a2 = 153.747724
a3 = 153.747724
d4 = 62.768226
d5 = 97.990914
d6 = 58.81878

PT = [[0, 0, 0, d1],
      [0, pi / 2, 0, 0],
      [0, 0, -a2, 0],
      [0, 0, -a3, d4],
      [0, pi / 2, 0, d5],
      [0, -pi / 2, 0, d6]]


def transform_matrix(ind):
    return [[cos(PT[ind][0]), -sin(PT[ind][0]) * cos(PT[ind][1]), sin(PT[ind][0]) * sin(PT[ind][1]),
             PT[ind][2] * cos(PT[ind][0])],
            [sin(PT[ind][0]), cos(PT[ind][0]) * cos(PT[ind][1]), -cos(PT[ind][0]) * sin(PT[ind][1]),
             PT[ind][2] * sin(PT[ind][0])],
            [0, sin(PT[ind][1]), cos(PT[ind][1]), PT[ind][3]],
            [0, 0, 0, 1]]


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
print(P0_5)
print("\n")
print(H0_5)
print("\n")
print(H0_4)

R = (sqrt(square(P0_5[1][0]) + square(P0_5[0][0])))

t1 = []
val = math.atan2(P0_5[1][0], P0_5[0][0]) + math.acos(d4 / R) + pi / 2
if -0.005 < val < 0.005:
    t1.append(0)
else:
    t1.append(val)

val = math.atan2(P0_5[1][0], P0_5[0][0]) - math.acos(d4 / R) + pi / 2
if -0.005 < val < 0.005:
    t1.append(0)
else:
    t1.append(val)

val = (H0_6[0][3] * sin(t1[0]) - H0_6[1][3] * cos(t1[0]) - d4) / d6

t5 = []

if 1 > val > -1:
    t5.append(math.acos((H0_6[0][3] * sin(t1[0]) - H0_6[1][3] * cos(t1[0]) - d4) / d6))
    t5.append(-math.acos((H0_6[0][3] * sin(t1[0]) - H0_6[1][3] * cos(t1[0]) - d4) / d6))
elif val > 1 > val - 0.005 and val > 0:
    if 0 not in t5:
        t5.append(math.acos(1))
elif val + 0.005 > -1 > val and val < 0:
    if pi not in t5:
        t5.append(math.acos(-1))

val = (H0_6[0][3] * sin(t1[1]) - H0_6[1][3] * cos(t1[1]) - d4) / d6
if 1 > val > -1:
    t5.append(math.acos((H0_6[0][3] * sin(t1[1]) - H0_6[1][3] * cos(t1[1]) - d4) / d6))
    t5.append(-math.acos((H0_6[0][3] * sin(t1[1]) - H0_6[1][3] * cos(t1[1]) - d4) / d6))
elif val > 1 > val - 0.005 and val > 0:
    if 0 not in t5:
        t5.append(math.acos(1))
elif val + 0.005 > -1 > val and val < 0:
    if pi not in t5:
        t5.append(math.acos(-1))

t6 = []

for g in t1:
    for i in t5:
        if sin(i) == 0:
            if 0 not in t6:
                t6.append(0)
        elif sin(i) != 0:
            if round(math.atan2(((-H0_6[1][0] * sin(g) + H0_6[1][1] * cos(g)) / sin(i)),
                                ((H0_6[0][0] * sin(g) - H0_6[0][1] * cos(g)) / sin(i))), 7) not in t6:
                t6.append(round(math.atan2(((-H0_6[1][0] * sin(g) + H0_6[1][1] * cos(g)) / sin(i)),
                                           ((H0_6[0][0] * sin(g) - H0_6[0][1] * cos(g)) / sin(i))), 3))

print("\n")
print("Theta 1: ")
for o in t1:
    print(o / pi * 180)

print("\n")
print("Theta 5: ")
for f in t5:
    print(f / pi * 180)

print("\n")
print("Theta 6: ")
for h in t6:
    print(h / pi * 180)

print("\n")
print("Theta 3: ")
