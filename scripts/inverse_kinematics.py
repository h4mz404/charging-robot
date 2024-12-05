#!/usr/bin/env python3
# Name: Hamza Shah Khan
# UID: 119483152
# email: hamzask@umd.edu
# ## Problem1: UR3 Velocity Kinemtics
#  DH parameters:

import sympy as sp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math


theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
d1, d2, d3, d4, d5, d6 = sp.symbols('d1 d2 d3 d4 d5 d6')
a1, a2, a3, a4, a5, a6 = sp.symbols('a1 a2 a3 a4 a5 a6')
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = sp.symbols('alpha1 alpha2 alpha3 alpha4 alpha5 alpha6')

# DH parameters
a1 = 0
a2 = 0.24365
a3 = 0.21325
a4 = 0
a5 = 0
a6 = 0

d1 = 0.1519
d2 = 0
d3 = 0
d4 = 0.11235
d5 = 0.08535
d6 = 0.0819

alpha1 = -sp.pi/2
alpha2 = 0
alpha3 = 0
alpha4 = sp.pi/2
alpha5 = -sp.pi/2
alpha6 = 0

# Print the DH Table
dh_table = pd.DataFrame({
    'a': [a1, a2, a3, a4, a5, a6],
    'alpha': [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6],
    'd': [d1, d2, d3, d4, d5, d6],
    'theta': [theta1, theta2, theta3, theta4, theta5, theta6]
})

# print("DH Table:")
# sp.pprint(dh_table)

# Homogeneous transformation matrices
def dh_transform_matrix(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])

T1 = dh_transform_matrix(theta1-(sp.pi/2), d1, a1, alpha1)
T2 = dh_transform_matrix(theta2-(sp.pi/2), d2, a2, alpha2)
T3 = dh_transform_matrix(theta3, d3, a3, alpha3)
T4 = dh_transform_matrix(theta4+(sp.pi/2), d4, a4, alpha4)
T5 = dh_transform_matrix(theta5, d5, a5, alpha5)
T6 = dh_transform_matrix(theta6, d6, a6, alpha6)

T = T1 * T2 * T3 * T4 * T5 * T6

# Print the final transformation matrix
# eq7 = sp.Eq(sp.symbols('T0_6'),  T, evaluate=False)
# print("Final Transformation Matrix:")
# sp.pprint(eq7,use_unicode = True, wrap_line = False)

# Calculating Jacobian

#Step1 : Calculate Tranformation matrices T_0_i
T_0_1 =  T1 
T_0_2 = T_0_1 * T2
T_0_3 = T_0_2 * T3
T_0_4 = T_0_3 * T4
T_0_5 = T_0_4 * T5
T_0_6 = T_0_5 * T6

#Step2: Calculate P
P_mat = T_0_6[:3,3]

P_mat_diff_theta1 = sp.diff(P_mat, theta1)
P_mat_diff_theta2 = sp.diff(P_mat, theta2)
P_mat_diff_theta3 = sp.diff(P_mat, theta3)
P_mat_diff_theta4 = sp.diff(P_mat, theta4)
P_mat_diff_theta5 = sp.diff(P_mat, theta5)
P_mat_diff_theta6 = sp.diff(P_mat, theta6)

#Step3 : Calculate Z_i
Z_1=T_0_1[:3,2]
Z_2=T_0_2[:3,2]
Z_3=T_0_3[:3,2]
Z_4=T_0_4[:3,2]
Z_5=T_0_5[:3,2]
Z_6=T_0_6[:3,2]

#Step4 : Calculate Jacobian J_i
J_1 = sp.Matrix([[P_mat_diff_theta1], [Z_1]])
J_2 = sp.Matrix([[P_mat_diff_theta2], [Z_2]])
J_3 = sp.Matrix([[P_mat_diff_theta3], [Z_3]])
J_4 = sp.Matrix([[P_mat_diff_theta4], [Z_4]])
J_5 = sp.Matrix([[P_mat_diff_theta5], [Z_5]])
J_6 = sp.Matrix([[P_mat_diff_theta6], [Z_6]])

#Step5 : Combine Jacobians to get J
J_mat = sp.Matrix([[J_1, J_2, J_3, J_4, J_5, J_6]])


