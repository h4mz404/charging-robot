import numpy as np
import matplotlib.pyplot as plt
from inverse_kinematics import *
# Example function for forward kinematics
def forward_kinematics(joint_angles):
    # Replace with your specific kinematics model (e.g., D-H parameters)
        #Calculate the transformation matrix
    T_matrix = T.subs({theta1:joint_angles[0], theta2:joint_angles[1], theta3:joint_angles[2], theta4:joint_angles[3], theta5:joint_angles[4], theta6:joint_angles[5]}).evalf()

    #Extract the position of the end effector
    X = T_matrix[0,3]
    Y = T_matrix[1,3]
    Z = T_matrix[2,3]

    return np.array([X, Y, Z])

# Joint limits
joint_ranges = [(-np.pi, np.pi) for _ in range(7)]  # Modify based on your robot

# Sampling joint angles
samples = 100  # Adjust for precision
joint_angles = [np.linspace(joint_range[0], joint_range[1], samples) for joint_range in joint_ranges]
grid = np.meshgrid(*joint_angles)

# Calculate workspace
workspace = []
for idx in range(grid[0].size):
    joint_config = [grid[j].flat[idx] for j in range(7)]
    end_effector_position = forward_kinematics(joint_config)
    workspace.append(end_effector_position)

workspace = np.array(workspace)

# Plot workspace
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(workspace[:, 0], workspace[:, 1], workspace[:, 2], s=1)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
plt.show()
