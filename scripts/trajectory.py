
def rec_pose_velocity(x1, y1, x2, y2, v):
    d = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    x_dot = v * (x2-x1) / d
    y_dot = v * (y2-y1) / d
    return x_dot, y_dot

def pose_vel(t):
    # global h, k, r, v, path_x, path_y, path_z

    # print(t)
    if t < t_semi and t >= 0:
        # print("Semicircle")
        #For Semi circle
        x_dot = - r * W * np.sin(W * t)
        y_dot = r * W * np.cos(W * t)
        return x_dot, y_dot

    elif t >= t_semi and t <= total_time:
        t_rect_segment = t - t_semi

        #semicricle to point A: (h-r,k) to (h-r,k-r)
        if t_rect_segment >= 0 and t_rect_segment < t1:
            # print("Point o to A")
            x_dot, y_dot = rec_pose_velocity(h - r, k, h - r, k - r, v)
            return x_dot, y_dot
        
        #Pause at point A
        elif t_rect_segment >= t1 and t_rect_segment < t1 + pause_time:
            # print("Pause at A")
            x_dot = 0
            y_dot = 0
            return x_dot, y_dot
        
        #point A to point B: (h-r,k-r) to (h+r,k-r)
        elif (t_rect_segment >= t1 + pause_time) and (t_rect_segment < t1 + pause_time + t2):
            # print("A to B")
            x_dot, y_dot = rec_pose_velocity(h - r, k - r, h + r, k - r, v)
            return x_dot, y_dot
        
        #Pause at point B
        elif t_rect_segment >= t1 + pause_time + t2 and t_rect_segment < t1 + pause_time + t2 + pause_time:
            # print("Pause at B")
            x_dot = 0
            y_dot = 0
            return x_dot, y_dot
        
        #point B to point S: (h+r,k-r) to (h+r,k-r)
        elif (t_rect_segment >= t1 + pause_time+ t2 + pause_time) and (t_rect_segment < t1 + t2 + t3 + 2*pause_time):
            # print("B to S")
            x_dot, y_dot = rec_pose_velocity(h + r, k - r, h + r, k, v)
            return x_dot, y_dot
        
        #Pause at point S
        elif t_rect_segment >= t1 + t2 + t3 + 2*pause_time and t_rect_segment < t1 + t2 + t3 + 3*pause_time:
            # print("Pause at S")
            x_dot = 0
            y_dot = 0
            return x_dot, y_dot
        


def line_velocity(t):
    X_dot, Z_dot = rec_pose_velocity(h - r, k, h + r, k, v)
    # Y_dot = 0
    return X_dot, Z_dot

#Center of circle

h,k = 0.0,1424.11

total_time = 20


samples = 1000
t_samples = np.linspace(0,total_time,samples)
time_step = total_time / samples
path_x = []
path_y = []
path_z = []


V_matrix_vals = []
q_dot_values = []
q_mat_values = []

# q_matrix = sp.Matrix([0,sp.pi/2,0,sp.pi/2,0,0])
q_matrix = sp.Matrix([0.0,0.0,0.0,0.0,0.0,0.0])

J = J_mat

for t in t_samples:
    # print(trajectory(t))
    X_dot, Z_dot = pose_vel(t)

    Y_dot = 0.0

    #Position velocitiy Matrix
    V_matrix = sp.Matrix([X_dot, Y_dot, Z_dot, 0.0, 0.0, 0.0])

    #Update the jacobian with the new joint angles
    J_matrix = J.subs({theta1:q_matrix[0], theta2:q_matrix[1], theta3:q_matrix[2], theta4:q_matrix[3], theta5:q_matrix[4], theta6:q_matrix[5]})

    #Calculate the pseudo inverse of the jacobian
    J_inv = J_matrix.pinv()

    #Calculate the joint velocities
    q_dot = (J_matrix.pinv() * V_matrix).evalf()

    #Update the joint angles
    q_matrix = q_matrix + (q_dot * time_step)

    q_dot_values.append(q_dot)
    V_matrix_vals.append(V_matrix)
    q_mat_values.append(q_matrix)

    #Calculate the transformation matrix
    T_matrix = T.subs({theta1:q_matrix[0], theta2:q_matrix[1], theta3:q_matrix[2], theta4:q_matrix[3], theta5:q_matrix[4], theta6:q_matrix[5]}).evalf()

    #Extract the position of the end effector
    X = T_matrix[0,3]
    Y = T_matrix[1,3]
    Z = T_matrix[2,3]

    #Store the position of the end effector
    path_x.append(X)
    path_y.append(Y)
    path_z.append(Z)



#2D Plot
plt.figure(figsize=(10, 10))
plt.plot(path_x, path_z, label='Path in X-Z plane')
# plt.scatter([h], [k], color="orange", label="Center (h, k)")
# plt.scatter([h - r, h - r, h + r, h + r], [k, k - r, k - r, k], color="red", label="Rectangle Vertices")
plt.xlabel('X (cm)')
plt.ylabel('Z (cm)')
plt.title('2D Path of End Effector in X-Z Plane')
plt.legend()
plt.grid(True)
plt.show()

