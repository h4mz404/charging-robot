#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
from scipy.spatial.transform import Rotation as R
from inverse_kinematics import *


class jointPosPubNode(Node):

    def __init__(self):
        super().__init__('joint_position_pub_node')
        #initialize the publishers and subscribers
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        #Initizalize the variables
        self.goalPos = [0.0, 0.0, 0.0]
        self.trajectory = np.array([[]]).astype(np.float64)
        self.moveFlag = False
        self.dt = 0.1
        self.iter = 0
        #Initial Joint angles
        self.jointAngles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).astype(np.float64)
        #Move the end effector to the initial position
        joint_positions = Float64MultiArray()   
        joint_positions.data = [self.jointAngles[0], self.jointAngles[1], self.jointAngles[2], self.jointAngles[3], self.jointAngles[4], self.jointAngles[5], 0.0]
        self.joint_position_pub.publish(joint_positions)
        self.get_logger().info(f'Joint Angles: {self.jointAngles}')
        #Calc the initial position of the end effector
        self.currentPos, self.currentOrientation = self.forward_kinematics(self.jointAngles)
        self.get_logger().info(f'Current Position: {self.currentPos}')
        self.get_logger().info(f'Current Orientation: {self.currentOrientation}')
        #Get the goal position
        self.getGoalPos()
        #TODO change if goal orientation is different from current orientation
        # self.currentOrientation = np.array([0.0, 0.0, 0.0]).astype(np.float64)
        self.goalOrientation = self.currentOrientation
        self.get_logger().info(f'Goal Position: {self.goalPos}')
        self.get_logger().info(f'Goal Orientation: {self.goalOrientation}')
        #Get the trajectory
        self.getTrajectory()


    def getGoalPos(self):
        #Start position 0.347550000000000 0 0.694150000000000]
        #0.347550000000000 0.243650000000000 0.450500000000000
        self.goalPos = np.array([0.34755, 0.01, 0.40]).astype(np.float64)
        self.moveFlag = True

    def getTrajectory(self):
        self.currentPos = np.array(self.currentPos, dtype=np.float64)
        self.goalPos = np.array(self.goalPos, dtype=np.float64) 
        print(f'Current Position 0 datatype: {type(self.currentPos[0])}')
        iter_num = 1000
        xArray = np.linspace(self.currentPos[0]  , self.goalPos[0], num=iter_num)
        yArray = np.linspace(self.currentPos[1], self.goalPos[1], num=iter_num)
        zArray = np.linspace(self.currentPos[2], self.goalPos[2], num=iter_num)
        aArray = np.linspace(self.currentOrientation[0], self.goalOrientation[0], num=iter_num) 
        bArray = np.linspace(self.currentOrientation[1], self.goalOrientation[1], num=iter_num)
        cArray = np.linspace(self.currentOrientation[2], self.goalOrientation[2], num=iter_num)

        trajectory = np.column_stack((xArray, yArray, zArray, aArray, bArray, cArray))

        if self.iter == 0:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(xArray, yArray, zArray, label='Trajectory')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            plt.show()
            print("Trajectory Generated")
            self.trajectory = trajectory
            self.moveFlag = True

        if self.goalPos[2] > 0.69:
            self.get_logger().info(f'Position out of reach')
            self.moveFlag = False


    def checkIfGoalReached(self):
        self.goalPos = np.array(self.goalPos, dtype=float)
        self.currentPos = np.array(self.currentPos, dtype=float)
        if np.linalg.norm(self.goalPos - self.currentPos) < 0.05:
            self.get_logger().info(f'Goal position reached: {self.currentPos}')
            self.moveFlag = False
        else:
            self.moveFlag = True

    def velPlanner(self, velGain):
        currentTraj = self.trajectory[self.iter]
        X_dot = (currentTraj[0] - self.currentPos[0]) * velGain
        Y_dot = (currentTraj[1] - self.currentPos[1]) * velGain
        Z_dot = (currentTraj[2] - self.currentPos[2]) * velGain
        return X_dot, Y_dot, Z_dot
    
    def forward_kinematics(self, joint_angles):
        #Calculate the transformation matrix
        T_matrix = T.subs({theta1:joint_angles[0], theta2:joint_angles[1], theta3:joint_angles[2], theta4:joint_angles[3], theta5:joint_angles[4], theta6:joint_angles[5]}).evalf()
        #Extract the position of the end effector
        X = T_matrix[0,3]
        Y = T_matrix[1,3]
        Z = T_matrix[2,3]
        OrientationMat = R.from_matrix(T_matrix[:3,:3])
        return np.array([X, Y, Z]), OrientationMat.as_rotvec()

                
    def inverse_kinematics(self):
        joint_positions = Float64MultiArray()
        q_matrix = sp.Matrix([self.jointAngles[0], self.jointAngles[1], self.jointAngles[2], self.jointAngles[3], self.jointAngles[4], self.jointAngles[5]])
        print(f'Initial Qmat: {q_matrix}')
        while(self.moveFlag == True):
            if self.iter < len(self.trajectory):
                print(f'************Iteration: {self.iter}******************')
                currentTraj = self.trajectory[self.iter]
                vel_gain = 1
                X_dot, Y_dot, Z_dot = self.velPlanner(vel_gain)
                #Position velocitiy Matrix
                V_matrix = sp.Matrix([X_dot, Y_dot, Z_dot, 0.0, 0.0, 0.0])
                #Update the jacobian with the new joint angles
                J_matrix = J_mat.subs({theta1:q_matrix[0], theta2:q_matrix[1], theta3:q_matrix[2], theta4:q_matrix[3], theta5:q_matrix[4], theta6:q_matrix[5]})
                #Calculate the pseudo inverse of the jacobian
                J_inv = J_matrix.pinv()
                #Damping the Jacobian
                damping_factor = 0.01
                w2 = (damping_factor**2)*np.eye(J_matrix.shape[0])
                J_matrix = np.array(J_matrix, dtype=np.float64)
                J_inv_damped = np.dot(J_matrix.T, np.linalg.inv(np.dot(J_matrix,J_matrix.T)+w2))
                #Calculate the joint velocities
                q_dot = J_inv_damped @ V_matrix
                #Update the joint angles
                q_matrix = q_matrix + (q_dot)
                self.jointAngles = q_matrix
                #Publish the joint angles
                q_matrix_array = [float(q) for q in q_matrix]
                joint_positions.data = [q_matrix_array[0], q_matrix_array[1], q_matrix_array[2], q_matrix_array[3], q_matrix_array[4], q_matrix_array[5], 0.0]
                self.joint_position_pub.publish(joint_positions)
                #Update the current position of the end effector
                self.currentPos, self.currentOrientation = self.forward_kinematics(joint_positions.data)
                self.get_logger().info(f'Current Position: {self.currentPos}')
                self.get_logger().info(f'Current Orientation: {self.currentOrientation}')
                #Check if the goal position is reached
                self.checkIfGoalReached()
                #Next iteration
                self.iter += 1
            else:
                self.moveFlag = False
        #Stop the robot
        q_matrix = [float(q) for q in q_matrix]
        joint_positions.data = [q_matrix[0], q_matrix[1], q_matrix[2], q_matrix[3], q_matrix[4], q_matrix[5], 0.5]
        self.joint_position_pub.publish(joint_positions)
        self.get_logger().info(f'Joint Angles: {joint_positions.data}')
        self.get_logger().info(f'Goal position reached: {self.currentPos}')
        self.get_logger().info(f'Vehicle Charging')

def main(args=None):
    rclpy.init(args=args)
    node = jointPosPubNode()
    node.inverse_kinematics()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()