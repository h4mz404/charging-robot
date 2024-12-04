#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
LIN_VEL_STEP_SIZE = 5.0
ANG_VEL_STEP_SIZE = 0.5

MAX_VEL = 100.0

# joint1_vel = 0.0
# joint2_vel = 0.0
# joint3_vel = 0.0
# joint4_vel = 0.0
# joint5_vel = 0.0
# joint6_vel = 0.0
# joint7_vel = 0.0

# joint1_pos = 0.0
# joint2_pos = 0.0
# joint3_pos = 0.0
# joint4_pos = 0.0
# joint5_pos = 0.0
# joint6_pos = 0.0
# joint7_pos = 0.0

jointVel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
jointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        
        self.joint_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

        # jointVel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # jointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):

        global jointVel
        global jointPos

        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d
        Joint 1: a/d
        Joint 2: w/s
        Joint 3: i/k
        Joint 4: j/l
        Joint 5: z/x
        Joint 6: c/v
        Joint 7: b/n

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        joint_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    jointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    # jointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                elif key == 'a':  # Joint 1
                    jointPos[0] += 0.1
                elif key == 'd':
                    jointPos[0] -= 0.1
                elif key == 'w':  # Joint 2
                    jointPos[1] += 0.1
                elif key == 's':
                    jointPos[1] -= 0.1
                elif key == 'i':  # Joint 3
                    jointPos[2] += 0.1
                elif key == 'k':
                    jointPos[2] -= 0.1
                elif key == 'j':  # Joint 4
                    jointPos[3] += 0.1
                elif key == 'l':
                    jointPos[3] -= 0.1
                elif key == 'z':  # Joint 5
                    jointPos[4] += 0.1
                elif key == 'x':
                    jointPos[4] -= 0.1
                elif key == 'c':  # Joint 6
                    jointPos[5] += 0.1
                elif key == 'v':
                    jointPos[5] -= 0.1
                elif key == 'b':  # Joint 7
                    jointPos[6] += 0.01
                elif key == 'n':
                    jointPos[6] -= 0.01


                velLimit = 0.6  # Define the velocity limit

                # Constrain joint velocities to be within the limit
                jointVel = [max(min(vel, velLimit), -velLimit) for vel in jointVel]
                

                print("Joint Velocities: ", jointVel[0], jointVel[1], jointVel[2], jointVel[3], jointVel[4], jointVel[5], jointVel[6])
                print("Joint Positions: ", jointPos[0], jointPos[1], jointPos[2], jointPos[3], jointPos[4], jointPos[5], jointPos[6])

                joint_velocities.data = [jointVel[0], jointVel[1], jointVel[2], jointVel[3], jointVel[4], jointVel[5], jointVel[6]]
                joint_positions.data = [jointPos[0], jointPos[1], jointPos[2], jointPos[3], jointPos[4], jointPos[5], jointPos[6]]
    

                # wheel_velocities.data = [linear_vel, linear_vel, -linear_vel]
                # joint_positions.data = [steer_angle,steer_angle]

                self.joint_position_pub.publish(joint_positions)
                
                self.joint_velocities_pub.publish(joint_velocities)
                

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()