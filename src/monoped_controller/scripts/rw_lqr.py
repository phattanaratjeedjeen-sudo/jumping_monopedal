#!/usr/bin/python3
"""
LQR Reaction Wheel Controller for 2D Monoped Robot

Controls body pitch using two reaction wheels with LQR full-state feedback.
State: x = [theta, theta_dot, q_L, qdot_L, q_R, qdot_R]
Input: u = [tau_L, tau_R] (torque on left and right reaction wheels)

The controller also maintains spring compression to keep the foot on the ground.
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class LQRReactionWheelController(Node):
    def __init__(self):
        super().__init__('lqr_reaction_wheel_controller')

        # LQR gain matrix K (2x6)
        # State: x = [theta, theta_dot, q_L, qdot_L, q_R, qdot_R]
        # Control: u = -K*x = [tau_L, tau_R]
        # These gains are PLACEHOLDERS and need proper LQR design!
        k11 = 15.0;  k12 = 5.0;  k13 = 10.0;  k14 = 1.0;  k15 = 10.0;  k16 = 1.0
        k21 = 15.0;  k22 = 5.0;  k23 = 10.0;  k24 = 1.0;  k25 = 10.0;  k26 = 1.0
        self.K = np.array([
            [k11, k12, k13, k14, k15, k16],  # -> u_L
            [k21, k22, k23, k24, k25, k26],  # -> u_R
        ], dtype=float)

        self.theta = 0.0
        self.theta_dot = 0.0
        self.q_L = 0.0
        self.qdot_L = 0.0
        self.q_R = 0.0
        self.qdot_R = 0.0
        
        # Spring joint state
        self.spring_pos = 0.0
        self.spring_vel = 0.0

        self.left_joint_name = 'body_to_rw_left'
        self.right_joint_name = 'body_to_rw_right'
        self.spring_joint_name = 'thigh_to_shank'

        self.have_imu = False
        self.have_js = False

        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.effort_pub = self.create_publisher(Float64MultiArray,
                                                '/effort_controller/commands', 10)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)


    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.theta = pitch
        self.theta_dot = msg.angular_velocity.y
        self.have_imu = True

    def joint_callback(self, msg: JointState):
        names = msg.name
        pos = msg.position
        vel = msg.velocity

        try:
            iL = names.index(self.left_joint_name)
            iR = names.index(self.right_joint_name)
            iS = names.index(self.spring_joint_name)

            self.q_L = pos[iL]
            self.qdot_L = vel[iL]
            self.q_R = pos[iR]
            self.qdot_R = vel[iR]
            self.spring_pos = pos[iS]
            self.spring_vel = vel[iS]

            self.have_js = True
        except ValueError:
            pass

    def control_loop(self):
        if not (self.have_imu and self.have_js):
            return

        # State vector
        x = np.array([[self.theta],
                      [self.theta_dot],
                      [self.q_L],
                      [self.qdot_L],
                      [self.q_R],
                      [self.qdot_R]])

        # LQR control law: u = -K*x
        u_vec = - self.K @ x   # shape (2,1)
        u_L = float(u_vec[0, 0])
        u_R = float(u_vec[1, 0])

        # Saturate torque commands
        u_L = max(min(u_L, 5.0), -5.0)
        u_R = max(min(u_R, 5.0), -5.0)

        cmd = Float64MultiArray()
        # Order must match controller_full.yaml: [body_to_rw_left, body_to_rw_right]
        cmd.data = [u_L, u_R]  
        self.effort_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LQRReactionWheelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
