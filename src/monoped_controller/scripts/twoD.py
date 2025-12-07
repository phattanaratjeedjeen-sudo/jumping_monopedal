#!/usr/bin/python3

import math
from ament_index_python.packages import get_package_share_directory
from monoped_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import Altimeter
from monoped_interfaces.msg import Debug2D
import xml.etree.ElementTree as ET
import numpy as np
from tf_transformations import euler_from_quaternion

class TwoDController(Node):
    def __init__(self):
        super().__init__('TwoD_controller')
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Altimeter, '/altimeter_data', self.altimeter_callback, 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.debug_state_publisher = self.create_publisher(Debug2D, '/debug_2d', 10)
        self.create_timer(0.01, self.timer_callback)

        # Mass parameters (kg)
        self.mass_body = 0.5
        self.mass_foot = 0.1
        self.mass_rw = 0.1*2
        
        self.l0 = 0.30         # spring equilibrium (m) base from urdf
        self.zf = 0.0          # foot height (m)
        self.zb = 0.0          # body height (m)
        self.zb_dot = 0.0      # body velocity (m/s)
        self.Hk = 0.7          # actual height of current hop (m)
        self.Hc = 0.0          # value given to controller as apex height of current hop (m)
        self.Hd = 0.7          # desired height to reach at apex of next hop (m)
        self.Ls = self.l0      
        self.g = 9.81          # gravity acc (m/s^2)
        self.ks = 1000         # spring stiffness (N/m)
        self.spring_com = 0.0  # spring compressed (m)
        self.altimeter = [0, 0, 0]     # altimeater sensor data [position, vel, ref]
        
        self.state = 'air'     # initial state
        self.air_state_timer = None
        self.air_delay = 0.01
        self.zb_dot_prev = 0.0
        self.u = 0.0             # active spring energy (J)
        self.force = 0.0         # leg force command (N)
        self.Kg = [17.94, 1.26]  # feedback gains for ground phase
        self.Kf = [15.82, 0.92]  # feedback gains for flight phase
        self.Ng = self.Kg[0]     # compensate gain for ground phase
        self.Nf = self.Kf[0]     # compensate gain for flight phase
        
        self.theta_desired = 90.0   # desired body pitch angle (deg)  
        self.theta = 90.0           # body pitch angle (deg)
        self.theta_dot = 0.0        # body pitch angular velocity (deg/s)
        self.torque = 0.0           # torque command (Nm)
        self.tilt = 0.0             # tilt angle (deg)
        self.hop_count = 0          # hop counter
        self.hop_toggle = True   


    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.theta = 90 - math.degrees(pitch)
        omega_y_rad = msg.angular_velocity.y
        self.theta_dot = -math.degrees(omega_y_rad)
        

    def publish_debug_state(self):
        msg = Debug2D()
        msg.zb = self.zb
        msg.zf = self.zf
        msg.force = self.force
        msg.theta = self.theta
        msg.theta_dot = self.theta_dot
        msg.torque = self.torque 
        msg.u = self.u
        msg.hop = self.hop_count   
        self.debug_state_publisher.publish(msg)


    def altimeter_callback(self, msg: Altimeter):
        self.altimeter = np.array([msg.vertical_position, msg.vertical_velocity, msg.vertical_reference])
        self.zb = self.altimeter[2] - (-self.altimeter[0])
        self.zb_dot_prev = self.zb_dot
        self.zb_dot = self.altimeter[1]
        self.publish_debug_state()


    def state_manager(self):
        if self.state == 'air':
            # detect apex: previous vel > 0 and current vel <= 0
            if self.zb_dot_prev > 0.0 and self.zb_dot <= 0.0:
                self.Hk = self.zb  # apex hop height
                self.publish_debug_state()
                if self.air_state_timer is None:
                    self.air_state_timer = self.create_timer(
                        self.air_delay,
                        self.air_to_compress_callback
                    )
        elif self.state == 'compress' and self.zf <= 0.0:
            self.state = 'touchdown'
        elif self.state == 'touchdown' and self.zb_dot > 0:
            self.state = 'rebound'
            self.force_compute()
        elif self.state == 'rebound' and self.zf > 0:
            self.state = 'air'
            self.hop_count += 1
            if self.hop_count % 10 == 0 and self.hop_toggle:
                self.tilt = 5.0 if self.tilt == 0.0 else self.tilt * -1
    

    def air_to_compress_callback(self):
        if self.state == 'air':
            self.state = 'compress'
            self.force_compute()
            if self.air_state_timer is not None:
                self.destroy_timer(self.air_state_timer)
                self.air_state_timer = None


    def joint_states_callback(self, msg: JointState):
        self.zf = self.zb - 0.075 - 0.015 - 0.15 - 0.15 - 0.15 + msg.position[0] + msg.position[1]
        self.publish_debug_state()


    def timer_callback(self):
        self.state_manager()
        if self.state == 'compress' or self.state == 'air':
            self.torque_compute(self.Nf, self.theta_desired, self.Kf)
        elif self.state == 'touchdown' or self.state == 'rebound':
            self.torque_compute(self.Ng, self.theta_desired + self.tilt, self.Kg)
        self.pub_effort(self.force, self.torque)
        # self.get_logger().info(f'State: {self.state}, zb: {self.zb:.2f}, Zb_dot: {self.zb_dot:.2f}, zf: {self.zf:.2f}, T: {self.torque:.2f}, theta: {self.theta:.2f}, theta_dot = {self.theta_dot:.2f}' ) 

    def force_compute(self):
        if self.state == 'compress':
            self.Hc = self.Hk
            El = self.mass_foot * self.g * (self.Hc - self.Ls) 
            Et = (self.mass_body + self.mass_rw + self.mass_foot) * self.g * (self.Hd - self.Ls) * (self.mass_foot / (self.mass_body + self.mass_rw))
            Eh = (self.mass_body + self.mass_rw + self.mass_foot) * self.g *(self.Hd - self.Hc) 
            self.u = El + Et + Eh
            if self.u > 0.0:
                self.spring_com = math.sqrt(2.0 * self.u / self.ks)
                self.force = self.spring_com * self.ks
            self.pub_effort(self.force, self.torque)
        if self.state == 'rebound':
            self.force = 0.0
            self.pub_effort(self.force, self.torque)
    

    def torque_compute(self, Nbar, theta_desired, K):
        theta_desired_rad = math.radians(theta_desired)
        theta_dot_rad = math.radians(self.theta_dot)
        theta_rad = math.radians(self.theta)
        self.torque = theta_desired_rad * Nbar - (K[0] * theta_rad + K[1] * theta_dot_rad)


    def pub_effort(self, force, torque):
        msg = Float64MultiArray()
        msg.data = [force, torque*0.5, -torque*0.5] 
        self.effort_publisher.publish(msg)
        self.publish_debug_state() 
        

def main(args=None):
    rclpy.init(args=args)
    node = TwoDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
