#!/usr/bin/python3

import math
import os
from ament_index_python.packages import get_package_share_directory
from monoped_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import Altimeter
import xml.etree.ElementTree as ET
from math import sqrt
import numpy as np
import random
from tf_transformations import euler_from_quaternion

class TwoDController(Node):
    def __init__(self):
        super().__init__('TwoD_controller')
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Altimeter, '/altimeter_data', self.altimeter_callback, 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        # Single debug topic carrying combined state for easier plotting/logging
        self.debug_state_publisher = self.create_publisher(Float64MultiArray, '/debug_state', 10)

        # Mass parameters [kg]
        self.mass_body = 0.5
        self.mass_foot = 0.1
        self.mass_rw = 0.25*2
        
        self.l0 = 0.30         # spring equilibrium (m) base from urdf
        self.zf = 0.0          # foot height (m)
        self.zb = 0.0          # body height (m)
        self.zb_dot = 0.0      # body velocity (m/s)
        self.Hk = None         # actual height of current hop (m)
        self.Hc = 0.0          # value given to controller as apex height of current hop (m)
        self.Hd = 1.5          # desired height to reach at apex of next hop (m)
        self.Ls = self.l0
        self.g = 9.81          # gravity acc (m/s^2)
        self.ks = 1000         # spring stiffness (N/m)
        self.desired_compression = 0.0 # spring compressed (m)
        self.altimeter = [0, 0, 0]     # altimeater sensor data [position, vel, ref]
        self.state = 'air' # initial state
        self.air_state_timer = None
        self.air_delay = 0.01
        self.measurement_noise_std = 0.01  # e.g. 1 cm noise
        self.zb_dot_prev = 0.0
        self.u = 0.0
        self.effort_command = 0.0

        self.Kg = [11.5792, 1.028]  
        self.Kf = [56.2725/2, 2.25/2]  
        self.Nf = 56.2725/2
        self.Ng = 11.6     
        
        # Torque saturation and rate limiting parameters
        self.max_torque = 5.0  # maximum torque limit (Nm) - adjust based on your motor specs
        self.max_torque_rate = 50.0  # max torque change per second (Nm/s)
        self.prev_torque = 0.0  # previous torque command for rate limiting
        
        # Sensor clamping limits to reject impossible values
        self.max_theta_dot = 500.0  # max believable angular velocity (deg/s)
        self.max_zb_dot = 20.0  # max believable vertical velocity (m/s)
        
        self.theta_desired = 90.0   # desired body pitch angle (deg)  
        self.theta = 90.0        
        self.theta_dot = 0.0
        self.torque = 0.0           # torque command (Nm)
        # # Simple guardrail for debugging IMU spikes
        # self.theta_dot_warn_deg = 30.0  # deg/s
        # # Thresholds for logging other sensor anomalies
        # self.zb_dot_warn = 10.0  # m/s
        # self.zf_warn = 1.5       # m

        self.create_timer(0.01, self.timer_callback)


    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.theta = 90 - pitch * 180.0 / math.pi
        omega_y_rad = msg.angular_velocity.y
        self.theta_dot = -omega_y_rad * 180.0 / math.pi
        
        # Clamp theta_dot to reject impossible values (prevents torque explosion)
        # if abs(raw_theta_dot) > self.max_theta_dot:
        #     self.get_logger().warn(
        #         f"IMU CLAMPED: raw theta_dot={raw_theta_dot:.2f} deg/s -> clamped to {np.sign(raw_theta_dot) * self.max_theta_dot:.2f} deg/s"
        #     )
        #     self.theta_dot = np.sign(raw_theta_dot) * self.max_theta_dot
        # else:
        #     self.theta_dot = raw_theta_dot
            
        # if abs(self.theta_dot) > self.theta_dot_warn_deg:
        #     self.get_logger().info(
        #         f"IMU spike? ang_vel.y={omega_y_rad:.3f} rad/s -> theta_dot={self.theta_dot:.2f} deg/s"
        #     )
        # else:
        #     self.get_logger().debug(
        #         f"IMU ok: ang_vel.y={omega_y_rad:.3f} rad/s -> theta_dot={self.theta_dot:.2f} deg/s"
        #     )
        # else:
        #     self.get_logger().info(
        #         f"IMU ang_vel.y={omega_y_rad:.3f} rad/s -> theta_dot={self.theta_dot:.2f} deg/s"
        #     )

    def publish_debug_state(self, effort=None):
        """
        Publish combined debug data on one topic to simplify logging/plotting.
        Data layout: [zb, zb_dot, zf, Hk, Hc, Hd, u, effort, state_code]
        state_code: air=0, compress=1, touchdown=2, rebound=3, unknown=-1
        """
        effort_val = effort if effort is not None else self.u
        msg = Float64MultiArray()
        msg.data = [
            self.zb,
            self.zf,
            effort_val,
            self.theta,
            self.theta_dot,
            self.torque
        ]
        self.debug_state_publisher.publish(msg)

    def altimeter_callback(self, msg: Altimeter):
        self.altimeter = np.array([msg.vertical_position, msg.vertical_velocity, msg.vertical_reference])
        self.zb = self.altimeter[2] - (-self.altimeter[0])
        self.zb_dot_prev = self.zb_dot
        self.zb_dot = self.altimeter[1]
        
        # # Clamp zb_dot to reject impossible values
        # if abs(raw_zb_dot) > self.max_zb_dot:
        #     self.get_logger().warn(
        #         f"Altimeter CLAMPED: raw zb_dot={raw_zb_dot:.2f} m/s -> clamped to {np.sign(raw_zb_dot) * self.max_zb_dot:.2f} m/s"
        #     )
        #     self.zb_dot = np.sign(raw_zb_dot) * self.max_zb_dot
        # else:
        #     self.zb_dot = raw_zb_dot
            
        # if abs(self.zb_dot) > self.zb_dot_warn:
        #     self.get_logger().info(
        #         f"Altimeter spike? zb_dot={self.zb_dot:.2f} m/s pos={self.zb:.2f}"
        #     )
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
        elif self.state == 'compress' and self.zf <= 0.00001:
            self.state = 'touchdown'
            self.command_pub()

        elif self.state == 'touchdown' and self.zb_dot > 0:
            self.state = 'rebound'
            self.command_pub()
        
        elif self.state == 'rebound' and self.zf > 0:
            self.state = 'air'
    

    def air_to_compress_callback(self):
        if self.state == 'air':
            self.state = 'compress'
            self.command_pub()

            if self.air_state_timer is not None:
                self.destroy_timer(self.air_state_timer)
                self.air_state_timer = None


    def joint_states_callback(self, msg: JointState):
        self.zf = self.zb - 0.075 - 0.17 - 0.15 - 0.15 + msg.position[0] + msg.position[1]
        # if abs(self.zf) > self.zf_warn:
        #     self.get_logger().info(
        #         f"Joint-derived foot height outlier? zf={self.zf:.2f} from joints {msg.position[0]:.3f}, {msg.position[1]:.3f}, zb={self.zb:.2f}"
        #     )
        self.publish_debug_state()


    def timer_callback(self):
        self.state_manager()
        if self.state == 'compress' or self.state == 'air':
            # self.theta_desired = 90.0
            self.torque_compute(self.Nf, self.theta_desired, self.Kf)
            self.pub_effort(self.effort_command, self.torque)
            # self.get_logger().info(f'Compress State: Effort={self.effort_command:.2f} N, Torque={self.torque:.2f} Nm')
        elif self.state == 'touchdown' or self.state == 'rebound':
            self.theta_desired = 90.0
            self.torque_compute(self.Ng, self.theta_desired, self.Kg)
            self.pub_effort(self.effort_command, self.torque)
            # self.get_logger().info(f'{self.state.capitalize()} State: Effort={self.effort_command:.2f} N, Torque={self.torque:.2f} Nm')
        self.get_logger().info(f'State: {self.state}, zb: {self.zb:.2f}, Zb_dot: {self.zb_dot:.2f}, zf: {self.zf:.2f}, T: {self.torque:.2f}, theta: {self.theta:.2f}, theta_dot = {self.theta_dot:.2f}' ) 
  
        


    def command_pub(self):
        if self.state == 'compress':
            if self.Hk is None:
                self.get_logger().info("Hk is None, skip compress command")
                return
            # Case1: Hc set to desired height
            # self.Hc = self.Hd
            # Case2: Hc with noise
            self.Hc = random.gauss(self.Hk, self.measurement_noise_std)
            El = self.mass_foot * self.g * (self.Hc - self.Ls) 
            Et = (self.mass_body + self.mass_rw + self.mass_foot) * self.g * (self.Hd - self.Ls) * (self.mass_foot / (self.mass_body + self.mass_rw))
            Eh = (self.mass_body + self.mass_rw + self.mass_foot) * self.g *(self.Hd - self.Hc) 
            self.u = El + Et + Eh

            if self.u <= 0.0:
                self.desired_compression = 0.0
                self.effort_command = 0.0
            else:
                self.desired_compression = math.sqrt(2.0 * self.u / self.ks)
                self.effort_command = self.desired_compression * self.ks

            self.pub_effort(self.effort_command, self.torque)
            
        if self.state == 'rebound':
            # become free joint
            self.effort_command = 0.0
            self.pub_effort(self.effort_command, self.torque)
            # self.publish_debug_state(0.0)
    

    def torque_compute(self, Nbar, theta_desired, K):
        theta_desired_rad = theta_desired * math.pi / 180.0
        theta_dot_rad = self.theta_dot * math.pi / 180.0
        theta_rad = self.theta * math.pi / 180.0

        self.torque = theta_desired_rad * Nbar - (K[0] * theta_rad + K[1] * theta_dot_rad)
        
        # Compute raw desired torque
        # raw_torque = theta_desired_rad * Nbar - (K[0] * theta_rad + K[1] * theta_dot_rad)
        
        # # Apply torque saturation (prevents physics explosion)
        # saturated_torque = np.clip(raw_torque, -self.max_torque, self.max_torque)
        
        # # Apply rate limiting to prevent sudden torque changes (smooths control)
        # dt = 0.01  # 10ms timer period
        # max_change = self.max_torque_rate * dt
        # torque_change = saturated_torque - self.prev_torque
        # limited_change = np.clip(torque_change, -max_change, max_change)
        
        # self.torque = self.prev_torque + limited_change
        # self.prev_torque = self.torque
        
        # # Log if torque was saturated
        # if abs(raw_torque) > self.max_torque:
        #     self.get_logger().warn(
        #         f"Torque SATURATED: raw={raw_torque:.2f} Nm -> saturated={self.torque:.2f} Nm"
        #     )

    def pub_effort(self, force, torque):
        msg = Float64MultiArray()
        msg.data = [force, torque*0.5, -torque*0.5]  # [thigh_to_shank, body_to_rw_left, body_to_rw_right]
        self.effort_publisher.publish(msg)
        self.publish_debug_state(force) 
        

def main(args=None):
    rclpy.init(args=args)
    node = TwoDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
