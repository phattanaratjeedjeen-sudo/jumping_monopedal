#!/usr/bin/python3

from monoped_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import Altimeter
import xml.etree.ElementTree as ET
from math import sqrt
import numpy as np
import random

class DeadBeatController(Node):
    def __init__(self):
        super().__init__('deadbeat_controller')
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Altimeter, '/altimeter_data', self.altimeter_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)

        tree = ET.parse("monoped_description/urdf/robot_params.xacro")
        xacro_ns = {'xacro': 'http://www.ros.org/wiki/xacro'}
        root = tree.getroot()

        self.mass = []
        mass_properties = ['mass_body', 'mass_foot']
        for properties in mass_properties:
            xpath_query = f"xacro:property[@name='{properties}']"
            property_element = root.find(xpath_query, xacro_ns)
            value = float(property_element.get('value'))
            self.mass.append(value)

        self.dimension = [] 
        dimension_properties = ['b2ul', 'ul2ll', 'll2f', 'f_r']
        for properties in dimension_properties:
            xpath_query = f"xacro:property[@name='{properties}']"
            property_element = root.find(xpath_query, xacro_ns)
            value = float(property_element.get('value'))
            self.dimension.append(value)


        self.l0 = 0.22         # spring equilibrium (m)
        self.zf = 0.0          # foot height (m)
        self.zb = 0.0          # body height (m)
        self.zb_dot = 0.0      
        self.Hk = 0.5          # actual height of current hop (m)
        self.Hc = 0.0          # value given to controller as apex height of current hop (m)
        self.Hd = 0.5          # desired height to reach at apex of next hop (m)
        self.Ls = 0            # unknown variable
        self.g = 9.81          # gravity acc (m/s^2)
        self.ks = 1000         # spring stiffness (N/m)
        self.desired_compression = 0.0 # spring compressed (m)
        self.altimeter = [0, 0, 0]     # altimeater sensor data [position, vel, ref]

        self.state = 'air' # initial state
        self.air_state_timer = None
        self.air_delay = 0.1

        self.create_timer(0.01, self.timer_callback)
        # self.get_logger().info(f'mass: {self.dimension}')


    def altimeter_callback(self, msg:Altimeter):
        self.altimeter = np.array([msg.vertical_position, msg.vertical_velocity, msg.vertical_reference])
        self.zb = self.altimeter[2] - (-self.altimeter[0])
        self.zb_dot = self.altimeter[1]
        # self.get_logger().info(f'altimeter: {self.altimeter}, zb: {self.zb}')


    def state_manager(self):
        
        if self.state == 'air':
            if self.zb_dot <= 0:
                self.Hk = self.zb # apex hop height
                if self.air_state_timer is None:
                    self.air_state_timer = self.create_timer(self.air_delay, self.air_to_compress_callback)

        elif self.state == 'compress' and self.zf <= 0:
            self.state = 'touchdown'

        elif self.state == 'touchdown' and self.zb_dot >= 0:
            self.state = 'rebound'
            self.command_pub()
        
        elif self.state == 'rebound' and self.zf > 0:
            self.state = 'air'
    

    def air_to_compress_callback(self):
        
        if self.state == 'air':
            self.state = 'compress'
            self.command_pub()

            if self.air_state_timer is not None:
                self.air_state_timer.cancel()
                self.destroy_timer(self.air_state_timer)
                self.air_state_timer = None


    def joint_states_callback(self, msg: JointState):
        # self.zf = self.zb - self.dimension[0] - (self.dimension[1] - msg.position[0]) - self.dimension[2] - 0.005
        self.zf = self.zb + sum(self.dimension[0:2]) + msg.position[0] - self.dimension[3]
        # self.get_logger().info(f"{self.zf}")


    def timer_callback(self):
        self.state_manager()
        # self.get_logger().info(f'{self.state}')


    def command_pub(self):
        msg = Float64MultiArray()
        
        if self.state == 'compress':
            self.Hc = random.gauss(0.01, self.Hk)
            El = self.mass[1] * self.g * (self.Hc - self.Ls) 
            Et = (self.mass[0] + self.mass[1]) * self.g * (self.Hd - self.Ls) * (self.mass[1] / self.mass[0])
            Eh = (self.mass[0] + self.mass[1]) * self.g *(self.Hd - self.Hc) 
            u = El + Et + Eh
            self.desired_compression = sqrt((2 * u) / self.ks)
            effort_command = self.desired_compression*self.ks

            msg.data = [effort_command]
            self.effort_publisher.publish(msg)
            # self.get_logger().info(f"compress: {self.desired_compression}, effort: {effort_command}")
            
        elif self.state == 'rebound':
            # become free joint
            msg.data = [0.0]
            self.effort_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeadBeatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
