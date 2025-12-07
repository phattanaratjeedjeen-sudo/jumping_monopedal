#!/usr/bin/python3

import math
import os
from ament_index_python.packages import get_package_share_directory
from monoped_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import Altimeter
from monoped_interfaces.msg import DebugDeadbeat
import xml.etree.ElementTree as ET
from math import sqrt
import numpy as np

class DeadBeatController(Node):
    def __init__(self):
        super().__init__('deadbeat_controller')
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Altimeter, '/altimeter_data', self.altimeter_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.debug_state_publisher = self.create_publisher(DebugDeadbeat, '/debug_deadbeat', 10)
        self.create_timer(0.01, self.timer_callback)
        
        tree_path = os.path.join(get_package_share_directory('monoped_description'), "urdf", "robot_params.xacro")
        tree = ET.parse(tree_path)
        xacro_ns = {'xacro': 'http://www.ros.org/wiki/xacro'}
        root = tree.getroot()

        self.mass = []
        mass_properties = ['mass_body', 'mass_foot']
        for properties in mass_properties:
            xpath_query = f"xacro:property[@name='{properties}']"
            property_element = root.find(xpath_query, xacro_ns)
            value = float(property_element.get('value'))
            self.mass.append(value)

        self.l0 = 0.38         # leg length (m) (0.125 + 0.1 + 0.1 + 0.05 + 0.005) base from urdf
        self.zf = 0.0          # foot height (m)
        self.zb = 0.0          # body height (m)
        self.zb_dot = 0.0      # body velocity (m/s)
        self.Hk = 0.6          # actual height of current hop (m)
        self.Hc = 0.0          # value given to controller as apex height of current hop (m)
        self.Hd = 0.6          # desired height to reach at apex of next hop (m)
        self.Ls = self.l0
        self.g = 9.81          # gravity acc (m/s^2)
        self.ks = 1000         # spring stiffness (N/m)
        self.spring_com = 0.0  # spring compressed (m)
        self.altimeter = [0, 0, 0]     # altimeater sensor data [position, vel, ref]
        
        self.state = 'air'     # initial state
        self.air_state_timer = None
        self.air_delay = 0.01
        self.zb_dot_prev = 0.0
        self.u = 0.0           # active spring energy (J)
        self.force = 0.0       # leg force command (N)
        self.hop_count = 0     # hop counter


    def publish_debug_state(self):
        state_code = {'air': 0, 'compress': 1, 'touchdown': 2, 'rebound': 3}.get(self.state, -1)
        msg = DebugDeadbeat()
        msg.zb = self.zb
        msg.zb_dot = self.zb_dot
        msg.zf = max(0.0, self.zf)
        msg.hk = self.Hk
        msg.hc = self.Hc
        msg.hd = self.Hd
        msg.u = self.u
        msg.force = self.force
        msg.state_code = state_code
        msg.hop = self.hop_count
        self.debug_state_publisher.publish(msg)


    def pub_effort(self):
        msg = Float64MultiArray()
        msg.data = [self.force]
        self.effort_publisher.publish(msg)
        self.publish_debug_state()


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
                if self.air_state_timer is None:
                    self.air_state_timer = self.create_timer(
                        self.air_delay,
                        self.air_to_compress_callback
                    )
        elif self.state == 'compress' and self.zf <= 0:
            self.state = 'touchdown'
        elif self.state == 'touchdown' and self.zb_dot >= 0:
            self.state = 'rebound'
            self.force_compute()
        elif self.state == 'rebound' and self.zf > 0:
            self.state = 'air'
            self.hop_count += 1


    def air_to_compress_callback(self):
        if self.state == 'air':
            self.state = 'compress'
            self.force_compute()
            if self.air_state_timer is not None:
                self.destroy_timer(self.air_state_timer)
                self.air_state_timer = None


    def joint_states_callback(self, msg: JointState):
        self.zf = self.zb - 0.125 - 0.05 -0.005 - (0.1 - msg.position[0]) - (0.1 - msg.position[1])
        self.publish_debug_state()


    def timer_callback(self):
        self.state_manager()
        self.pub_effort()


    def force_compute(self):
        if self.state == 'compress':
            self.Hc = self.Hk
            El = self.mass[1] * self.g * (self.Hc - self.Ls) 
            Et = np.sum(self.mass) * self.g * (self.Hd - self.Ls) * (self.mass[1] / self.mass[0])
            Eh = np.sum(self.mass) * self.g *(self.Hd - self.Hc) 
            self.u = El + Et + Eh
            if self.u >= 0.0:
                self.spring_com = math.sqrt(2.0 * self.u / self.ks)
                self.force = self.spring_com * self.ks
        elif self.state == 'rebound':
            self.force = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DeadBeatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
