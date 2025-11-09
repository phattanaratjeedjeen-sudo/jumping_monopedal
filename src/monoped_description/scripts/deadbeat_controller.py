#!/usr/bin/python3

from monoped_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
import random
from math import sqrt


class DeadBeatController(Node):
    def __init__(self):
        super().__init__('deadbeat_controller')
        
        self.get_logger().info('DeadBeatController node has been started.')
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.position_publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.q = np.array([0.0, 0.0])  # [body_height, leg_extension]

        self.l0 = 0.23 # 
        self.mass_body = 1.0   # kg
        self.mass_foot = 0.08  # kg
        self.zf = 0 # foot height (m)
        self.zb = 0 # body height (m)
        self.Hk = 0 # actual height of current hop (m)
        self.Hc = 0 # value given to controller as apex height of current hop (m)
        self.Hd = 0.5  # desired height to reach at apex of next hop (m)
        self.Ls = 0
        self.g = 9.81  # gravity acc (m/s^2)
        self.ks = 100  # stiffness (N/m)

        self.state = 'air' # initial state
        self.air_state_timer = None
        self.air_delay = 0.1

        self.Ts = 0.01
        self.t = 0

        self.create_timer(self.Ts, self.timer_callback)

    def state_manager(self):
        if self.state == 'air':
            if self.air_state_timer is None:
                self.air_state_timer = self.create_timer(self.air_delay, self.air_to_compress_callback)

        elif self.state == 'compress' and self.zf <= 0:
            self.state = 'touchdown'
            self.get_logger().info('touchdown')

        elif self.state == 'touchdown' and self.zb-self.zf > self.l0:
            self.state = 'rebound'
            # shut off prismatic
            self.command_pub()
            self.get_logger().info('rebound')
        
        elif self.state == 'rebound' and self.zf >= 0:
            self.state = 'air'
            self.get_logger().info('air')

    def air_to_compress_callback(self):
        if self.state == 'air':
            self.state = 'compress'
            self.command_pub()
            self.get_logger().info('compress')

            if self.air_state_timer is not None:
                self.air_state_timer.cancel()
                self.destroy_timer(self.air_state_timer)
                self.air_state_timer = None

    def joint_state_callback(self, msg: JointState):
        self.q = np.array(msg.position)
        

    def tf_callback(self, msg: TFMessage):
        self.zb = msg.transforms[1].transform.translation.z
        self.get_logger().info(f'Body height: {self.zb}')

    def timer_callback(self):
        self.state_manager()
        # self.get_logger().info(f'Published position commands: {command_msg.data}')
        pass

    def command_pub(self):

        msg = Float64MultiArray()
        desired_compression = 0.0
        
        if self.state == 'compress':
            self.Hc = random.uniform(0.1, 0.3)
            El = self.mass_foot * self.g * (self.Hc - self.Ls)
            Et = (self.mass_body + self.mass_foot) * self.g * (self.Hd - self.Ls) * (self.mass_foot / self.mass_body)
            Eh = (self.mass_body + self.mass_foot) * self.g *(self.Hd - self.Hc)
            u = El + Et + Eh
            desired_compression = sqrt((2 * u) / self.ks)

            msg = Float64MultiArray()
            msg.data = [desired_compression]  # Example command values
        else:
            pass

        msg.data = [desired_compression]  # Example command values
        self.position_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DeadBeatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
