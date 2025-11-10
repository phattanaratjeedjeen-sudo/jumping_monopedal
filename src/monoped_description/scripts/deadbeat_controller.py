#!/usr/bin/python3

from monoped_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
import numpy as np
import random
from math import sqrt


class DeadBeatController(Node):
    def __init__(self):
        super().__init__('deadbeat_controller')
        
        self.get_logger().info('DeadBeatController node has been started.')
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.position_publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.q = np.array([0.0, 0.0])  # [body_height, leg_extension]

        self.l0 = 0.225 # spring equilibrium (m)
        self.mass_body = 1.0   # kg
        self.mass_foot = 0.08  # kg
        self.zf = 0 # foot height (m)
        self.zb = 0 # body height (m)
        self.zb_o = 0.5
        self.zb_dot = 0
        self.Hk = 0.5 # actual height of current hop (m)
        self.Hc = 0.5 # value given to controller as apex height of current hop (m)
        self.Hd = 0.5  # desired height to reach at apex of next hop (m)
        self.Ls = 0
        self.g = 9.81  # gravity acc (m/s^2)
        self.ks = 1000  # spring stiffness (N/m)

        self.b2ul = 0.125
        self.ul2ll = 0
        self.ll2f = 0.05
        self.f2fc = 0.005

        self.state = 'air' # initial state
        self.air_state_timer = None
        self.air_delay = 0.1

        self.Ts = 0.05

        self.create_timer(self.Ts, self.timer_callback)

    def state_manager(self):
        if self.state == 'air':
            if self.air_state_timer is None:
                self.air_state_timer = self.create_timer(self.air_delay, self.air_to_compress_callback)

        elif self.state == 'compress' and self.zf <= 0:
            self.state = 'touchdown'
            # self.get_logger().info('touchdown')

        elif self.state == 'touchdown' and self.zb > self.l0 and self.zb_dot > 0:
            self.state = 'rebound'
            # joint shut down
            self.command_pub()
            # self.get_logger().info('rebound')
        
        elif self.state == 'rebound' and self.zf > 0:
            self.state = 'air'
            # self.get_logger().info('air')

    def air_to_compress_callback(self):
        if self.state == 'air':
            self.state = 'compress'
            self.command_pub()
            # self.get_logger().info('compress')

            if self.air_state_timer is not None:
                self.air_state_timer.cancel()
                self.destroy_timer(self.air_state_timer)
                self.air_state_timer = None

    def tf_callback(self, msg: TFMessage):
        self.zb = msg.transforms[1].transform.translation.z
        self.ul2ll = -msg.transforms[0].transform.translation.z
        self.zf = self.zb-(self.b2ul + self.ul2ll + self.ll2f + self.f2fc)
        self.zb_dot = (self.zb - self.zb_o)/(1/19)
        self.zb_o = self.zb
        self.get_logger().info(f'Body height: {self.zb}, Foot height: {self.zf}')

    def timer_callback(self):
        self.state_manager()
        self.get_logger().info(f'{self.state}')


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
            self.get_logger().info(f"command: {desired_compression}")

            msg = Float64MultiArray()
            msg.data = [desired_compression]  
        else:
            # stop controller (free joint)

            pass

        msg.data = [desired_compression]
        self.position_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DeadBeatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
