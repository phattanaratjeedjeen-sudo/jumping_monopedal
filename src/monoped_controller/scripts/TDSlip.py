#!/usr/bin/python3

from monoped_2d_description.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Altimeter
import numpy as np

class TDSlip(Node):
    def __init__(self):
        super().__init__('td_slip_node')
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Altimeter, '/altimeter_data', self.altimeter_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.create_timer(0.01, self.timer_callback)

        # eq 3 parameters
        self.La = None
        self.Va = None
        self.Ra = None
        self.kb = None
        self.w = None

        self.Torque = None
        self.kt = self.kb

        # eq 8 (control law) parameters
        self.V = None
        self.a0 = 1.383
        self.a1 = 1.0727
        self.a2 = 2.000
        self.a3 = -1.441
        self.a4 = 1.706
        self.a5 = -0.398

        self.Tfc = 0.074

        self.rho = 0.03
        self.h = 0.00425
        self.madd = 0.4

        self.altimeter = [0, 0, 0]     # altimeater sensor data [position, vel, ref]
        self.zf = 0.0                  # foot height (m)
    
    def altimeter_callback(self, msg: Altimeter):
        self.altimeter = np.array([msg.vertical_position, msg.vertical_velocity, msg.vertical_reference])
        self.zb = self.altimeter[2] - (-self.altimeter[0])
        self.zb_dot_prev = self.zb_dot
        self.zb_dot = self.altimeter[1]

    def state_manager(self):
        if self.zb <= 0.0: # stance phase
            self.V = self.a5*t * self.a
        else:  
            self.V = 3.0

    def joint_states_callback(self, msg: JointState):
        self.w = msg.velocity[2]  # reaction wheel angular velocity

    def timer_callback(self):
        self.state_manager()
    
    def command_pub(self):
        msg = Float64MultiArray()

        msg.data = [0.0]
        self.effort_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TDSlip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
