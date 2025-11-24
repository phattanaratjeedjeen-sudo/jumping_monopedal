#!/usr/bin/python3
"""
Reaction Wheel Stabilization Controller for 2D Monoped Robot

Uses two reaction wheels to stabilize the robot's pitch angle.
PID controller on body pitch with reaction wheel *torque/effort* as output.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.msg import Altimeter
import numpy as np
import math


class ReactionWheelStabilizer(Node):
    def __init__(self):
        super().__init__('rw_stabilizer')

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.create_subscription(Altimeter, '/altimeter_data', self.altimeter_callback, 10)

        # Publisher - single effort controller for all joints
        # Order: [thigh_to_shank, body_to_rw_left, body_to_rw_right]
        self.effort_publisher = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10
        )

        # Debug publisher
        self.debug_publisher = self.create_publisher(
            Float64MultiArray, '/rw_debug', 10
        )

        # PID Controller Gains for pitch stabilization (EFFORT/TORQUE output)
        self.declare_parameter('Kp', 10.0)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('Kd', 2.0)
        self.declare_parameter('desired_pitch', 0.0)
        self.declare_parameter('max_torque', 20.0)  # N.m
        self.declare_parameter('dead_zone', 0.005)
        self.declare_parameter('control_rate', 100.0)  # Hz - match controller_manager update_rate

        # Simple stance control for prismatic joint
        self.declare_parameter('spring_desired_pos', 0.0)
        self.declare_parameter('spring_Kp', 200.0)
        self.declare_parameter('spring_Kd', 5.0)
        self.declare_parameter('spring_effort_limit', 200.0)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.desired_pitch = self.get_parameter('desired_pitch').value
        self.max_torque = self.get_parameter('max_torque').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.control_rate = self.get_parameter('control_rate').value
        self.spring_desired_pos = self.get_parameter('spring_desired_pos').value
        self.spring_Kp = self.get_parameter('spring_Kp').value
        self.spring_Kd = self.get_parameter('spring_Kd').value
        self.spring_effort_limit = self.get_parameter('spring_effort_limit').value

        # IMU state
        self.imu_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.imu_angular_velocity = [0.0, 0.0, 0.0]

        # State variables
        self.pitch = 0.0
        self.pitch_error = 0.0
        self.sum_error = 0.0
        self.prev_error = 0.0
        self.body_height = 0.0
        self.spring_pos = 0.0
        self.spring_vel = 0.0

        # Controller enable flag
        self.is_enabled = False

        # Time step
        self.dt = 1.0 / self.control_rate

        # Fixed-rate control loop
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Reaction Wheel Stabilizer (EFFORT/TORQUE Control) Started')
        self.get_logger().info(f'Gains: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, max_torque={self.max_torque}')

    def quaternion_to_euler(self, q):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]

    def imu_callback(self, msg: Imu):
        if not self.is_enabled:
            self.get_logger().info('First IMU data received - controller active!')
            self.is_enabled = True
            self.sum_error = 0.0
            self.prev_error = 0.0
        self.imu_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.imu_angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def altimeter_callback(self, msg: Altimeter):
        self.body_height = msg.vertical_reference - (-msg.vertical_position)

    def joint_states_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'thigh_to_shank':
                self.spring_pos = msg.position[i]
                if msg.velocity:
                    self.spring_vel = msg.velocity[i]

    def control_loop(self):
        if not self.is_enabled:
            return

        euler = self.quaternion_to_euler(self.imu_orientation)
        self.pitch = euler[1]

        self.pitch_error = self.desired_pitch - self.pitch

        if abs(self.pitch_error) < self.dead_zone:
            self.sum_error = 0.0

        # PID terms for TORQUE output
        P = self.Kp * self.pitch_error

        self.sum_error += self.pitch_error * self.dt
        max_integral = self.max_torque / (abs(self.Ki) + 1e-6)
        self.sum_error = max(-max_integral, min(max_integral, self.sum_error))
        I = self.Ki * self.sum_error

        # D term from IMU angular velocity (smoother than derivative of error)
        # Wheels spin around Z axis (after frame rotation, this controls pitch)
        pitch_rate = self.imu_angular_velocity[1]  # Y-axis is pitch rate in body frame
        D = -self.Kd * pitch_rate
        self.prev_error = self.pitch_error

        # Total control output is torque
        tau_rw_unsat = P + I + D

        # Saturate torque
        tau_rw = np.clip(tau_rw_unsat, -self.max_torque, self.max_torque)

        # Anti-windup (back-calculation method)
        if abs(tau_rw_unsat - tau_rw) > 0.01 and abs(self.Ki) > 1e-6:
            self.sum_error -= (tau_rw_unsat - tau_rw) / self.Ki

        # Apply torque to both wheels
        # Both wheels have axis Z in rotated joint frame:
        #   Left wheel: joint Z = parent +Y (after -90° X rotation)
        #   Right wheel: joint Z = parent -Y (after +90° X rotation)
        # Since axes point opposite directions, apply opposite-signed torques
        # for same body reaction:
        tau_left = -tau_rw
        tau_right = tau_rw  # Opposite sign because joint Z points opposite direction

        # Spring joint control
        spring_error = self.spring_desired_pos - self.spring_pos
        tau_spring = self.spring_Kp * spring_error - self.spring_Kd * self.spring_vel
        tau_spring = np.clip(tau_spring, -self.spring_effort_limit, self.spring_effort_limit)

        # Publish efforts: [thigh_to_shank, body_to_rw_left, body_to_rw_right]
        effort_msg = Float64MultiArray()
        effort_msg.data = [tau_spring, tau_left, tau_right]
        self.effort_publisher.publish(effort_msg)

        # Publish debug info
        debug_msg = Float64MultiArray()
        debug_msg.data = [
            self.pitch,       # 0: pitch angle (rad)
            pitch_rate,       # 1: pitch rate from IMU (rad/s)
            tau_rw,           # 2: commanded rw torque (N.m)
            self.spring_pos,  # 3: spring position
            tau_spring,       # 4: commanded spring effort
            P, I, D           # 5, 6, 7: PID terms
        ]
        self.debug_publisher.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactionWheelStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
