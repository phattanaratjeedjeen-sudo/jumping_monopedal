#!/usr/bin/env python3

"""Publish an effort command that emulates a physical spring."""

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class SpringEffortController(Node):
    """Simple joint-space spring-damper running in ROS 2."""

    def __init__(self) -> None:
        super().__init__("spring_effort_controller")

        self.declare_parameter("joint_name", "body_to_foot")
        self.declare_parameter("spring_reference", 0.3)
        self.declare_parameter("spring_stiffness", 5000.0)
        self.declare_parameter("joint_damping", 20.0)

        self._joint_name: str = self.get_parameter("joint_name").get_parameter_value().string_value
        self._spring_reference: float = self.get_parameter("spring_reference").get_parameter_value().double_value
        self._spring_stiffness: float = self.get_parameter("spring_stiffness").get_parameter_value().double_value
        self._joint_damping: float = self.get_parameter("joint_damping").get_parameter_value().double_value

        self._latest_state: Dict[str, Dict[str, float]] = {}

        self._state_sub = self.create_subscription(
            JointState, "joint_states", self._state_callback, 10
        )
        self._effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controller/commands", 10
        )

        self._timer = self.create_timer(0.01, self._publish_command)

        self.get_logger().info(
            "Spring controller active for '%s' with k=%.1f, ref=%.3f m"
            % (self._joint_name, self._spring_stiffness, self._spring_reference)
        )

    def _state_callback(self, msg: JointState) -> None:
        idx_map = {name: idx for idx, name in enumerate(msg.name)}
        if self._joint_name not in idx_map:
            return
        idx = idx_map[self._joint_name]
        position = msg.position[idx] if idx < len(msg.position) else 0.0
        velocity = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
        self._latest_state[self._joint_name] = {
            "position": position,
            "velocity": velocity,
        }

    def _publish_command(self) -> None:
        state = self._latest_state.get(self._joint_name)
        if not state:
            return

        position = state["position"]
        velocity = state["velocity"]

        error = self._spring_reference - position
        effort = self._spring_stiffness * error - self._joint_damping * velocity

        msg = Float64MultiArray()
        msg.data = [effort]
        self.get_logger().info(
            "Cmd effort=%.3f (pos=%.3f, vel=%.3f)" % (effort, position, velocity)
        )
        self._effort_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SpringEffortController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
