#!/usr/bin/env python3
"""Effort-based position hold for the six-axis free-floating arm."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_NAMES = [f'arm_joint_{index}' for index in range(1, 7)]
LOWER_LIMITS = [-2.967, -1.745, -2.268, -3.142, -2.094, -3.142]
UPPER_LIMITS = [2.967, 1.745, 2.268, 3.142, 2.094, 3.142]
TORQUE_LIMITS = [30.0, 30.0, 25.0, 20.0, 15.0, 10.0]
DEFAULT_KP = [14.0, 14.0, 12.0, 9.0, 7.0, 5.0]
DEFAULT_KD = [4.0, 4.0, 3.5, 2.5, 2.0, 1.5]


def clamp(value, lower, upper):
    """Clamp a scalar to an inclusive interval."""
    return max(lower, min(upper, value))


def position_effort(target, position, velocity, kp, kd, torque_limit):
    """Return one saturated PD effort command."""
    return clamp(kp * (target - position) - kd * velocity,
                 -torque_limit, torque_limit)


class ArmPositionHold(Node):
    """Hold requested joint positions by publishing effort commands."""

    def __init__(self):
        super().__init__('arm_position_hold')
        self._positions = None
        self._velocities = None
        self._targets = None
        self._publisher = self.create_publisher(
            Float64MultiArray, '/arm_effort_controller/commands', 10)
        self._state_subscription = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
        self._target_subscription = self.create_subscription(
            Float64MultiArray, '/arm_position_targets', self._target_callback, 10)
        self._timer = self.create_timer(0.01, self._publish_efforts)
        self.get_logger().info(
            'Waiting for joint states; current six-axis pose becomes the initial hold target.')

    def _joint_state_callback(self, message):
        try:
            indices = [message.name.index(name) for name in JOINT_NAMES]
        except ValueError:
            return
        if any(index >= len(message.position) or index >= len(message.velocity)
               for index in indices):
            return
        self._positions = [message.position[index] for index in indices]
        self._velocities = [message.velocity[index] for index in indices]
        if self._targets is None:
            self._targets = list(self._positions)
            self.get_logger().info('Initial arm pose captured and held.')

    def _target_callback(self, message):
        if len(message.data) != len(JOINT_NAMES):
            self.get_logger().error('Expected exactly 6 target positions in radians.')
            return
        self._targets = [clamp(value, lower, upper) for value, lower, upper in zip(
            message.data, LOWER_LIMITS, UPPER_LIMITS)]
        self.get_logger().info('New position target accepted within joint limits.')

    def _publish_efforts(self):
        if self._positions is None or self._targets is None:
            return
        efforts = [position_effort(target, position, velocity, kp, kd, torque_limit)
                   for target, position, velocity, kp, kd, torque_limit in zip(
                       self._targets, self._positions, self._velocities,
                       DEFAULT_KP, DEFAULT_KD, TORQUE_LIMITS)]
        self._publisher.publish(Float64MultiArray(data=efforts))


def main(args=None):
    rclpy.init(args=args)
    node = ArmPositionHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
