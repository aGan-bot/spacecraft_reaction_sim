#!/usr/bin/env python3
"""Apply a repeatable torque profile and report the spacecraft reaction."""

from math import isfinite

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ReactionExperiment(Node):
    """Publish wrist torque and compare measured motion with momentum theory."""

    def __init__(self):
        super().__init__('reaction_experiment')
        self.declare_parameter('base_inertia_z', 123.0)
        self.declare_parameter('wrist_inertia_z', 0.389)
        self.declare_parameter('torque_nm', 1.0)
        self.declare_parameter('command_topic', '/wrist_effort_controller/commands')
        self._base_inertia_z = self.get_parameter('base_inertia_z').value
        self._wrist_inertia_z = self.get_parameter('wrist_inertia_z').value
        self._torque_nm = self.get_parameter('torque_nm').value
        topic = self.get_parameter('command_topic').value
        self._publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self._subscription = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
        self._odometry_subscription = self.create_subscription(
            Odometry, '/model/spacecraft_reaction/odometry',
            self._odometry_callback, 10)
        self._timer = self.create_timer(0.01, self._publish_command)
        self._start_time = None
        self._last_report_time = None
        self._joint_velocity = None
        self._base_angular_velocity_z = None
        self.get_logger().info('Waiting for /joint_states before starting torque profile.')

    def _joint_state_callback(self, message):
        if 'joint_6' not in message.name:
            return
        index = message.name.index('joint_6')
        if index < len(message.velocity):
            velocity = message.velocity[index]
            if isfinite(velocity):
                self._joint_velocity = velocity

    def _odometry_callback(self, message):
        self._base_angular_velocity_z = message.twist.twist.angular.z

    def _publish_command(self):
        now = self.get_clock().now()
        if self._joint_velocity is None:
            self._publisher.publish(Float64MultiArray(data=[0.0]))
            return
        if self._start_time is None:
            self._start_time = now
            self.get_logger().info('Applying 1, 0, -1 N m torque profile over 4 seconds.')
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        effort = self._profile(elapsed)
        self._publisher.publish(Float64MultiArray(data=[effort]))
        if self._last_report_time is None or (now - self._last_report_time).nanoseconds >= 100_000_000:
            self._last_report_time = now
            self._report_state(elapsed, effort)

    def _report_state(self, elapsed, effort):
        expected_base_velocity = self._expected_base_velocity(self._joint_velocity)
        if self._base_angular_velocity_z is None:
            self.get_logger().info(
                't=%.2f s, effort=%.2f N m, joint_6 velocity=%.6f rad/s, '
                'predicted base wz=%.6f rad/s' % (
                    elapsed, effort, self._joint_velocity, expected_base_velocity))
            return
        momentum = self._total_angular_momentum(
            self._base_angular_velocity_z, self._joint_velocity)
        self.get_logger().info(
            't=%.2f s, effort=%.2f N m, joint_6 velocity=%.6f rad/s, '
            'base wz=%.6f rad/s, predicted wz=%.6f rad/s, H_z=%.6e kg m^2/s' % (
                elapsed, effort, self._joint_velocity,
                self._base_angular_velocity_z, expected_base_velocity, momentum))

    def _profile(self, elapsed):
        if elapsed < 1.0:
            return self._torque_nm
        if elapsed < 3.0:
            return 0.0
        if elapsed < 4.0:
            return -self._torque_nm
        return 0.0

    def _expected_base_velocity(self, joint_velocity):
        denominator = self._base_inertia_z + self._wrist_inertia_z
        return -self._wrist_inertia_z * joint_velocity / denominator

    def _total_angular_momentum(self, base_velocity, joint_velocity):
        return ((self._base_inertia_z + self._wrist_inertia_z) * base_velocity
                + self._wrist_inertia_z * joint_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = ReactionExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
