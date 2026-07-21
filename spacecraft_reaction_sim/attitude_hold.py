#!/usr/bin/env python3
"""Hold the spacecraft's initial attitude with three reaction wheels."""

from math import asin, atan2

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def quaternion_to_rpy(x, y, z, w):
    """Return roll, pitch, yaw from a normalized quaternion."""
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = 2.0 * (w * y - z * x)
    pitch = asin(max(-1.0, min(1.0, pitch_argument)))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class AttitudeHold(Node):
    """Command wheel torques that counter small spacecraft attitude errors."""

    def __init__(self):
        super().__init__('attitude_hold')
        self.declare_parameter('kp', 0.4)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('max_wheel_torque', 0.5)
        self._kp = self.get_parameter('kp').value
        self._kd = self.get_parameter('kd').value
        self._max_torque = self.get_parameter('max_wheel_torque').value
        self._reference_rpy = None
        self._orientation = None
        self._angular_velocity = None
        self._publisher = self.create_publisher(
            Float64MultiArray, '/reaction_wheel_effort_controller/commands', 10)
        self._subscription = self.create_subscription(
            Odometry, '/model/spacecraft_arm/odometry', self._odometry_callback, 10)
        self._timer = self.create_timer(0.01, self._publish_command)
        self.get_logger().info('Waiting for odometry; first pose becomes the attitude reference.')

    def _odometry_callback(self, message):
        orientation = message.pose.pose.orientation
        self._orientation = quaternion_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w)
        angular = message.twist.twist.angular
        self._angular_velocity = (angular.x, angular.y, angular.z)
        if self._reference_rpy is None:
            self._reference_rpy = self._orientation
            self.get_logger().info('Initial attitude captured as the hold reference.')

    def _publish_command(self):
        if self._reference_rpy is None or self._angular_velocity is None:
            return
        errors = [current - reference for current, reference in zip(
            self._orientation, self._reference_rpy)]
        efforts = [self._wheel_effort(error, velocity)
                   for error, velocity in zip(errors, self._angular_velocity)]
        self._publisher.publish(Float64MultiArray(data=efforts))

    def _wheel_effort(self, error, angular_velocity):
        effort = self._kp * error + self._kd * angular_velocity
        return max(-self._max_torque, min(self._max_torque, effort))


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
