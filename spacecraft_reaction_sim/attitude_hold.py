#!/usr/bin/env python3
"""Hold the spacecraft's initial attitude with three reaction wheels."""

from math import asin, atan2, copysign

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from spacecraft_reaction_sim.wrench_allocator import allocate_wrench


def quaternion_to_rpy(x, y, z, w):
    """Return roll, pitch, yaw from a normalized quaternion."""
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = 2.0 * (w * y - z * x)
    pitch = asin(max(-1.0, min(1.0, pitch_argument)))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wheel_desaturation(wheel_velocity, was_active, start_speed, release_speed,
                       unload_torque, rcs_torque, positive_rcs, negative_rcs):
    """Return braking torque, six RCS duty cycles, and active-state."""
    speed = abs(wheel_velocity)
    active = was_active
    if active and speed <= release_speed:
        active = False
    elif not active and speed >= start_speed:
        active = True
    if not active:
        return 0.0, [0.0] * 6, False

    motor_torque = -copysign(unload_torque, wheel_velocity)
    duty_cycle = min(abs(motor_torque) / rcs_torque, 1.0)
    rcs_command = [0.0] * 6
    rcs_command[negative_rcs if motor_torque < 0.0 else positive_rcs] = duty_cycle
    return motor_torque, rcs_command, True


def z_wheel_desaturation(wheel_velocity, was_active, start_speed,
                          release_speed, unload_torque, rcs_torque):
    """Backward-compatible Z-axis wrapper around the generic desaturation helper."""
    return wheel_desaturation(wheel_velocity, was_active, start_speed, release_speed,
                              unload_torque, rcs_torque, 1, 0)


class AttitudeHold(Node):
    """Command wheel torques that counter small spacecraft attitude errors."""

    def __init__(self):
        super().__init__('attitude_hold')
        self.declare_parameter('kp', 0.4)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('max_wheel_torque', 0.5)
        self.declare_parameter('enable_desaturation', True)
        self.declare_parameter('desaturation_start_speed', 314.159)
        self.declare_parameter('desaturation_release_speed', 261.799)
        self.declare_parameter('desaturation_torque', 0.30)
        self._kp = self.get_parameter('kp').value
        self._kd = self.get_parameter('kd').value
        self._max_torque = self.get_parameter('max_wheel_torque').value
        self._desaturation_enabled = self.get_parameter('enable_desaturation').value
        self._desaturation_start_speed = self.get_parameter('desaturation_start_speed').value
        self._desaturation_release_speed = self.get_parameter('desaturation_release_speed').value
        self._desaturation_torque = self.get_parameter('desaturation_torque').value
        self._axis_config = (
            ('x', 'wheel_joint_x'),
            ('y', 'wheel_joint_y'),
            ('z', 'wheel_joint_z'),
        )
        if self._desaturation_release_speed >= self._desaturation_start_speed:
            raise ValueError('Desaturation release speed must be lower than start speed.')
        if self._desaturation_torque <= 0.0:
            raise ValueError('Desaturation torques must be positive.')
        self.add_on_set_parameters_callback(self._parameter_callback)
        self._reference_rpy = None
        self._orientation = None
        self._angular_velocity = None
        self._wheel_velocities = {}
        self._desaturation_active = [False, False, False]
        self._publisher = self.create_publisher(
            Float64MultiArray, '/reaction_wheel_effort_controller/commands', 10)
        self._rcs_publisher = self.create_publisher(
            Actuators, '/spacecraft_arm/command/duty_cycle', 10)
        self._subscription = self.create_subscription(
            Odometry, '/model/spacecraft_arm/odometry', self._odometry_callback, 10)
        self._joint_subscription = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
        self._timer = self.create_timer(0.01, self._publish_command)
        self.get_logger().info('Waiting for odometry; first pose becomes the attitude reference.')

    def _joint_state_callback(self, message):
        for _, joint_name in self._axis_config:
            try:
                index = message.name.index(joint_name)
            except ValueError:
                continue
            if index < len(message.velocity):
                self._wheel_velocities[joint_name] = message.velocity[index]

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
        self._apply_desaturation(efforts)
        self._publisher.publish(Float64MultiArray(data=efforts))

    def _apply_desaturation(self, efforts):
        desired_torque = [0.0, 0.0, 0.0]
        transitions = []
        for axis_index, (axis, joint_name) in enumerate(self._axis_config):
            was_active = self._desaturation_active[axis_index]
            wheel_velocity = self._wheel_velocities.get(joint_name)
            if self._desaturation_enabled and wheel_velocity is not None:
                effort, _, active = wheel_desaturation(
                    wheel_velocity, was_active, self._desaturation_start_speed,
                    self._desaturation_release_speed, self._desaturation_torque,
                    1.0, 0, 0)
                self._desaturation_active[axis_index] = active
                if active:
                    efforts[axis_index] = effort
                    desired_torque[axis_index] = effort
            if was_active != self._desaturation_active[axis_index]:
                state = "started" if self._desaturation_active[axis_index] else "finished"
                transitions.append((axis, state, wheel_velocity))

        rcs_command = allocate_wrench((0.0, 0.0, 0.0, *desired_torque))
        active_duty = max(rcs_command)
        for axis, state, wheel_velocity in transitions:
            self.get_logger().info(
                "%s-wheel momentum desaturation %s "
                "(wheel=%.3f rad/s, RCS duty=%.3f)." % (
                    axis.upper(), state, wheel_velocity, active_duty))
        self._rcs_publisher.publish(Actuators(normalized=rcs_command))

    def _parameter_callback(self, parameters):
        """Allow desaturation thresholds to be tuned without restarting Gazebo."""
        values = {
            "desaturation_start_speed": self._desaturation_start_speed,
            "desaturation_release_speed": self._desaturation_release_speed,
            "desaturation_torque": self._desaturation_torque,
        }
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value

        if values["desaturation_release_speed"] >= values["desaturation_start_speed"]:
            return SetParametersResult(successful=False,
                                       reason="Release speed must be lower than start speed.")
        if values["desaturation_torque"] <= 0.0:
            return SetParametersResult(successful=False,
                                       reason="Desaturation torque must be positive.")

        self._desaturation_start_speed = values["desaturation_start_speed"]
        self._desaturation_release_speed = values["desaturation_release_speed"]
        self._desaturation_torque = values["desaturation_torque"]
        return SetParametersResult(successful=True)

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
