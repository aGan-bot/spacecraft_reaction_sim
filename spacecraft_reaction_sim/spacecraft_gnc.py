#!/usr/bin/env python3
"""Closed-loop pose guidance, wheel control and RCS allocation."""

from math import copysign, cos, sin, sqrt

import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from spacecraft_reaction_sim.wrench_allocator import allocate_wrench


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quaternion_normalize(quaternion):
    norm = sqrt(sum(value * value for value in quaternion))
    if norm == 0.0:
        raise ValueError('Quaternion norm must be non-zero.')
    return tuple(value / norm for value in quaternion)


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(quaternion, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (*vector, 0.0)),
        quaternion_conjugate(quaternion))
    return rotated[:3]


def world_to_body(quaternion, vector):
    return rotate_vector(quaternion_conjugate(quaternion), vector)


def attitude_error_body(current, target):
    error = quaternion_multiply(quaternion_conjugate(current), target)
    if error[3] < 0.0:
        error = tuple(-value for value in error)
    return tuple(2.0 * value for value in error[:3])


def position_force_world(position, velocity_world, target_position, mass, kp, kd):
    return tuple(mass * (kp * (target - current) - kd * velocity)
                 for current, velocity, target in zip(
                     position, velocity_world, target_position))


def quaternion_from_axis_angle(axis, angle):
    half_angle = angle / 2.0
    sine = sin(half_angle)
    return (axis[0] * sine, axis[1] * sine, axis[2] * sine, cos(half_angle))


def arm_joint_axes_body(joint_positions):
    """Return the six actuated arm axes expressed in spacecraft-body frame."""
    if len(joint_positions) != 6:
        raise ValueError('Exactly six arm joint positions are required.')
    parent_axes = (
        (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0),
    )
    orientation = (0.0, 0.0, 0.0, 1.0)
    body_axes = []
    for parent_axis, position in zip(parent_axes, joint_positions):
        body_axes.append(rotate_vector(orientation, parent_axis))
        orientation = quaternion_multiply(
            orientation, quaternion_from_axis_angle(parent_axis, position))
    return tuple(body_axes)


def arm_feedforward_wheel_effort(joint_positions, joint_efforts, gain):
    """Approximate the wheel command that cancels arm actuator reaction torque.

    The serial-chain motor effort is measured from the joint-state topic. Each arm
    motor applies its counter-torque to the spacecraft; a wheel motor applies
    the opposite torque to the base, so its feed-forward command is negative.
    """
    if len(joint_efforts) != 6:
        raise ValueError('Exactly six arm joint efforts are required.')
    if gain < 0.0:
        raise ValueError('Feed-forward gain must be non-negative.')
    axes = arm_joint_axes_body(joint_positions)
    return tuple(-gain * sum(effort * axis[component]
                             for effort, axis in zip(joint_efforts, axes))
                 for component in range(3))


def wheel_desaturation_effort(wheel_velocity, was_active, start_speed,
                              release_speed, unload_torque):
    active = was_active
    if active and abs(wheel_velocity) <= release_speed:
        active = False
    elif not active and abs(wheel_velocity) >= start_speed:
        active = True
    if not active:
        return 0.0, False
    return -copysign(unload_torque, wheel_velocity), True


class SpacecraftGnc(Node):
    """Owns pose-hold wheel and RCS commands."""

    def __init__(self):
        super().__init__('spacecraft_gnc')
        self.declare_parameter('mass_kg', 400.0)
        self.declare_parameter('position_kp', 0.02)
        self.declare_parameter('position_kd', 0.18)
        self.declare_parameter('attitude_kp', 1.2)
        self.declare_parameter('attitude_kd', 2.0)
        self.declare_parameter('max_wheel_torque', 1.5)
        self.declare_parameter('enable_arm_feedforward', True)
        self.declare_parameter('arm_feedforward_gain', 0.15)
        self.declare_parameter('enable_desaturation', True)
        self.declare_parameter('desaturation_start_speed', 314.159)
        self.declare_parameter('desaturation_release_speed', 261.799)
        self.declare_parameter('desaturation_torque', 0.30)
        self.declare_parameter('rcs_force_weight', 0.25)
        self.declare_parameter('rcs_torque_weight', 1.0)

        self._mass = self.get_parameter('mass_kg').value
        self._position_kp = self.get_parameter('position_kp').value
        self._position_kd = self.get_parameter('position_kd').value
        self._attitude_kp = self.get_parameter('attitude_kp').value
        self._attitude_kd = self.get_parameter('attitude_kd').value
        self._max_wheel_torque = self.get_parameter('max_wheel_torque').value
        self._arm_feedforward_enabled = self.get_parameter('enable_arm_feedforward').value
        self._arm_feedforward_gain = self.get_parameter('arm_feedforward_gain').value
        self._desaturation_enabled = self.get_parameter('enable_desaturation').value
        self._desaturation_start_speed = self.get_parameter('desaturation_start_speed').value
        self._desaturation_release_speed = self.get_parameter('desaturation_release_speed').value
        self._desaturation_torque = self.get_parameter('desaturation_torque').value
        self._rcs_force_weight = self.get_parameter('rcs_force_weight').value
        self._rcs_torque_weight = self.get_parameter('rcs_torque_weight').value
        self._validate()
        self.add_on_set_parameters_callback(self._parameter_callback)

        self._axis_config = (
            ('x', 'wheel_joint_x'), ('y', 'wheel_joint_y'), ('z', 'wheel_joint_z'))
        self._arm_joint_names = tuple(
            'arm_joint_%d' % index for index in range(1, 7))
        self._position = None
        self._orientation = None
        self._linear_velocity_body = None
        self._angular_velocity_body = None
        self._target_position = None
        self._target_orientation = None
        self._wheel_velocities = {}
        self._arm_positions = {}
        self._arm_efforts = {}
        self._desaturation_active = [False, False, False]

        self._wheel_publisher = self.create_publisher(
            Float64MultiArray, '/reaction_wheel_effort_controller/commands', 10)
        self._rcs_publisher = self.create_publisher(
            Actuators, '/spacecraft_arm/command/duty_cycle', 10)
        self.create_subscription(
            Odometry, '/model/spacecraft_arm/odometry', self._odometry_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
        self.create_subscription(
            PoseStamped, '/spacecraft_arm/guidance/target_pose',
            self._target_pose_callback, 10)
        self.create_timer(0.01, self._control_step)
        self.get_logger().info(
            'Waiting for odometry; first pose becomes the pose-hold target.')

    def _validate(self):
        if self._mass <= 0.0:
            raise ValueError('mass_kg must be positive.')
        if (self._position_kp <= 0.0 or self._position_kd <= 0.0 or
                self._attitude_kp <= 0.0 or self._attitude_kd <= 0.0):
            raise ValueError('Controller gains must be positive.')
        if self._max_wheel_torque <= 0.0:
            raise ValueError('max_wheel_torque must be positive.')
        if self._arm_feedforward_gain < 0.0:
            raise ValueError('arm_feedforward_gain must be non-negative.')
        if self._desaturation_release_speed >= self._desaturation_start_speed:
            raise ValueError('Desaturation release speed must be below start speed.')
        if self._desaturation_torque <= 0.0:
            raise ValueError('desaturation_torque must be positive.')
        if self._rcs_force_weight <= 0.0 or self._rcs_torque_weight <= 0.0:
            raise ValueError('RCS allocation weights must be positive.')

    def _odometry_callback(self, message):
        pose = message.pose.pose
        self._position = (pose.position.x, pose.position.y, pose.position.z)
        self._orientation = quaternion_normalize(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w))
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        self._linear_velocity_body = (linear.x, linear.y, linear.z)
        self._angular_velocity_body = (angular.x, angular.y, angular.z)
        if self._target_position is None:
            self._target_position = self._position
            self._target_orientation = self._orientation
            self.get_logger().info('Initial pose captured as the pose-hold target.')

    def _joint_state_callback(self, message):
        for _, joint_name in self._axis_config:
            try:
                index = message.name.index(joint_name)
            except ValueError:
                continue
            if index < len(message.velocity):
                self._wheel_velocities[joint_name] = message.velocity[index]
        for joint_name in self._arm_joint_names:
            try:
                index = message.name.index(joint_name)
            except ValueError:
                continue
            if index < len(message.position):
                self._arm_positions[joint_name] = message.position[index]
            if index < len(message.effort):
                self._arm_efforts[joint_name] = message.effort[index]

    def _target_pose_callback(self, message):
        pose = message.pose
        self._target_position = (pose.position.x, pose.position.y, pose.position.z)
        self._target_orientation = quaternion_normalize(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w))
        self.get_logger().info(
            'Received pose target: position=(%.3f, %.3f, %.3f).' %
            self._target_position)

    def _control_step(self):
        required = (
            self._position, self._orientation, self._linear_velocity_body,
            self._angular_velocity_body, self._target_position,
            self._target_orientation)
        if any(value is None for value in required):
            return

        velocity_world = rotate_vector(
            self._orientation, self._linear_velocity_body)
        force_world = position_force_world(
            self._position, velocity_world, self._target_position, self._mass,
            self._position_kp, self._position_kd)
        force_body = world_to_body(self._orientation, force_world)
        attitude_error = attitude_error_body(
            self._orientation, self._target_orientation)
        # A positive wheel motor torque produces the opposite torque on the
        # spacecraft base.  The error is target relative to current, hence
        # the proportional term is negated; the rate term retains the same
        # sign as the established attitude_hold controller.
        wheel_efforts = [clamp(
            -self._attitude_kp * error + self._attitude_kd * angular_rate,
            -self._max_wheel_torque, self._max_wheel_torque)
            for error, angular_rate in zip(
                attitude_error, self._angular_velocity_body)]

        if self._arm_feedforward_enabled:
            arm_positions = [self._arm_positions.get(name, 0.0)
                             for name in self._arm_joint_names]
            arm_efforts = [self._arm_efforts.get(name, 0.0)
                           for name in self._arm_joint_names]
            feedforward = arm_feedforward_wheel_effort(
                arm_positions, arm_efforts, self._arm_feedforward_gain)
            wheel_efforts = [clamp(feedback + compensation,
                                   -self._max_wheel_torque,
                                   self._max_wheel_torque)
                             for feedback, compensation in
                             zip(wheel_efforts, feedforward)]

        desaturation_torque = [0.0, 0.0, 0.0]
        transitions = []
        for index, (axis, joint_name) in enumerate(self._axis_config):
            was_active = self._desaturation_active[index]
            speed = self._wheel_velocities.get(joint_name)
            if self._desaturation_enabled and speed is not None:
                effort, active = wheel_desaturation_effort(
                    speed, was_active, self._desaturation_start_speed,
                    self._desaturation_release_speed, self._desaturation_torque)
                self._desaturation_active[index] = active
                if active:
                    wheel_efforts[index] = effort
                    desaturation_torque[index] = effort
            if was_active != self._desaturation_active[index]:
                state = 'started' if self._desaturation_active[index] else 'finished'
                transitions.append((axis, state, speed))

        duties = allocate_wrench(
            (*force_body, *desaturation_torque),
            force_weight=self._rcs_force_weight,
            torque_weight=self._rcs_torque_weight)
        for axis, state, speed in transitions:
            self.get_logger().info(
                '%s-wheel momentum desaturation %s '
                '(wheel=%.3f rad/s, max RCS duty=%.3f).' %
                (axis.upper(), state, speed, max(duties)))
        self._wheel_publisher.publish(Float64MultiArray(data=wheel_efforts))
        self._rcs_publisher.publish(Actuators(normalized=duties))

    def _parameter_callback(self, parameters):
        values = {
            'mass_kg': self._mass,
            'position_kp': self._position_kp,
            'position_kd': self._position_kd,
            'attitude_kp': self._attitude_kp,
            'attitude_kd': self._attitude_kd,
            'max_wheel_torque': self._max_wheel_torque,
            'enable_arm_feedforward': self._arm_feedforward_enabled,
            'arm_feedforward_gain': self._arm_feedforward_gain,
            'enable_desaturation': self._desaturation_enabled,
            'desaturation_start_speed': self._desaturation_start_speed,
            'desaturation_release_speed': self._desaturation_release_speed,
            'desaturation_torque': self._desaturation_torque,
            'rcs_force_weight': self._rcs_force_weight,
            'rcs_torque_weight': self._rcs_torque_weight,
        }
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
        if (values['mass_kg'] <= 0.0 or values['position_kp'] <= 0.0 or
                values['position_kd'] <= 0.0 or values['attitude_kp'] <= 0.0 or
                values['attitude_kd'] <= 0.0 or
                values['max_wheel_torque'] <= 0.0 or
                values['arm_feedforward_gain'] < 0.0):
            return SetParametersResult(
                successful=False, reason='Mass, gains and wheel torque must be positive.')
        if values['desaturation_release_speed'] >= values['desaturation_start_speed']:
            return SetParametersResult(
                successful=False, reason='Release speed must be below start speed.')
        if values['desaturation_torque'] <= 0.0:
            return SetParametersResult(
                successful=False, reason='Desaturation torque must be positive.')
        if values['rcs_force_weight'] <= 0.0 or values['rcs_torque_weight'] <= 0.0:
            return SetParametersResult(
                successful=False, reason='RCS allocation weights must be positive.')
        self._mass = values['mass_kg']
        self._position_kp = values['position_kp']
        self._position_kd = values['position_kd']
        self._attitude_kp = values['attitude_kp']
        self._attitude_kd = values['attitude_kd']
        self._max_wheel_torque = values['max_wheel_torque']
        self._arm_feedforward_enabled = values['enable_arm_feedforward']
        self._arm_feedforward_gain = values['arm_feedforward_gain']
        self._desaturation_enabled = values['enable_desaturation']
        self._desaturation_start_speed = values['desaturation_start_speed']
        self._desaturation_release_speed = values['desaturation_release_speed']
        self._desaturation_torque = values['desaturation_torque']
        self._rcs_force_weight = values['rcs_force_weight']
        self._rcs_torque_weight = values['rcs_torque_weight']
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SpacecraftGnc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
