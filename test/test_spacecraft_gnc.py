"""Tests for pure spacecraft GNC math."""

from math import cos, pi, sin

from spacecraft_reaction_sim.spacecraft_gnc import (
    attitude_error_body,
    clamp,
    position_force_world,
    rotate_vector,
    wheel_desaturation_effort,
    world_to_body,
)


def test_world_and_body_vector_rotation_are_inverse():
    yaw_90 = (0.0, 0.0, sin(pi / 4.0), cos(pi / 4.0))
    assert tuple(round(value, 12) for value in rotate_vector(yaw_90, (1.0, 0.0, 0.0))) == (
        0.0, 1.0, 0.0)
    assert tuple(round(value, 12) for value in world_to_body(yaw_90, (0.0, 1.0, 0.0))) == (
        1.0, 0.0, 0.0)


def test_attitude_error_uses_shortest_quaternion_direction():
    yaw_90 = (0.0, 0.0, sin(pi / 4.0), cos(pi / 4.0))
    error = attitude_error_body((0.0, 0.0, 0.0, 1.0), yaw_90)
    assert error[0] == 0.0
    assert error[1] == 0.0
    assert error[2] > 1.4


def test_wheel_command_opposes_positive_target_relative_attitude_error():
    """Wheel torque is opposite the torque applied to the spacecraft base."""
    yaw_90 = (0.0, 0.0, sin(pi / 4.0), cos(pi / 4.0))
    error = attitude_error_body((0.0, 0.0, 0.0, 1.0), yaw_90)
    wheel_effort = clamp(-0.4 * error[2], -0.5, 0.5)
    assert wheel_effort < 0.0


def test_position_pd_requests_force_toward_target():
    force = position_force_world(
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, -0.5, 0.0),
        400.0, 0.02, 0.18)
    assert force == (8.0, -4.0, 0.0)


def test_wheel_desaturation_has_hysteresis():
    effort, active = wheel_desaturation_effort(
        315.0, False, 314.159, 261.799, 0.30)
    assert (effort, active) == (-0.30, True)
    effort, active = wheel_desaturation_effort(
        261.799, True, 314.159, 261.799, 0.30)
    assert (effort, active) == (0.0, False)
