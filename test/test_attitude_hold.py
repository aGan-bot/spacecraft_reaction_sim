"""Unit tests for the reaction-wheel attitude hold helper math."""

from math import pi, sin

from spacecraft_reaction_sim.attitude_hold import (
    AttitudeHold, quaternion_to_rpy, wheel_desaturation, z_wheel_desaturation,
)


def test_identity_quaternion_has_zero_rpy():
    """The identity quaternion is the zero attitude."""
    assert quaternion_to_rpy(0.0, 0.0, 0.0, 1.0) == (0.0, 0.0, 0.0)


def test_quaternion_yaw_is_converted_correctly():
    """A 90-degree yaw quaternion yields the expected yaw angle."""
    _, _, yaw = quaternion_to_rpy(0.0, 0.0, sin(pi / 4.0), sin(pi / 4.0))
    assert abs(yaw - pi / 2.0) < 1e-12


def test_wheel_effort_is_limited():
    """Wheel torque is clamped to the configured safe bound."""
    controller = AttitudeHold.__new__(AttitudeHold)
    controller._kp = 0.4
    controller._kd = 1.0
    controller._max_torque = 0.5
    assert controller._wheel_effort(4.0, 4.0) == 0.5


def test_positive_z_wheel_unload_uses_negative_z_rcs():
    torque, command, active = z_wheel_desaturation(
        315.0, False, 314.159, 261.799, 0.30, 0.55)
    assert torque == -0.30
    assert command == [0.30 / 0.55, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert active


def test_negative_z_wheel_unload_uses_positive_z_rcs():
    torque, command, active = z_wheel_desaturation(
        -315.0, False, 314.159, 261.799, 0.30, 0.55)
    assert torque == 0.30
    assert command == [0.0, 0.30 / 0.55, 0.0, 0.0, 0.0, 0.0]
    assert active


def test_desaturation_stops_at_release_speed():
    torque, command, active = z_wheel_desaturation(
        261.799, True, 314.159, 261.799, 0.30, 0.55)
    assert torque == 0.0
    assert command == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert not active


def test_positive_x_wheel_unload_uses_negative_x_rcs():
    torque, command, active = wheel_desaturation(
        315.0, False, 314.159, 261.799, 0.30, 0.52, 2, 3)
    assert torque == -0.30
    assert command == [0.0, 0.0, 0.0, 0.30 / 0.52, 0.0, 0.0]
    assert active


def test_negative_y_wheel_unload_uses_positive_y_rcs():
    torque, command, active = wheel_desaturation(
        -315.0, False, 314.159, 261.799, 0.30, 0.52, 4, 5)
    assert torque == 0.30
    assert command == [0.0, 0.0, 0.0, 0.0, 0.30 / 0.52, 0.0]
    assert active
