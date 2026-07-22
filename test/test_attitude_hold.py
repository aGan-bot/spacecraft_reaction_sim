"""Unit tests for the reaction-wheel attitude hold helper math."""

from math import pi, sin

from spacecraft_reaction_sim.attitude_hold import (
    AttitudeHold, quaternion_to_rpy, z_wheel_desaturation,
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
        185.0, False, 180.0, 150.0, 0.30, 0.55)
    assert torque == -0.30
    assert command == [0.30 / 0.55, 0.0]
    assert active


def test_negative_z_wheel_unload_uses_positive_z_rcs():
    torque, command, active = z_wheel_desaturation(
        -185.0, False, 180.0, 150.0, 0.30, 0.55)
    assert torque == 0.30
    assert command == [0.0, 0.30 / 0.55]
    assert active


def test_desaturation_stops_at_release_speed():
    torque, command, active = z_wheel_desaturation(
        150.0, True, 180.0, 150.0, 0.30, 0.55)
    assert torque == 0.0
    assert command == [0.0, 0.0]
    assert not active
