"""Unit tests for effort-based arm position hold math."""

from spacecraft_reaction_sim.arm_position_hold import clamp, position_effort


def test_clamp_enforces_joint_limits():
    """Out-of-range position targets are clipped to the allowed interval."""
    assert clamp(4.0, -3.0, 3.0) == 3.0
    assert clamp(-4.0, -3.0, 3.0) == -3.0


def test_position_effort_opposes_position_and_velocity_error():
    """The PD command accelerates toward target and damps movement."""
    assert position_effort(1.0, 0.0, 0.0, 2.0, 1.0, 10.0) == 2.0
    assert position_effort(1.0, 0.0, 3.0, 2.0, 1.0, 10.0) == -1.0


def test_position_effort_is_saturated():
    """The configured torque bound is never exceeded."""
    assert position_effort(10.0, 0.0, 0.0, 5.0, 1.0, 2.0) == 2.0
