"""Unit tests for RCS normalized actuator commands."""

from spacecraft_reaction_sim.thruster_pulse import normalized_command


def test_active_thruster_uses_full_duty_cycle():
    assert normalized_command(0, True) == [1.0]


def test_inactive_thruster_uses_zero_duty_cycle():
    assert normalized_command(0, False) == [0.0]


def test_second_thruster_uses_the_second_command_slot():
    assert normalized_command(1, True) == [0.0, 1.0]


def test_negative_actuator_index_is_rejected():
    try:
        normalized_command(-1, True)
    except ValueError:
        return
    raise AssertionError("Negative actuator index should be rejected")
