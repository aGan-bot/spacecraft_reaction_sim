"""Tests for the three-corner RCS wrench allocator."""

from spacecraft_reaction_sim.wrench_allocator import allocate_wrench, wrench_from_duty


def test_zero_wrench_keeps_all_nozzles_closed():
    assert allocate_wrench((0.0,) * 6) == [0.0] * 6


def test_equal_x_pair_cancels_force_and_creates_pitch_torque():
    wrench = wrench_from_duty([0.4, 0.0, 0.4, 0.0, 0.0, 0.0])
    assert wrench == (0.0, 0.0, 0.0, 0.0, -0.4, 0.0)


def test_allocator_respects_duty_bounds_and_tracks_pitch_torque():
    desired = (0.0, 0.0, 0.0, 0.0, -0.30, 0.0)
    duty = allocate_wrench(desired)
    actual = wrench_from_duty(duty)
    assert all(0.0 <= value <= 1.0 for value in duty)
    assert abs(actual[4] - desired[4]) < 1e-3


def test_allocator_can_request_a_translation_component():
    duty = allocate_wrench((0.4, 0.0, 0.0, 0.0, 0.0, 0.0))
    actual = wrench_from_duty(duty)
    assert actual[0] > 0.2
