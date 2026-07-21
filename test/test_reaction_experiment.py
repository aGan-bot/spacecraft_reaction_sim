"""Unit checks for the spacecraft reaction experiment model."""

from spacecraft_reaction_sim.reaction_experiment import ReactionExperiment


def test_profile_has_accelerate_coast_brake_sequence():
    """The prescribed effort profile accelerates, coasts, then brakes."""
    experiment = ReactionExperiment.__new__(ReactionExperiment)
    experiment._torque_nm = 1.0

    assert experiment._profile(0.0) == 1.0
    assert experiment._profile(1.0) == 0.0
    assert experiment._profile(3.0) == -1.0
    assert experiment._profile(4.0) == 0.0


def test_base_velocity_follows_angular_momentum_relation():
    """A 2 rad/s relative wrist speed yields the expected base reaction."""
    experiment = ReactionExperiment.__new__(ReactionExperiment)
    experiment._base_inertia_z = 123.0
    experiment._wrist_inertia_z = 0.389

    assert abs(experiment._expected_base_velocity(2.0) + 0.006305) < 0.00001


def test_total_angular_momentum_is_zero_for_predicted_reaction():
    """The predicted base reaction preserves the initial zero momentum."""
    experiment = ReactionExperiment.__new__(ReactionExperiment)
    experiment._base_inertia_z = 123.0
    experiment._wrist_inertia_z = 0.389
    joint_velocity = 2.0
    base_velocity = experiment._expected_base_velocity(joint_velocity)

    assert abs(experiment._total_angular_momentum(base_velocity, joint_velocity)) < 1e-12
