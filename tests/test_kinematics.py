"""Unit tests for Kinematics and F1 Physics profile engine."""

import math

import pytest

from maze_solver.core.kinematics import (
    KinematicProfile,
    compute_straight_time,
    compute_turn_time,
    evaluate_trajectory_kinematics,
)
from maze_solver.core.types import MovementCommand


def test_straight_trapezoidal_profile() -> None:
    """Verify trapezoidal profile accelerates, cruises, and decelerates."""
    profile = KinematicProfile(max_velocity_mps=3.0, max_accel_mps2=10.0)
    # Long straight distance (3.6 meters = 20 cells)
    dist = 3.6
    time_sec, exit_v = compute_straight_time(dist, v_start=0.0, v_end=0.0, profile=profile)

    # d_accel = 3^2 / 20 = 0.45m (t = 0.3s)
    # d_decel = 3^2 / 20 = 0.45m (t = 0.3s)
    # d_cruise = 3.6 - 0.9 = 2.7m (t = 2.7 / 3 = 0.9s)
    # total = 0.3 + 0.9 + 0.3 = 1.5s
    assert time_sec == pytest.approx(1.5, rel=1e-2)
    assert exit_v == 0.0


def test_suction_fan_increases_cornering_speed() -> None:
    """Verify that suction downforce dramatically increases cornering limit and cuts turn time."""
    no_suction = KinematicProfile(suction_multiplier=0.0, friction_coefficient=0.8)
    with_suction = KinematicProfile(suction_multiplier=3.0, friction_coefficient=0.8)

    v_corner_standard = no_suction.max_cornering_velocity_mps
    v_corner_suction = with_suction.max_cornering_velocity_mps

    # Cornering speed with 3x suction should be twice (sqrt(1+3) = 2) the standard speed
    assert v_corner_suction == pytest.approx(2.0 * v_corner_standard, rel=1e-2)

    # Turn time must be halved
    t_turn_standard = compute_turn_time(no_suction)
    t_turn_suction = compute_turn_time(with_suction)
    assert t_turn_suction == pytest.approx(0.5 * t_turn_standard, rel=1e-2)


def test_trajectory_kinematics_evaluation() -> None:
    """Verify continuous kinematics over a mixed sequence of straight runs and turns."""
    profile = KinematicProfile()
    commands = [
        MovementCommand.FORWARD,
        MovementCommand.FORWARD,
        MovementCommand.TURN_RIGHT,
        MovementCommand.FORWARD,
        MovementCommand.HALT,
    ]

    duration = evaluate_trajectory_kinematics(commands, profile)
    assert duration > 0.0
    assert not math.isnan(duration)
