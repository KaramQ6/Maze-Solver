"""Unit tests for Diagonal Traversal and Turn Smoothing Planner."""

from maze_solver.core.diagonal_planner import (
    MotionSegmentType,
    evaluate_smoothed_trajectory_time,
    is_diagonal_clear,
    smooth_path_to_diagonals,
)
from maze_solver.core.kinematics import KinematicProfile
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import Cell, Direction, MovementCommand, RobotState


def test_is_diagonal_clear() -> None:
    """Verify corner clearance check for diagonal traversal."""
    grid = MazeGrid()
    # (2,2) to (1,3): North-East diagonal
    from_cell = Cell(2, 2)
    to_cell = Cell(1, 3)

    # On open grid, diagonal is clear
    assert is_diagonal_clear(grid, from_cell, to_cell)

    # If East wall of (2,2) is blocked, diagonal cannot be traversed
    grid.set_wall(from_cell, Direction.EAST, present=True)
    assert not is_diagonal_clear(grid, from_cell, to_cell)


def test_smooth_path_to_diagonals_converts_zigzag() -> None:
    """Verify that a right-left zigzag sequence is transformed into a diagonal sprint."""
    grid = MazeGrid()
    start_state = RobotState(Cell(5, 5), Direction.NORTH)

    # Zigzag sequence: [TURN_RIGHT, FORWARD, TURN_LEFT, FORWARD]
    commands = [
        MovementCommand.TURN_RIGHT,
        MovementCommand.FORWARD,
        MovementCommand.TURN_LEFT,
        MovementCommand.FORWARD,
    ]

    segments = smooth_path_to_diagonals(commands, grid, start_state)
    segment_types = [s.segment_type for s in segments]

    # Must contain a DIAGONAL_SPRINT
    assert MotionSegmentType.DIAGONAL_SPRINT in segment_types


def test_evaluate_smoothed_trajectory_time() -> None:
    """Verify physical time calculation for smoothed diagonal trajectory."""
    profile = KinematicProfile(max_velocity_mps=3.0, max_accel_mps2=10.0)
    grid = MazeGrid()
    start_state = RobotState(Cell(5, 5), Direction.NORTH)

    commands = [
        MovementCommand.FORWARD,
        MovementCommand.FORWARD,
        MovementCommand.TURN_RIGHT,
        MovementCommand.FORWARD,
    ]

    segments = smooth_path_to_diagonals(commands, grid, start_state)
    duration = evaluate_smoothed_trajectory_time(segments, profile)

    assert duration > 0.0
    assert duration < 5.0


def test_smooth_path_to_diagonals_chains_multiple_zigzags() -> None:
    """Verify that multiple consecutive zigzags are chained into a single multi-cell sprint."""
    grid = MazeGrid(10, 10)
    start_state = RobotState(Cell(8, 2), Direction.NORTH)

    # 3 consecutive right-left zigzags
    single_zigzag = [
        MovementCommand.TURN_RIGHT,
        MovementCommand.FORWARD,
        MovementCommand.TURN_LEFT,
        MovementCommand.FORWARD,
    ]
    commands = single_zigzag * 3

    segments = smooth_path_to_diagonals(commands, grid, start_state)

    # Must contain exactly ONE continuous DIAGONAL_SPRINT with units_count=3
    diagonal_sprints = [s for s in segments if s.segment_type == MotionSegmentType.DIAGONAL_SPRINT]
    assert len(diagonal_sprints) == 1
    assert diagonal_sprints[0].units_count == 3
