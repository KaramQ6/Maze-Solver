"""Unit tests for Layer 2 Turn-Weighted State-Space Planner (A*)."""

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.planner import plan_turn_weighted_path
from maze_solver.core.types import Cell, Direction, MovementCommand, RobotState


def test_planner_already_at_goal() -> None:
    """Verify planner returns empty command list when starting inside goal."""
    grid = MazeGrid()
    state = RobotState(Cell(4, 4), Direction.NORTH)
    commands = plan_turn_weighted_path(grid, state, goals=((4, 4),))
    assert commands == []


def test_planner_reaches_goal_on_grid() -> None:
    """Verify planner computes valid path from start to goal."""
    grid = MazeGrid()
    grid.initialize_competition_defaults(Cell(9, 0), Direction.NORTH)
    state = RobotState(Cell(9, 0), Direction.NORTH)
    commands = plan_turn_weighted_path(grid, state, goals=((4, 4),))

    assert len(commands) > 0
    # Simulate commands to verify they reach (4,4)
    curr_cell = state.cell
    curr_h = state.heading
    for cmd in commands:
        if cmd == MovementCommand.FORWARD:
            curr_cell = curr_cell.neighbor(curr_h)
        elif cmd == MovementCommand.TURN_LEFT:
            curr_h = curr_h.turn_left()
        elif cmd == MovementCommand.TURN_RIGHT:
            curr_h = curr_h.turn_right()
        elif cmd == MovementCommand.TURN_AROUND:
            curr_h = curr_h.turn_around()

    assert curr_cell == Cell(4, 4)


def test_planner_penalizes_turns() -> None:
    """Verify planner prefers longer straight path over high-turn zigzag path."""
    grid = MazeGrid()
    # Path 1 (Straight): (3,0) -> (3,1) -> (3,2) (2 forward moves, 0 turns)
    # Path 2 (Zigzag): (3,0) -> (2,0) -> (2,1) -> (3,1) -> (3,2) (4 moves, 3 turns)
    start = RobotState(Cell(3, 0), Direction.EAST)
    goal = ((3, 2),)

    commands = plan_turn_weighted_path(grid, start, goals=goal, turn_penalty=2.0)
    # Planner must choose straight path: [FORWARD, FORWARD]
    assert commands == [MovementCommand.FORWARD, MovementCommand.FORWARD]
