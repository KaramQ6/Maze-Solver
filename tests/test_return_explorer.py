"""Unit tests for Return-Trip exploration module."""

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.return_explorer import (
    compute_return_distance_map,
    get_next_return_move,
    run_return_trip,
)
from maze_solver.core.types import Cell, Direction, MovementCommand, RobotState
from maze_solver.simulator.maze_generator import generate_island_maze
from maze_solver.simulator.virtual_mouse import VirtualMouse


def test_return_distance_map_zero_at_target() -> None:
    """Verify target start corner has distance 0."""
    grid = MazeGrid()
    target = Cell(9, 0)
    dist_map = compute_return_distance_map(grid, target)
    assert dist_map[9][0] == 0
    assert dist_map[8][0] == 1
    assert dist_map[9][1] == 1


def test_get_next_return_move_halts_at_target() -> None:
    """Verify return planner halts when reaching target cell."""
    grid = MazeGrid()
    target = Cell(9, 0)
    state = RobotState(target, Direction.SOUTH)
    dist_map = compute_return_distance_map(grid, target)
    cmd, next_state = get_next_return_move(state, grid, dist_map, set(), target)
    assert cmd == MovementCommand.HALT
    assert next_state == state


def test_run_return_trip_from_center_to_start() -> None:
    """Verify virtual mouse successfully returns from center to start corner."""
    maze = generate_island_maze(seed=42)
    discovered = MazeGrid()
    discovered.initialize_competition_defaults(Cell(9, 0), Direction.NORTH)

    # Place mouse at center cell
    mouse = VirtualMouse(Cell(4, 5), Direction.NORTH)

    success, duration, new_walls = run_return_trip(
        mouse=mouse,
        ground_truth=maze,
        discovered_grid=discovered,
        target_cell=Cell(9, 0),
        max_steps=500,
    )

    assert success is True
    assert mouse.pose.cell == Cell(9, 0)
    assert duration > 0.0
