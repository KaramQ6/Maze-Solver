"""Unit tests for procedural MMRC26 Island-Goal maze generator."""

from maze_solver.core.types import Cell, Direction
from maze_solver.simulator.maze_generator import (
    CENTER_EDGES,
    check_lattice_points_satisfied,
    generate_island_maze,
    is_center_detached,
    is_solvable,
)


def test_generator_island_goal_specs() -> None:
    """Verify that generated mazes strictly comply with MMRC26 rules."""
    start_cell = Cell(9, 0)
    grid = generate_island_maze(seed=123, start_corner=start_cell, initial_heading=Direction.NORTH)

    # 1. 10x10 dimensions
    assert grid.rows == 10
    assert grid.cols == 10

    # 2. Outer perimeter enclosed
    for c in range(10):
        assert grid.horizontal_walls[0][c]
        assert grid.horizontal_walls[10][c]
    for r in range(10):
        assert grid.vertical_walls[r][0]
        assert grid.vertical_walls[r][10]

    # 3. Strictly ONE entrance to the center zone
    open_edges = 0
    for edge_type, r, c in CENTER_EDGES:
        wall_present = (
            grid.horizontal_walls[r][c] if edge_type == "H" else grid.vertical_walls[r][c]
        )
        if not wall_present:
            open_edges += 1
    assert open_edges == 1, f"Center zone must have exactly 1 entrance, found {open_edges}"

    # 4. Center walls detached from outer perimeter
    assert is_center_detached(grid)

    # 5. At least one wall attached to every lattice point (§5.d)
    assert check_lattice_points_satisfied(grid)

    # 6. Solvable from start corner
    assert is_solvable(grid, start_cell)


def test_multiple_seeds_produce_valid_mazes() -> None:
    """Verify validity across multiple random seeds."""
    for seed in [1, 42, 99, 2026]:
        grid = generate_island_maze(seed=seed)
        assert is_solvable(grid, Cell(9, 0))
        assert is_center_detached(grid)
        assert check_lattice_points_satisfied(grid)
