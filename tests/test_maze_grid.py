"""Unit tests for edge-based MazeGrid data structure."""

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import Cell, Direction, RobotState, WallSensations


def test_maze_grid_dimensions() -> None:
    """Verify grid array dimensions match the MMRC26 10x10 specification."""
    grid = MazeGrid(rows=10, cols=10)
    assert len(grid.horizontal_walls) == 11
    assert all(len(row) == 10 for row in grid.horizontal_walls)
    assert len(grid.vertical_walls) == 10
    assert all(len(row) == 11 for row in grid.vertical_walls)


def test_shared_edge_no_desync() -> None:
    """Verify that a wall between two cells is stored on a single edge."""
    grid = MazeGrid(rows=10, cols=10)
    c00 = Cell(0, 0)
    c10 = Cell(1, 0)

    # Initially open
    assert not grid.has_wall(c00, Direction.SOUTH)
    assert not grid.has_wall(c10, Direction.NORTH)

    # Set south wall of (0,0)
    grid.set_wall(c00, Direction.SOUTH, present=True)

    # North wall of (1,0) MUST automatically be True
    assert grid.has_wall(c00, Direction.SOUTH)
    assert grid.has_wall(c10, Direction.NORTH)

    # Clear north wall of (1,0)
    grid.set_wall(c10, Direction.NORTH, present=False)
    assert not grid.has_wall(c00, Direction.SOUTH)
    assert not grid.has_wall(c10, Direction.NORTH)


def test_competition_defaults() -> None:
    """Verify initialization of perimeter walls and start cell bounded on 3 sides."""
    grid = MazeGrid(rows=10, cols=10)
    start_cell = Cell(9, 0)  # Southwest corner
    initial_heading = Direction.NORTH

    grid.initialize_competition_defaults(start_corner=start_cell, initial_heading=initial_heading)

    # Perimeter walls must all be True
    for c in range(10):
        assert grid.has_wall(Cell(0, c), Direction.NORTH)
        assert grid.has_wall(Cell(9, c), Direction.SOUTH)
    for r in range(10):
        assert grid.has_wall(Cell(r, 0), Direction.WEST)
        assert grid.has_wall(Cell(r, 9), Direction.EAST)

    # Start cell at (9,0) with initial heading NORTH:
    # Exit is NORTH -> open
    assert grid.is_passable(start_cell, Direction.NORTH)
    # West, South, East must have walls
    assert grid.has_wall(start_cell, Direction.WEST)
    assert grid.has_wall(start_cell, Direction.SOUTH)
    assert grid.has_wall(start_cell, Direction.EAST)


def test_update_from_sensations() -> None:
    """Verify wall updates from onboard sensor sensations."""
    grid = MazeGrid(rows=10, cols=10)
    state = RobotState(cell=Cell(3, 3), heading=Direction.NORTH)

    # Front and right walls detected
    sensations = WallSensations(front=True, left=False, right=True)
    changed = grid.update_from_sensations(state, sensations)

    assert changed
    assert grid.has_wall(Cell(3, 3), Direction.NORTH)
    assert not grid.has_wall(Cell(3, 3), Direction.WEST)
    assert grid.has_wall(Cell(3, 3), Direction.EAST)
