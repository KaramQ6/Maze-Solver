"""Unit tests for Layer 1 Base Flood Fill algorithm."""

from maze_solver.config.settings import MazeConfig
from maze_solver.core.floodfill import compute_distance_map, get_next_search_move
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import Cell, Direction, MovementCommand, RobotState


def test_distance_map_goals_are_zero() -> None:
    """Verify all destination center cells have distance 0."""
    grid = MazeGrid()
    start = Cell(9, 0)
    grid.initialize_competition_defaults(start_corner=start, initial_heading=Direction.NORTH)

    dist_map = compute_distance_map(grid, MazeConfig.GOAL_CELLS)

    for r, c in MazeConfig.GOAL_CELLS:
        assert dist_map[r][c] == 0

    # Orthogonal neighbors of goals should have distance 1
    assert dist_map[3][4] == 1
    assert dist_map[4][3] == 1
    assert dist_map[6][5] == 1
    assert dist_map[5][6] == 1


def test_floodfill_halt_at_goal() -> None:
    """Verify robot halts when it is inside any destination cell."""
    grid = MazeGrid()
    dist_map = compute_distance_map(grid)
    goal_state = RobotState(cell=Cell(4, 4), heading=Direction.NORTH)

    cmd, next_state = get_next_search_move(goal_state, grid, dist_map)
    assert cmd == MovementCommand.HALT
    assert next_state == goal_state


def test_floodfill_heading_tie_breaker() -> None:
    """Verify Master Plan §4.1: ties for min distance prioritize keeping the current heading."""
    grid = MazeGrid()
    # At (9,0) facing NORTH, moving NORTH reduces distance
    start = Cell(9, 0)
    grid.initialize_competition_defaults(start_corner=start, initial_heading=Direction.NORTH)
    dist_map = compute_distance_map(grid)

    state = RobotState(cell=start, heading=Direction.NORTH)
    cmd, next_state = get_next_search_move(state, grid, dist_map)

    # Must move FORWARD to (8,0)
    assert cmd == MovementCommand.FORWARD
    assert next_state.cell == Cell(8, 0)
    assert next_state.heading == Direction.NORTH


def test_floodfill_turns_when_forward_blocked() -> None:
    """Verify robot turns to optimal neighbor when current heading is blocked by a wall."""
    grid = MazeGrid()
    curr_cell = Cell(2, 2)
    # Block NORTH
    grid.set_wall(curr_cell, Direction.NORTH, present=True)
    dist_map = compute_distance_map(grid)

    state = RobotState(cell=curr_cell, heading=Direction.NORTH)
    cmd, next_state = get_next_search_move(state, grid, dist_map)

    # Robot cannot go FORWARD, so it must turn left, right, or around
    assert cmd in (
        MovementCommand.TURN_LEFT,
        MovementCommand.TURN_RIGHT,
        MovementCommand.TURN_AROUND,
    )
    assert next_state.cell == curr_cell  # Turn in place
    assert next_state.heading != Direction.NORTH


def test_floodfill_unvisited_preference_breaks_oscillation() -> None:
    """Verify that unvisited cells are prioritized over visited cells when distances are tied.

    In a symmetric corridor where North and West both have distance 7:
    If North was already visited, but West is unvisited, the robot facing North must turn West
    rather than blindly repeating the visited North corridor.
    """
    grid = MazeGrid()
    curr_cell = Cell(3, 3)
    dist_map = [[10 for _ in range(10)] for _ in range(10)]

    # Make North and West equidistant (dist 7)
    north_cell = curr_cell.neighbor(Direction.NORTH)  # (2, 3)
    west_cell = curr_cell.neighbor(Direction.WEST)  # (3, 2)
    dist_map[north_cell.row][north_cell.col] = 7
    dist_map[west_cell.row][west_cell.col] = 7

    # Mark north cell as visited, west cell as unvisited
    visited = {north_cell}

    # Facing North: heading tie-breaker would pick North, but visited preference MUST pick West!
    state = RobotState(cell=curr_cell, heading=Direction.NORTH)
    cmd, next_state = get_next_search_move(state, grid, dist_map, visited_cells=visited)

    # West is to the left of North, so robot should turn left towards the unvisited cell
    assert cmd == MovementCommand.TURN_LEFT
    assert next_state.heading == Direction.WEST


def test_incremental_update_matches_full_recompute() -> None:
    """Verify incremental update produces the same distances as full BFS recomputation."""
    from maze_solver.core.floodfill import incremental_update

    grid = MazeGrid()
    start = Cell(9, 0)
    grid.initialize_competition_defaults(start_corner=start, initial_heading=Direction.NORTH)

    # Compute initial distance map
    dist_map = compute_distance_map(grid, MazeConfig.GOAL_CELLS)

    # Add a wall and update incrementally
    wall_cell = Cell(7, 3)
    grid.set_wall(wall_cell, Direction.NORTH, present=True)
    incremental_update(grid, dist_map, wall_cell, MazeConfig.GOAL_CELLS)

    # Recompute from scratch for comparison
    expected = compute_distance_map(grid, MazeConfig.GOAL_CELLS)

    for r in range(grid.rows):
        for c in range(grid.cols):
            assert dist_map[r][c] == expected[r][c], (
                f"Mismatch at ({r},{c}): incremental={dist_map[r][c]}, expected={expected[r][c]}"
            )


def test_incremental_update_no_change_without_wall() -> None:
    """Verify incremental update is a no-op when called without an actual wall change."""
    from maze_solver.core.floodfill import incremental_update

    grid = MazeGrid()
    start = Cell(9, 0)
    grid.initialize_competition_defaults(start_corner=start, initial_heading=Direction.NORTH)

    dist_map = compute_distance_map(grid, MazeConfig.GOAL_CELLS)
    original = [row[:] for row in dist_map]

    # Call incremental update on a cell with no wall change
    incremental_update(grid, dist_map, Cell(5, 5), MazeConfig.GOAL_CELLS)

    for r in range(grid.rows):
        for c in range(grid.cols):
            assert dist_map[r][c] == original[r][c]


def test_fill_dead_ends_marks_dead_end_cells() -> None:
    """Verify dead-end cells (1 or 0 passable neighbors) are marked as unreachable."""
    from maze_solver.core.floodfill import fill_dead_ends

    grid = MazeGrid(4, 4)
    goals = ((1, 1),)

    # Create a dead-end corridor: (0,0) -> (0,1) -> (0,2) with (0,2) as dead-end
    # Wall off (0,2) on N, E, S — only exit is W to (0,1)
    grid.set_wall(Cell(0, 2), Direction.NORTH, True)
    grid.set_wall(Cell(0, 2), Direction.EAST, True)
    grid.set_wall(Cell(0, 2), Direction.SOUTH, True)

    dist_map = compute_distance_map(grid, goals)
    filled = fill_dead_ends(grid, dist_map, goals)

    assert filled >= 1
    assert dist_map[0][2] == MazeConfig.UNREACHABLE_DISTANCE


def test_fill_dead_ends_does_not_fill_goals() -> None:
    """Verify goal cells are never marked as dead-ends even if they have only 1 neighbor."""
    from maze_solver.core.floodfill import fill_dead_ends

    grid = MazeGrid(4, 4)
    goals = ((0, 0),)

    # Wall off (0,0) on all sides except S
    grid.set_wall(Cell(0, 0), Direction.NORTH, True)
    grid.set_wall(Cell(0, 0), Direction.WEST, True)
    grid.set_wall(Cell(0, 0), Direction.EAST, True)

    dist_map = compute_distance_map(grid, goals)
    fill_dead_ends(grid, dist_map, goals)

    # Goal must remain at distance 0
    assert dist_map[0][0] == 0
