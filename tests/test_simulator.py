"""Unit tests for virtual robot simulation and end-to-end Search Run execution."""

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import Cell, Direction, MovementCommand, RobotState
from maze_solver.simulator.maze_generator import generate_island_maze
from maze_solver.simulator.runner import run_search
from maze_solver.simulator.virtual_mouse import VirtualMouse


def test_virtual_mouse_movement_and_collision() -> None:
    """Verify physical simulation of movement commands and collision prevention."""
    grid = MazeGrid()
    start_cell = Cell(9, 0)
    # Set wall to the NORTH of start
    grid.set_wall(start_cell, Direction.NORTH, present=True)

    mouse = VirtualMouse(start_cell, Direction.NORTH)

    # 1. Forward into wall should fail and log collision
    success = mouse.apply_command(MovementCommand.FORWARD, grid)
    assert not success
    assert mouse.collision_count == 1
    assert mouse.pose == RobotState(start_cell, Direction.NORTH)

    # 2. Turn right should update heading
    turn_ok = mouse.apply_command(MovementCommand.TURN_RIGHT, grid)
    assert turn_ok
    assert mouse.pose == RobotState(start_cell, Direction.EAST)

    # 3. Forward into open EAST should succeed
    move_ok = mouse.apply_command(MovementCommand.FORWARD, grid)
    assert move_ok
    assert mouse.pose == RobotState(Cell(9, 1), Direction.EAST)
    assert mouse.collision_count == 1


def test_virtual_mouse_sensing() -> None:
    """Verify virtual sensor readings against ground truth."""
    grid = MazeGrid()
    cell = Cell(5, 5)
    grid.set_wall(cell, Direction.NORTH, present=True)
    grid.set_wall(cell, Direction.WEST, present=True)

    # Facing NORTH: front has wall, left (WEST) has wall, right (EAST) is open
    mouse = VirtualMouse(cell, Direction.NORTH, noise_rate=0.0)
    sensations = mouse.sense_walls(grid)

    assert sensations.front is True
    assert sensations.left is True
    assert sensations.right is False


def test_end_to_end_search_run_success() -> None:
    """Verify that a robot successfully discovers the center island goal in simulation."""
    # Generate deterministic island-goal maze
    ground_truth = generate_island_maze(seed=42)

    result = run_search(
        ground_truth=ground_truth,
        start_cell=Cell(9, 0),
        initial_heading=Direction.NORTH,
        max_steps=400,
        render=False,
    )

    # Robot must reach destination zone
    assert result.success is True, (
        f"Search run failed, remaining distance: {result.remaining_distance_to_goal}"
    )
    assert result.remaining_distance_to_goal == 0
    assert result.total_steps > 0
    assert result.cell_moves > 0
    assert result.turn_moves > 0
    assert result.run_time_estimate > 0.0
    assert len(result.visited_cells) > 5


def test_renderer_outputs_expected_format() -> None:
    """Verify ASCII renderer renders maze, robot, and distance values."""
    from maze_solver.core.floodfill import compute_distance_map
    from maze_solver.simulator.renderer import render_maze_state

    grid = MazeGrid()
    grid.initialize_competition_defaults(Cell(9, 0), Direction.NORTH)
    state = RobotState(Cell(9, 0), Direction.NORTH)
    dist_map = compute_distance_map(grid)

    output = render_maze_state(grid, robot_state=state, distance_map=dist_map, show_distances=True)
    assert "+" in output
    assert "^" in output
    assert len(output.splitlines()) == 21  # 11 wall lines + 10 cell lines


def test_virtual_mouse_noise_injection() -> None:
    """Verify virtual mouse sensor noise flips bits when enabled."""
    grid = MazeGrid()
    cell = Cell(5, 5)
    mouse = VirtualMouse(cell, Direction.NORTH, noise_rate=1.0, seed=42)
    # Ground truth: no walls
    # With noise_rate=1.0, all readings must be flipped to True
    sensations = mouse.sense_walls(grid)
    assert sensations.front is True
    assert sensations.left is True
    assert sensations.right is True
