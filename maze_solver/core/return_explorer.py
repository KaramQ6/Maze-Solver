"""Return-trip exploration engine for secondary maze discovery.

Per historic All-Japan Championship strategy:
After reaching the center island goal in Phase A, the robot navigates back to
the start square along alternative unvisited paths to discover longer straightaways
and alternative optimal routes before the Speed Run.
"""

from collections import deque

from maze_solver.config.settings import MazeConfig
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RobotState,
)
from maze_solver.simulator.virtual_mouse import VirtualMouse


def compute_return_distance_map(
    grid: MazeGrid,
    target_cell: Cell,
) -> list[list[int]]:
    """Compute BFS topological distance map from all cells towards the start corner."""
    rows, cols = grid.rows, grid.cols
    distance: list[list[int]] = [
        [MazeConfig.UNREACHABLE_DISTANCE for _ in range(cols)] for _ in range(rows)
    ]
    queue: deque[Cell] = deque()

    distance[target_cell.row][target_cell.col] = 0
    queue.append(target_cell)

    while queue:
        current = queue.popleft()
        current_dist = distance[current.row][current.col]

        for neighbor, _ in grid.passable_neighbors(current):
            if distance[neighbor.row][neighbor.col] > current_dist + 1:
                distance[neighbor.row][neighbor.col] = current_dist + 1
                queue.append(neighbor)

    return distance


def get_next_return_move(
    state: RobotState,
    grid: MazeGrid,
    distance_map: list[list[int]],
    visited_cells: set[Cell],
    target_cell: Cell,
) -> tuple[MovementCommand, RobotState]:
    """Determine next return-trip motion command, prioritizing unvisited candidate cells."""
    if state.cell == target_cell:
        return MovementCommand.HALT, state

    passable = grid.passable_neighbors(state.cell)
    if not passable:
        new_heading = state.heading.turn_around()
        return MovementCommand.TURN_AROUND, RobotState(state.cell, new_heading)

    min_dist = min(distance_map[nb.row][nb.col] for nb, _ in passable)

    # Candidates that achieve minimum distance to start cell
    candidates = [
        (nb, direction) for nb, direction in passable if distance_map[nb.row][nb.col] == min_dist
    ]

    # Exploration heuristic: prioritize candidate cells not yet visited
    unvisited_candidates = [(nb, d) for nb, d in candidates if nb not in visited_cells]
    chosen_pool = unvisited_candidates if unvisited_candidates else candidates

    current_h = state.heading
    directions_pool = {d for _, d in chosen_pool}

    # Tie-breaker hierarchy: straight -> left -> right -> back
    left_h = current_h.turn_left()
    right_h = current_h.turn_right()
    back_h = current_h.turn_around()

    target_direction: Direction
    if current_h in directions_pool:
        target_direction = current_h
    elif left_h in directions_pool:
        target_direction = left_h
    elif right_h in directions_pool:
        target_direction = right_h
    else:
        target_direction = back_h

    if target_direction == current_h:
        next_cell = state.cell.neighbor(current_h)
        return MovementCommand.FORWARD, RobotState(next_cell, current_h)
    if target_direction == left_h:
        return MovementCommand.TURN_LEFT, RobotState(state.cell, left_h)
    if target_direction == right_h:
        return MovementCommand.TURN_RIGHT, RobotState(state.cell, right_h)

    return MovementCommand.TURN_AROUND, RobotState(state.cell, back_h)


def run_return_trip(
    mouse: VirtualMouse,
    ground_truth: MazeGrid,
    discovered_grid: MazeGrid,
    target_cell: Cell | None = None,
    visited_cells: set[Cell] | None = None,
    max_steps: int = 400,
) -> tuple[bool, float, int]:
    """Execute autonomous return trip from center back to start square.

    Returns (success, duration_seconds, new_walls_discovered).
    """
    if target_cell is None:
        target_cell = Cell(9, 0)
    if visited_cells is None:
        visited_cells = set()

    step_time = 0.25
    turn_time = step_time * MazeConfig.TURN_PENALTY
    total_time = 0.0
    new_walls_count = 0

    for _ in range(max_steps):
        current_pose = mouse.pose
        visited_cells.add(current_pose.cell)

        if current_pose.cell == target_cell:
            return True, total_time, new_walls_count

        # 1. Sense and update map along the return path
        sensations = mouse.sense_walls(ground_truth)
        if discovered_grid.update_from_sensations(current_pose, sensations):
            new_walls_count += 1

        # 2. Recompute return distance map
        dist_map = compute_return_distance_map(discovered_grid, target_cell)

        # 3. Get next move
        command, _ = get_next_return_move(
            current_pose, discovered_grid, dist_map, visited_cells, target_cell
        )

        if command == MovementCommand.HALT:
            return True, total_time, new_walls_count

        if command == MovementCommand.FORWARD:
            total_time += step_time
        else:
            total_time += turn_time

        ok = mouse.apply_command(command, ground_truth)
        if not ok:
            # Blocked
            return False, total_time, new_walls_count

    return mouse.pose.cell == target_cell, total_time, new_walls_count
