"""Layer 1: Base Flood Fill algorithm for exploration and mapping.

Per MMRC26 Master Plan §4.1:
- Recomputes topological distance from the 4 central goal cells.
- Emits deterministic movement commands.
- Fixed tie-breaking policy: always prioritizes continuing in the current heading.
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


def compute_distance_map(
    grid: MazeGrid,
    goals: tuple[tuple[int, int], ...] = MazeConfig.GOAL_CELLS,
) -> list[list[int]]:
    """Compute shortest topological distance from all cells to destination zone using BFS."""
    rows, cols = grid.rows, grid.cols
    distance: list[list[int]] = [
        [MazeConfig.UNREACHABLE_DISTANCE for _ in range(cols)] for _ in range(rows)
    ]
    queue: deque[Cell] = deque()

    # Seed the queue with all goal cells
    for r, c in goals:
        if 0 <= r < rows and 0 <= c < cols:
            distance[r][c] = 0
            queue.append(Cell(r, c))

    while queue:
        current = queue.popleft()
        current_dist = distance[current.row][current.col]

        for neighbor, _ in grid.passable_neighbors(current):
            if distance[neighbor.row][neighbor.col] > current_dist + 1:
                distance[neighbor.row][neighbor.col] = current_dist + 1
                queue.append(neighbor)

    return distance


def get_next_search_move(
    state: RobotState,
    grid: MazeGrid,
    distance_map: list[list[int]],
    goals: tuple[tuple[int, int], ...] = MazeConfig.GOAL_CELLS,
    visited_cells: set[Cell] | dict[Cell, int] | None = None,
) -> tuple[MovementCommand, RobotState]:
    """Determine the next discrete navigation command during the search/mapping run.

    Tie-breaking priority per Master Plan §4.1 (Anti-Oscillation):
    1. Unvisited or least-visited cells (prevents cyclical ping-pong traps)
    2. Forward (current heading)
    3. Left
    4. Right
    5. Turn Around
    """
    # Check if robot has arrived at destination
    if (state.cell.row, state.cell.col) in goals:
        return MovementCommand.HALT, state

    passable = grid.passable_neighbors(state.cell)
    if not passable:
        # Dead end with no exit: turn around in place
        new_heading = state.heading.turn_around()
        return MovementCommand.TURN_AROUND, RobotState(state.cell, new_heading)

    # Find the minimum distance value among accessible neighbors
    min_dist = min(distance_map[neighbor.row][neighbor.col] for neighbor, _ in passable)

    # Filter candidate directions that achieve min_dist
    candidates = [
        (neighbor, direction)
        for neighbor, direction in passable
        if distance_map[neighbor.row][neighbor.col] == min_dist
    ]

    current_h = state.heading
    left_h = current_h.turn_left()
    right_h = current_h.turn_right()
    back_h = current_h.turn_around()

    def heading_rank(d: Direction) -> int:
        if d == current_h:
            return 0
        if d == left_h:
            return 1
        if d == right_h:
            return 2
        return 3

    def visit_rank(c: Cell) -> int:
        if visited_cells is None:
            return 0
        if isinstance(visited_cells, set):
            return 1 if c in visited_cells else 0
        return visited_cells.get(c, 0)

    # Sort candidates: least visited first, then deterministic heading order
    best_candidate = min(
        candidates,
        key=lambda item: (visit_rank(item[0]), heading_rank(item[1])),
    )
    target_direction = best_candidate[1]

    # Translate target direction to motion primitive
    if target_direction == current_h:
        next_cell = state.cell.neighbor(current_h)
        return MovementCommand.FORWARD, RobotState(next_cell, current_h)
    if target_direction == left_h:
        return MovementCommand.TURN_LEFT, RobotState(state.cell, left_h)
    if target_direction == right_h:
        return MovementCommand.TURN_RIGHT, RobotState(state.cell, right_h)

    return MovementCommand.TURN_AROUND, RobotState(state.cell, back_h)
