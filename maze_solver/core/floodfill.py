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


def incremental_update(
    grid: MazeGrid,
    distance_map: list[list[int]],
    changed_cell: Cell,
    goals: tuple[tuple[int, int], ...] = MazeConfig.GOAL_CELLS,
) -> None:
    """Update distance_map in-place after a wall change near changed_cell.

    Instead of recomputing the entire BFS (O(N^2)), this propagates only from
    cells whose distances became invalid due to a newly discovered wall.
    Amortized cost: O(K) where K << N^2 is the number of affected cells.
    """
    goal_set = {(r, c) for r, c in goals}
    rows, cols = grid.rows, grid.cols
    # ponytail: uses a simple FIFO re-flood from invalidated cells.
    # Could upgrade to Bellman-Ford stack for even fewer updates.

    # Phase 1: Invalidate cells whose current distance relies on a now-blocked path
    invalidated: deque[Cell] = deque()

    # Check if changed_cell's distance is still consistent
    if (changed_cell.row, changed_cell.col) in goal_set:
        return  # Goal cells always have distance 0

    _check_and_invalidate(grid, distance_map, changed_cell, goal_set, invalidated)

    # Also check all neighbors of changed_cell (their paths may have gone through it)
    for neighbor, _ in grid.passable_neighbors(changed_cell):
        _check_and_invalidate(grid, distance_map, neighbor, goal_set, invalidated)

    # Phase 2: Re-flood from valid neighbors of invalidated cells
    repair_queue: deque[Cell] = deque()
    for cell in invalidated:
        for neighbor, _ in grid.passable_neighbors(cell):
            if distance_map[neighbor.row][neighbor.col] < MazeConfig.UNREACHABLE_DISTANCE:
                repair_queue.append(neighbor)

    # Also seed from goals in case invalidation disconnected some paths
    for r, c in goals:
        if 0 <= r < rows and 0 <= c < cols:
            repair_queue.append(Cell(r, c))

    while repair_queue:
        current = repair_queue.popleft()
        current_dist = distance_map[current.row][current.col]

        for neighbor, _ in grid.passable_neighbors(current):
            if distance_map[neighbor.row][neighbor.col] > current_dist + 1:
                distance_map[neighbor.row][neighbor.col] = current_dist + 1
                repair_queue.append(neighbor)


def _check_and_invalidate(
    grid: MazeGrid,
    distance_map: list[list[int]],
    cell: Cell,
    goal_set: set[tuple[int, int]],
    invalidated: deque[Cell],
) -> None:
    """Check if a cell's distance is still valid; if not, invalidate it and propagate."""
    if (cell.row, cell.col) in goal_set:
        return

    current_dist = distance_map[cell.row][cell.col]
    if current_dist >= MazeConfig.UNREACHABLE_DISTANCE:
        return

    passable = grid.passable_neighbors(cell)
    if not passable:
        # No exits — cell is now unreachable
        distance_map[cell.row][cell.col] = MazeConfig.UNREACHABLE_DISTANCE
        invalidated.append(cell)
        return

    # A cell's distance should be min(neighbor distances) + 1
    min_neighbor = min(distance_map[n.row][n.col] for n, _ in passable)
    if current_dist != min_neighbor + 1:
        distance_map[cell.row][cell.col] = MazeConfig.UNREACHABLE_DISTANCE
        invalidated.append(cell)
        # Propagate: neighbors that depended on this cell may also be invalid
        for neighbor, _ in passable:
            n_dist = distance_map[neighbor.row][neighbor.col]
            if n_dist == current_dist + 1:
                _check_and_invalidate(grid, distance_map, neighbor, goal_set, invalidated)


def fill_dead_ends(
    grid: MazeGrid,
    distance_map: list[list[int]],
    goals: tuple[tuple[int, int], ...] = MazeConfig.GOAL_CELLS,
) -> int:
    """Mark dead-end cells as unreachable and propagate through dead-end chains.

    A dead-end is a cell with only 1 passable neighbor (excluding goal cells).
    Returns the number of cells filled.
    """
    goal_set = {(r, c) for r, c in goals}
    filled = 0
    stack: list[Cell] = []

    # Find initial dead-ends
    for r in range(grid.rows):
        for c in range(grid.cols):
            if (r, c) in goal_set:
                continue
            cell = Cell(r, c)
            passable = grid.passable_neighbors(cell)
            if len(passable) <= 1 and distance_map[r][c] < MazeConfig.UNREACHABLE_DISTANCE:
                distance_map[r][c] = MazeConfig.UNREACHABLE_DISTANCE
                filled += 1
                if passable:
                    stack.append(passable[0][0])

    # Propagate: if filling a dead-end makes its neighbor a dead-end too
    while stack:
        cell = stack.pop()
        if (cell.row, cell.col) in goal_set:
            continue
        if distance_map[cell.row][cell.col] >= MazeConfig.UNREACHABLE_DISTANCE:
            continue

        # Count remaining passable neighbors with valid distances
        valid_neighbors = [
            (n, d)
            for n, d in grid.passable_neighbors(cell)
            if distance_map[n.row][n.col] < MazeConfig.UNREACHABLE_DISTANCE
        ]

        if len(valid_neighbors) <= 1:
            distance_map[cell.row][cell.col] = MazeConfig.UNREACHABLE_DISTANCE
            filled += 1
            for n, _ in valid_neighbors:
                stack.append(n)

    return filled


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
