"""Layer 2: Turn-Weighted State-Space Path Planner for MMRC26 Speed Runs.

Per MMRC26 Master Plan §4.2:
- Searches over (Cell, Direction) state space (400 states on 10x10).
- Applies TURN_PENALTY to realign topological distance with physical run time.
- Uses A* search with Manhattan distance heuristic to guarantee optimal trajectory.
"""

import heapq

from maze_solver.config.settings import MazeConfig
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import (
    Cell,
    MovementCommand,
    RobotState,
)


def _heuristic_manhattan(cell: Cell, goals: tuple[tuple[int, int], ...]) -> float:
    """Calculate admissible Manhattan distance to the closest center goal cell."""
    return float(min(abs(cell.row - gr) + abs(cell.col - gc) for gr, gc in goals))


def plan_turn_weighted_path(
    grid: MazeGrid,
    start_state: RobotState,
    goals: tuple[tuple[int, int], ...] = MazeConfig.GOAL_CELLS,
    turn_penalty: float = MazeConfig.TURN_PENALTY,
    known_cells: set[Cell] | None = None,
) -> list[MovementCommand]:
    """Calculate the optimal turn-weighted sequence of motion commands to reach destination.

    Considers physical turning costs to select the fastest traversable route
    rather than simply the route with the fewest cells.

    If known_cells is specified, the path is strictly constrained to cells
    that have been visited and verified by onboard sensors (preventing
    optimistic traversal through unobserved, potentially blocked walls).
    """
    goal_set = {(r, c) for r, c in goals}

    # If already at destination
    if (start_state.cell.row, start_state.cell.col) in goal_set:
        return []

    # Priority queue stores: (f_score, g_score, counter, state, command_history)
    counter = 0
    start_h = _heuristic_manhattan(start_state.cell, goals)
    open_set: list[tuple[float, float, int, RobotState, list[MovementCommand]]] = [
        (start_h, 0.0, counter, start_state, [])
    ]

    # Best known cost to reach each (cell, heading) state
    g_scores: dict[RobotState, float] = {start_state: 0.0}

    while open_set:
        _, g_cost, _, current_state, commands = heapq.heappop(open_set)

        # Goal check
        if (current_state.cell.row, current_state.cell.col) in goal_set:
            return commands

        # Skip if we already found a cheaper way to this exact state
        if g_cost > g_scores.get(current_state, float("inf")):
            continue

        curr_cell = current_state.cell
        curr_heading = current_state.heading

        # 1. Option: Move FORWARD (if edge is passable)
        if grid.is_passable(curr_cell, curr_heading):
            next_cell = curr_cell.neighbor(curr_heading)

            # If constrained to known cells, reject stepping into unvisited territory
            if (
                known_cells is not None
                and next_cell not in known_cells
                and (next_cell.row, next_cell.col) not in goal_set
            ):
                pass
            else:
                next_state = RobotState(next_cell, curr_heading)
                move_cost = 1.0  # Base unit forward traversal cost
                new_g = g_cost + move_cost

                if new_g < g_scores.get(next_state, float("inf")):
                    g_scores[next_state] = new_g
                    f_score = new_g + _heuristic_manhattan(next_cell, goals)
                    counter += 1
                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            new_g,
                            counter,
                            next_state,
                            [*commands, MovementCommand.FORWARD],
                        ),
                    )

        # 2. Option: TURN_LEFT (in place)
        left_state = RobotState(curr_cell, curr_heading.turn_left())
        left_g = g_cost + turn_penalty
        if left_g < g_scores.get(left_state, float("inf")):
            g_scores[left_state] = left_g
            f_score = left_g + _heuristic_manhattan(curr_cell, goals)
            counter += 1
            heapq.heappush(
                open_set,
                (
                    f_score,
                    left_g,
                    counter,
                    left_state,
                    [*commands, MovementCommand.TURN_LEFT],
                ),
            )

        # 3. Option: TURN_RIGHT (in place)
        right_state = RobotState(curr_cell, curr_heading.turn_right())
        right_g = g_cost + turn_penalty
        if right_g < g_scores.get(right_state, float("inf")):
            g_scores[right_state] = right_g
            f_score = right_g + _heuristic_manhattan(curr_cell, goals)
            counter += 1
            heapq.heappush(
                open_set,
                (
                    f_score,
                    right_g,
                    counter,
                    right_state,
                    [*commands, MovementCommand.TURN_RIGHT],
                ),
            )

        # 4. Option: TURN_AROUND (180-degree turnaround)
        back_state = RobotState(curr_cell, curr_heading.turn_around())
        back_g = g_cost + (2.0 * turn_penalty)
        if back_g < g_scores.get(back_state, float("inf")):
            g_scores[back_state] = back_g
            f_score = back_g + _heuristic_manhattan(curr_cell, goals)
            counter += 1
            heapq.heappush(
                open_set,
                (
                    f_score,
                    back_g,
                    counter,
                    back_state,
                    [*commands, MovementCommand.TURN_AROUND],
                ),
            )

    return []
