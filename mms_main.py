"""MMS (Micromouse Simulator by mackorone) Bridge Adapter for MMRC26 Maze Solver.

Implements the official MMS protocol with strict stdin/stdout synchronization.
"""

import sys
from collections import defaultdict
from typing import Any

from maze_solver.core.floodfill import compute_distance_map, get_next_search_move
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.planner import plan_turn_weighted_path
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RobotState,
    WallSensations,
)


class MMS_API:
    """Official MMS API interface with strict command-response synchronization."""

    @staticmethod
    def _command(args: list[Any], return_type: Any = None) -> Any:
        line = " ".join([str(x) for x in args]) + "\n"
        sys.stdout.write(line)
        sys.stdout.flush()
        if return_type is not None:
            response = sys.stdin.readline().strip()
            if return_type is bool:
                return response == "true"
            return return_type(response)
        return None

    @classmethod
    def maze_width(cls) -> int:
        return int(cls._command(["mazeWidth"], int) or 16)

    @classmethod
    def maze_height(cls) -> int:
        return int(cls._command(["mazeHeight"], int) or 16)

    @classmethod
    def wall_front(cls) -> bool:
        return bool(cls._command(["wallFront"], bool))

    @classmethod
    def wall_right(cls) -> bool:
        return bool(cls._command(["wallRight"], bool))

    @classmethod
    def wall_left(cls) -> bool:
        return bool(cls._command(["wallLeft"], bool))

    @classmethod
    def wall_back(cls) -> bool:
        return bool(cls._command(["wallBack"], bool))

    @classmethod
    def move_forward(cls) -> None:
        # MMS replies with ack / crash
        cls._command(["moveForward"], str)

    @classmethod
    def turn_right(cls) -> None:
        # MMS replies with ack
        cls._command(["turnRight"], str)

    @classmethod
    def turn_left(cls) -> None:
        # MMS replies with ack
        cls._command(["turnLeft"], str)

    @classmethod
    def set_wall(cls, x: int, y: int, direction_char: str) -> None:
        cls._command(["setWall", x, y, direction_char])

    @classmethod
    def set_color(cls, x: int, y: int, color_char: str) -> None:
        cls._command(["setColor", x, y, color_char])

    @classmethod
    def set_text(cls, x: int, y: int, text: str) -> None:
        cls._command(["setText", x, y, text])

    @classmethod
    def log(cls, message: str) -> None:
        sys.stderr.write(f"[MMRC26] {message}\n")
        sys.stderr.flush()


def run_mms_solver() -> None:
    """Main control loop executing inside the MMS simulator."""
    width = MMS_API.maze_width()
    height = MMS_API.maze_height()
    MMS_API.log(f"Starting MMRC26 Solver in MMS ({width}x{height})")

    grid = MazeGrid(rows=height, cols=width)

    # 1. Enclose outer perimeter boundaries
    for r in range(height):
        grid.vertical_walls[r][0] = True
        grid.vertical_walls[r][width] = True
    for c in range(width):
        grid.horizontal_walls[0][c] = True
        grid.horizontal_walls[height][c] = True

    # 2. Robot starts at bottom-left corner: (x=0, y=0) in MMS -> (row=height-1, col=0)
    current_cell = Cell(row=height - 1, col=0)
    current_heading = Direction.NORTH
    state = RobotState(current_cell, current_heading)

    # Mark start cell
    MMS_API.set_color(0, 0, "c")

    # 3. Center goal definition (2x2 center)
    center_row_low = (height // 2) - 1
    center_col_low = (width // 2) - 1
    goals = (
        (center_row_low, center_col_low),
        (center_row_low, center_col_low + 1),
        (center_row_low + 1, center_col_low),
        (center_row_low + 1, center_col_low + 1),
    )

    for gr, gc in goals:
        gx = gc
        gy = height - 1 - gr
        MMS_API.set_color(gx, gy, "y")

    step_count = 0
    max_steps = width * height * 8
    visit_counts: dict[Cell, int] = defaultdict(int)

    dir_char_map = {
        Direction.NORTH: "n",
        Direction.EAST: "e",
        Direction.SOUTH: "s",
        Direction.WEST: "w",
    }

    # -------------------------------------------------------------
    # PHASE 1: EXPLORATION / FLOODFILL RUN TO ISLAND GOAL
    # -------------------------------------------------------------
    while step_count < max_steps:
        step_count += 1
        r, c = state.cell.row, state.cell.col
        x, y = c, height - 1 - r

        # Read sensors with synchronized responses
        wf = MMS_API.wall_front()
        wl = MMS_API.wall_left()
        wr = MMS_API.wall_right()
        wb = MMS_API.wall_back()

        # Update local grid model
        sensations = WallSensations(front=wf, left=wl, right=wr)
        grid.update_from_sensations(state, sensations)
        if wb:
            grid.set_wall(state.cell, state.heading.opposite(), True)

        # Mirror walls to MMS visualizer
        if wf:
            MMS_API.set_wall(x, y, dir_char_map[state.heading])
        if wl:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.turn_left()])
        if wr:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.turn_right()])
        if wb:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.opposite()])

        # Trail color and visitation tracking (anti-oscillation)
        visit_counts[state.cell] += 1
        MMS_API.set_color(x, y, "G")

        # Check goal arrival
        if (state.cell.row, state.cell.col) in goals:
            MMS_API.log(f"SUCCESS! Goal reached in {step_count} steps.")
            for gr, gc in goals:
                MMS_API.set_color(gc, height - 1 - gr, "Y")
            break

        # Compute distance map
        dist_map = compute_distance_map(grid, goals=goals)
        d_val = dist_map[r][c]
        if d_val < 9000:
            MMS_API.set_text(x, y, str(d_val))

        # Next move prioritizing unvisited exploration over oscillation loops
        cmd, next_state = get_next_search_move(
            state, grid, dist_map, goals=goals, visited_cells=visit_counts
        )

        # Dispatch motion
        if cmd == MovementCommand.FORWARD:
            MMS_API.move_forward()
            state = next_state
        elif cmd == MovementCommand.TURN_LEFT:
            MMS_API.turn_left()
            state = next_state
        elif cmd == MovementCommand.TURN_RIGHT:
            MMS_API.turn_right()
            state = next_state
        elif cmd == MovementCommand.TURN_AROUND:
            MMS_API.turn_right()
            MMS_API.turn_right()
            state = next_state
        elif cmd == MovementCommand.HALT:
            break

    # -------------------------------------------------------------
    # PHASE 2: OPTIMAL TURN-WEIGHTED SPEED RUN HIGHLIGHTING
    # -------------------------------------------------------------
    MMS_API.log("Calculating optimal turn-weighted speed run path...")
    start_state = RobotState(Cell(height - 1, 0), Direction.NORTH)
    speed_cmds = plan_turn_weighted_path(grid, start_state, goals=goals)
    MMS_API.log(f"Speed run plan: {len(speed_cmds)} actions.")

    # Highlight optimal path in blue
    curr_sim = start_state
    for cmd in speed_cmds:
        if cmd == MovementCommand.FORWARD:
            curr_sim = RobotState(curr_sim.cell.neighbor(curr_sim.heading), curr_sim.heading)
            MMS_API.set_color(curr_sim.cell.col, height - 1 - curr_sim.cell.row, "B")
        elif cmd == MovementCommand.TURN_LEFT:
            curr_sim = RobotState(curr_sim.cell, curr_sim.heading.turn_left())
        elif cmd == MovementCommand.TURN_RIGHT:
            curr_sim = RobotState(curr_sim.cell, curr_sim.heading.turn_right())


if __name__ == "__main__":
    run_mms_solver()
