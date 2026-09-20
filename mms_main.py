"""MMS (Micromouse Simulator by mackorone) Bridge Adapter for MMRC26 Maze Solver.

Allows our MMRC26 maze-solving engine and FloodFill / A* planner to run directly
inside the official Mackorone Micromouse Simulator (MMS).
"""

import sys
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
    """Interface to communicate with Mackorone Micromouse Simulator over stdin/stdout."""

    @staticmethod
    def _command(args: list[str], return_type: Any = None) -> Any:
        line = " ".join([str(x) for x in args]) + "\n"
        sys.stdout.write(line)
        sys.stdout.flush()
        if return_type:
            response = sys.stdin.readline().strip()
            if return_type is bool:
                return response == "true"
            return return_type(response)
        return None

    @classmethod
    def maze_width(cls) -> int:
        return int(cls._command(["mazeWidth"], int) or 10)

    @classmethod
    def maze_height(cls) -> int:
        return int(cls._command(["mazeHeight"], int) or 10)

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
    def move_forward(cls) -> None:
        cls._command(["moveForward"])

    @classmethod
    def turn_right(cls) -> None:
        cls._command(["turnRight"])

    @classmethod
    def turn_left(cls) -> None:
        cls._command(["turnLeft"])

    @classmethod
    def set_wall(cls, x: int, y: int, direction_char: str) -> None:
        cls._command(["setWall", str(x), str(y), direction_char])

    @classmethod
    def set_color(cls, x: int, y: int, color_char: str) -> None:
        cls._command(["setColor", str(x), str(y), color_char])

    @classmethod
    def set_text(cls, x: int, y: int, text: str) -> None:
        cls._command(["setText", str(x), str(y), text])

    @classmethod
    def log(cls, message: str) -> None:
        sys.stderr.write(f"[MMRC26] {message}\n")
        sys.stderr.flush()


def run_mms_solver() -> None:
    """Main control loop executing inside the MMS simulator."""
    width = MMS_API.maze_width()
    height = MMS_API.maze_height()
    MMS_API.log(f"Starting MMRC26 Solver in MMS ({width}x{height})")

    # MMRC26 uses a 10x10 grid with bottom-left as start (9, 0 in row/col representation)
    grid = MazeGrid(rows=height, cols=width)

    # Robot starts at bottom-left corner: (x=0, y=0) in MMS -> (row=height-1, col=0)
    current_cell = Cell(row=height - 1, col=0)
    current_heading = Direction.NORTH
    state = RobotState(current_cell, current_heading)

    # Mark start cell
    MMS_API.set_color(0, 0, "c")  # Cyan

    # Goal definition: center 2x2 cells
    center_row_low = (height // 2) - 1
    center_col_low = (width // 2) - 1
    goals = (
        (center_row_low, center_col_low),
        (center_row_low, center_col_low + 1),
        (center_row_low + 1, center_col_low),
        (center_row_low + 1, center_col_low + 1),
    )

    # Highlight goals in yellow/orange
    for gr, gc in goals:
        gx = gc
        gy = height - 1 - gr
        MMS_API.set_color(gx, gy, "y")

    step_count = 0
    max_steps = width * height * 4

    # -------------------------------------------------------------
    # PHASE 1: EXPLORATION / FLOODFILL RUN TO ISLAND GOAL
    # -------------------------------------------------------------
    while step_count < max_steps:
        step_count += 1
        r, c = state.cell.row, state.cell.col
        x, y = c, height - 1 - r

        # 1. Sense walls from MMS
        wf = MMS_API.wall_front()
        wl = MMS_API.wall_left()
        wr = MMS_API.wall_right()

        sensations = WallSensations(front=wf, left=wl, right=wr)
        grid.update_from_sensations(state, sensations)

        # Mirror walls to MMS visualizer
        dir_char_map = {
            Direction.NORTH: "n",
            Direction.EAST: "e",
            Direction.SOUTH: "s",
            Direction.WEST: "w",
        }
        if wf:
            MMS_API.set_wall(x, y, dir_char_map[state.heading])
        if wl:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.turn_left()])
        if wr:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.turn_right()])

        # Color current path
        MMS_API.set_color(x, y, "G")

        # 2. Check if arrived at Island Goal
        if (state.cell.row, state.cell.col) in goals:
            MMS_API.log(f"Island Goal reached in {step_count} steps!")
            for gr, gc in goals:
                MMS_API.set_color(gc, height - 1 - gr, "Y")
            break

        # 3. Compute FloodFill distance map
        dist_map = compute_distance_map(grid, goals=goals)
        MMS_API.set_text(x, y, str(dist_map[r][c]))

        # 4. Get next movement command
        cmd, next_state = get_next_search_move(state, grid, dist_map, goals=goals)

        # 5. Execute command in MMS
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
    # PHASE 2: OPTIMAL SPEED RUN HIGHLIGHTING
    # -------------------------------------------------------------
    MMS_API.log("Planning optimal Turn-Weighted Speed Run...")
    start_state = RobotState(Cell(height - 1, 0), Direction.NORTH)
    speed_cmds = plan_turn_weighted_path(grid, start_state, goals=goals)
    MMS_API.log(f"Optimal Speed Run commands: {len(speed_cmds)}")

    # Color the optimal path in Blue/Cyan
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
