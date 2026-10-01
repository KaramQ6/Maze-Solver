"""MMS (Micromouse Simulator by mackorone) Bridge Adapter for MMRC26 Maze Solver.

Implements the official MMS protocol with strict stdin/stdout synchronization.
"""

import os
import sys
import time
from collections import defaultdict
from typing import Any

from maze_solver.core.floodfill import (
    compute_distance_map,
    get_next_search_move,
)
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.planner import path_cost, plan_turn_weighted_path
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RobotState,
    WallSensations,
)


def read_mms_start(width: int, height: int) -> RobotState:
    """Validate the simulator's launch pose and convert bottom-left MMS coordinates."""
    if width < 1 or height < 1:
        raise ValueError("MMS maze dimensions must be positive")
    keys = ("MMS_START_X", "MMS_START_Y", "MMS_START_HEADING")
    provided = [key in os.environ for key in keys]
    if any(provided) and not all(provided):
        raise ValueError("MMS start requires X, Y, and heading together")
    try:
        x = int(os.environ.get("MMS_START_X", "0"))
        y = int(os.environ.get("MMS_START_Y", "0"))
    except ValueError as error:
        raise ValueError("MMS start coordinates must be integers") from error
    headings = {
        "n": Direction.NORTH,
        "e": Direction.EAST,
        "s": Direction.SOUTH,
        "w": Direction.WEST,
    }
    heading = os.environ.get("MMS_START_HEADING", "n").lower()
    if heading not in headings or not (0 <= x < width and 0 <= y < height):
        raise ValueError("MMS start must be inside the maze with a cardinal heading (n/e/s/w)")
    return RobotState(Cell(height - 1 - y, x), headings[heading])


class MMSResetRequested(Exception):
    """The native mouse has reset; discard the old discovered map and restart."""


def find_first_unvisited_on_path(
    commands: list[MovementCommand],
    start: RobotState,
    visited: set[Cell],
) -> Cell | None:
    """Find the first unvisited cell along a planned sequence of commands."""
    curr = start
    for cmd in commands:
        if cmd == MovementCommand.FORWARD:
            next_cell = curr.cell.neighbor(curr.heading)
            if next_cell not in visited:
                return next_cell
            curr = RobotState(next_cell, curr.heading)
        elif cmd == MovementCommand.TURN_LEFT:
            curr = RobotState(curr.cell, curr.heading.turn_left())
        elif cmd == MovementCommand.TURN_RIGHT:
            curr = RobotState(curr.cell, curr.heading.turn_right())
        elif cmd == MovementCommand.TURN_AROUND:
            curr = RobotState(curr.cell, curr.heading.turn_around())
    return None


class MMS_API:
    """Official MMS API interface with strict command-response synchronization."""

    @staticmethod
    def _command(args: list[Any], return_type: Any = None) -> Any:
        line = " ".join([str(x) for x in args]) + "\n"
        sys.stdout.write(line)
        sys.stdout.flush()
        if return_type is not None:
            response = sys.stdin.readline().strip()
            if not response:
                raise RuntimeError("MMS closed the protocol stream")
            if return_type is bool:
                if response not in ("true", "false"):
                    raise RuntimeError(f"Invalid MMS boolean response: {response}")
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
        cls.check_reset()
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
        cls.check_reset()
        # MMS replies with ack / crash
        response = cls._command(["moveForward"], str)
        if response != "ack":
            raise RuntimeError(f"MMS moveForward failed: {response}")

    @classmethod
    def turn_right(cls) -> None:
        cls.check_reset()
        # MMS replies with ack
        response = cls._command(["turnRight"], str)
        if response != "ack":
            raise RuntimeError(f"MMS turnRight failed: {response}")

    @classmethod
    def turn_left(cls) -> None:
        cls.check_reset()
        # MMS replies with ack
        response = cls._command(["turnLeft"], str)
        if response != "ack":
            raise RuntimeError(f"MMS turnLeft failed: {response}")

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
    def check_reset(cls) -> None:
        if os.environ.get("MMS_MMRC26") == "1" and cls._command(["wasReset"], bool):
            response = cls._command(["ackReset"], str)
            if response != "ack":
                raise RuntimeError(f"MMS ackReset failed: {response}")
            raise MMSResetRequested

    @classmethod
    def set_motion_mode(cls, mode: str) -> None:
        if mode not in ("search", "speed"):
            raise ValueError("MMS motion mode must be search or speed")
        if os.environ.get("MMS_MMRC26") == "1":
            cls._command(["setMotionMode", mode])

    @classmethod
    def log(cls, message: str) -> None:
        sys.stderr.write(f"[MMRC26] {message}\n")
        sys.stderr.flush()


def run_mms_solver() -> None:
    """Restart cleanly after an acknowledged native reset."""
    while True:
        try:
            _run_mms_solver()
            return
        except MMSResetRequested:
            MMS_API.log("Reset acknowledged; restarting from the selected start pose.")


def _run_mms_solver() -> None:
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

    # 2. The native simulator supplies the same pose it uses to place the mouse.
    start_state = read_mms_start(width, height)
    state = start_state
    MMS_API.set_motion_mode("search")
    MMS_API.log(
        f"Start: x={state.cell.col}, y={height - 1 - state.cell.row}, heading={state.heading.name}"
    )

    # Mark start cell
    MMS_API.set_color(state.cell.col, height - 1 - state.cell.row, "c")

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

    if (start_state.cell.row, start_state.cell.col) in goals:
        MMS_API.log("Start is already in the goal; no start-to-goal traversal to execute.")
        return

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

        # Recompute exact topological distance map on every step (guarantees zero local traps)
        dist_map = compute_distance_map(grid, goals=goals)

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

        # Display current distance on cell
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

    # Guard: Verify that the center goal was actually reached
    if (state.cell.row, state.cell.col) not in goals:
        MMS_API.log(
            "WARNING: Goal was not reached! The center goal in this maze is "
            "disconnected or sealed by walls."
        )
        return

    # -------------------------------------------------------------
    # PHASE 1.5: RETURN TO START FOR SPEED RUN
    # -------------------------------------------------------------
    MMS_API.log("Phase 1.5: Returning to start for speed run...")
    start_cell = (start_state.cell.row, start_state.cell.col)
    return_goals: tuple[tuple[int, int], ...] = (start_cell,)
    return_steps = 0
    max_return_steps = width * height * 4

    while return_steps < max_return_steps:
        return_steps += 1
        r, c = state.cell.row, state.cell.col
        x, y = c, height - 1 - r

        # Read sensors with synchronized responses
        wf = MMS_API.wall_front()
        wl = MMS_API.wall_left()
        wr = MMS_API.wall_right()
        wb = MMS_API.wall_back()

        # Update local grid model (continue exploring on the way back)
        sensations = WallSensations(front=wf, left=wl, right=wr)
        grid.update_from_sensations(state, sensations)
        if wb:
            grid.set_wall(state.cell, state.heading.opposite(), True)

        # Recompute exact return distance map
        return_dist_map = compute_distance_map(grid, goals=return_goals)

        # Mirror walls to MMS visualizer
        if wf:
            MMS_API.set_wall(x, y, dir_char_map[state.heading])
        if wl:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.turn_left()])
        if wr:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.turn_right()])
        if wb:
            MMS_API.set_wall(x, y, dir_char_map[state.heading.opposite()])

        # Color return path orange
        MMS_API.set_color(x, y, "o")
        visit_counts[state.cell] += 1

        # Check if we reached start
        if (state.cell.row, state.cell.col) == start_cell:
            MMS_API.log(f"Returned to start in {return_steps} steps.")
            break

        # Next move toward start
        cmd, next_state = get_next_search_move(
            state, grid, return_dist_map, goals=return_goals, visited_cells=visit_counts
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

    if (state.cell.row, state.cell.col) != start_cell:
        MMS_API.log("Return trip did not reach start; aborting speed run.")
        return

    # -------------------------------------------------------------
    # PHASE 2: ALL-JAPAN ACTIVE SHORTCUT PROBING
    # -------------------------------------------------------------
    max_probes = 4
    for probe_idx in range(1, max_probes + 1):
        # Align to the selected initial heading at the selected start.
        while state.heading != start_state.heading:
            MMS_API.turn_right()
            state = RobotState(state.cell, state.heading.turn_right())

        visited = set(visit_counts.keys())

        safe_cmds = plan_turn_weighted_path(grid, start_state, goals=goals, known_cells=visited)
        optimistic_cmds = plan_turn_weighted_path(grid, start_state, goals=goals)

        if not safe_cmds:
            MMS_API.log("No verified route available; aborting shortcut probing.")
            break
        if not optimistic_cmds or path_cost(optimistic_cmds) >= path_cost(safe_cmds):
            MMS_API.log("No cheaper optimistic route remains to probe.")
            break

        probe_target = find_first_unvisited_on_path(optimistic_cmds, start_state, visited)
        if probe_target is None:
            break

        MMS_API.log(
            f"All-Japan Phase 2: Probe {probe_idx}/{max_probes} - Active corridor probing "
            f"(Safe: {len(safe_cmds)} actions | Optimistic: {len(optimistic_cmds)} actions)..."
        )
        probe_steps = 0
        max_probe_steps = width * height * 2

        # Actively probe along the optimistic corridor towards the goal
        while probe_steps < max_probe_steps:
            probe_steps += 1
            pr, pc = state.cell.row, state.cell.col
            px, py = pc, height - 1 - pr

            wf = MMS_API.wall_front()
            wl = MMS_API.wall_left()
            wr = MMS_API.wall_right()
            wb = MMS_API.wall_back()

            sensations = WallSensations(front=wf, left=wl, right=wr)
            grid.update_from_sensations(state, sensations)
            if wb:
                grid.set_wall(state.cell, state.heading.opposite(), True)

            if wf:
                MMS_API.set_wall(px, py, dir_char_map[state.heading])
            if wl:
                MMS_API.set_wall(px, py, dir_char_map[state.heading.turn_left()])
            if wr:
                MMS_API.set_wall(px, py, dir_char_map[state.heading.turn_right()])
            if wb:
                MMS_API.set_wall(px, py, dir_char_map[state.heading.opposite()])

            visit_counts[state.cell] += 1
            MMS_API.set_color(px, py, "y")

            # Probe the first unknown cell of the candidate shortcut, then replan.
            if state.cell == probe_target:
                break

            probe_dist = compute_distance_map(grid, goals=((probe_target.row, probe_target.col),))
            if probe_dist[pr][pc] >= 9000:
                MMS_API.log(f"Probe {probe_idx}: Blocked corridor at ({px}, {py}); re-evaluating.")
                break

            cmd, next_state = get_next_search_move(
                state,
                grid,
                probe_dist,
                goals=((probe_target.row, probe_target.col),),
                visited_cells=visit_counts,
            )

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

        # Return to start after probe run
        ret_steps = 0
        while ret_steps < max_return_steps:
            ret_steps += 1
            if (state.cell.row, state.cell.col) == start_cell:
                break

            wf = MMS_API.wall_front()
            wl = MMS_API.wall_left()
            wr = MMS_API.wall_right()
            wb = MMS_API.wall_back()

            sensations = WallSensations(front=wf, left=wl, right=wr)
            grid.update_from_sensations(state, sensations)
            if wb:
                grid.set_wall(state.cell, state.heading.opposite(), True)

            visit_counts[state.cell] += 1
            MMS_API.set_color(state.cell.col, height - 1 - state.cell.row, "o")

            ret_dist = compute_distance_map(grid, goals=return_goals)
            cmd, next_state = get_next_search_move(
                state, grid, ret_dist, goals=return_goals, visited_cells=visit_counts
            )

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

        if (state.cell.row, state.cell.col) != start_cell:
            MMS_API.log("Probe return did not reach start; aborting speed run.")
            return

        # Re-align to the selected initial heading at start.
        while state.heading != start_state.heading:
            MMS_API.turn_right()
            state = RobotState(state.cell, state.heading.turn_right())

    # -------------------------------------------------------------
    # PHASE 3: PROVEN GLOBAL OPTIMAL SPEED RUN EXECUTION
    # -------------------------------------------------------------
    visited = set(visit_counts.keys())
    safe_cmds = plan_turn_weighted_path(grid, start_state, goals=goals, known_cells=visited)
    optimistic_cmds = plan_turn_weighted_path(grid, start_state, goals=goals)

    if safe_cmds and optimistic_cmds and path_cost(optimistic_cmds) >= path_cost(safe_cmds):
        MMS_API.log(
            "PROVEN GLOBAL OPTIMUM: Mathematically proven fastest route across entire maze!"
        )
        speed_cmds = safe_cmds
    elif safe_cmds:
        speed_cmds = safe_cmds
        MMS_API.log(f"Verified Safe Path: {len(speed_cmds)} actions.")
    else:
        MMS_API.log("No verified path to goal; aborting speed run.")
        return

    if not speed_cmds:
        MMS_API.log("WARNING: No valid path to goal could be computed! Aborting speed run.")
        return

    MMS_API.set_motion_mode("speed")
    MMS_API.log(f"Phase 3: Executing speed run ({len(speed_cmds)} actions)...")
    speed_run_started = time.monotonic()

    # Execute speed run through the real MMS API
    state = start_state
    for cmd in speed_cmds:
        if cmd == MovementCommand.FORWARD:
            MMS_API.move_forward()
            state = RobotState(state.cell.neighbor(state.heading), state.heading)
            sx, sy = state.cell.col, height - 1 - state.cell.row
            MMS_API.set_color(sx, sy, "B")
        elif cmd == MovementCommand.TURN_LEFT:
            MMS_API.turn_left()
            state = RobotState(state.cell, state.heading.turn_left())
        elif cmd == MovementCommand.TURN_RIGHT:
            MMS_API.turn_right()
            state = RobotState(state.cell, state.heading.turn_right())
        elif cmd == MovementCommand.TURN_AROUND:
            MMS_API.turn_right()
            MMS_API.turn_right()
            state = RobotState(state.cell, state.heading.opposite())

    # Check if speed run reached the goal
    if (state.cell.row, state.cell.col) in goals:
        elapsed = time.monotonic() - speed_run_started
        MMS_API.log(
            f"CHAMPIONSHIP FINISH! Reached goal in {len(speed_cmds)} actions; "
            f"wall time = {elapsed:.2f}s (includes Pause and protocol overhead)."
        )


if __name__ == "__main__":
    try:
        run_mms_solver()
    except (RuntimeError, ValueError) as error:
        MMS_API.log(f"Solver stopped: {error}")
        sys.exit(1)
