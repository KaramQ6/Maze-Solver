"""Diagonal Traversal and Turn Smoothing Planner ("The Fosbury Flop").

Inspired by David Otten's MIT Mitee-3 innovation (1980s All-Japan Championships):
- Converts orthogonal zigzag patterns into 45-degree diagonal sprints across open lattice posts.
- Reduces travel distance by 29.3% per diagonal step (sqrt(2) vs 2.0 orthogonal units).
- Eliminates stop-and-pivot 90-degree braking by replacing corners with continuous arc entries.
- Validates corner post clearance to prevent chassis collisions.
"""

import math
from dataclasses import dataclass
from enum import Enum

from maze_solver.core.kinematics import KinematicProfile, compute_straight_time
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RobotState,
)


class MotionSegmentType(Enum):
    """Primitive motion segments for smoothed trajectory execution."""

    ORTHOGONAL_STRAIGHT = "ORTHOGONAL_STRAIGHT"
    DIAGONAL_SPRINT = "DIAGONAL_SPRINT"
    TURN_90_LEFT = "TURN_90_LEFT"
    TURN_90_RIGHT = "TURN_90_RIGHT"
    TURN_45_ENTRY_LEFT = "TURN_45_ENTRY_LEFT"
    TURN_45_ENTRY_RIGHT = "TURN_45_ENTRY_RIGHT"
    TURN_45_EXIT_LEFT = "TURN_45_EXIT_LEFT"
    TURN_45_EXIT_RIGHT = "TURN_45_EXIT_RIGHT"
    TURN_180 = "TURN_180"


@dataclass(frozen=True)
class MotionSegment:
    """A continuous trajectory segment executed by the drivetrain controller."""

    segment_type: MotionSegmentType
    distance_m: float
    units_count: int = 1


def is_diagonal_clear(
    grid: MazeGrid,
    from_cell: Cell,
    to_cell: Cell,
) -> bool:
    """Verify that both orthogonal corner boundaries are open so mouse can cut diagonally.

    Diagonal movement from (r1, c1) to (r2, c2) requires that the two intermediate
    cells (r1, c2) and (r2, c1) do not have walls blocking the diagonal corridor.
    """
    dr = to_cell.row - from_cell.row
    dc = to_cell.col - from_cell.col

    if abs(dr) != 1 or abs(dc) != 1:
        return False

    # Intermediate corners
    c_horizontal = Cell(from_cell.row, to_cell.col)
    c_vertical = Cell(to_cell.row, from_cell.col)

    if not grid.is_valid_cell(c_horizontal) or not grid.is_valid_cell(c_vertical):
        return False

    # Check horizontal passage from from_cell to c_horizontal
    h_dir = Direction.EAST if dc > 0 else Direction.WEST
    v_dir = Direction.SOUTH if dr > 0 else Direction.NORTH

    if not grid.is_passable(from_cell, h_dir):
        return False
    if not grid.is_passable(from_cell, v_dir):
        return False

    return True


def smooth_path_to_diagonals(
    commands: list[MovementCommand],
    grid: MazeGrid,
    start_state: RobotState,
) -> list[MotionSegment]:
    """Transform discrete orthogonal commands into smoothed diagonal and straight segments.

    Detects alternating zigzags and replaces them with high-speed diagonal sprints.
    """
    if not commands:
        return []

    # 1. Reconstruct cell poses
    poses: list[RobotState] = [start_state]
    curr_cell = start_state.cell
    curr_h = start_state.heading

    for cmd in commands:
        if cmd == MovementCommand.FORWARD:
            curr_cell = curr_cell.neighbor(curr_h)
        elif cmd == MovementCommand.TURN_LEFT:
            curr_h = curr_h.turn_left()
        elif cmd == MovementCommand.TURN_RIGHT:
            curr_h = curr_h.turn_right()
        elif cmd == MovementCommand.TURN_AROUND:
            curr_h = curr_h.turn_around()
        poses.append(RobotState(curr_cell, curr_h))

    # 2. Parse command stream into segments
    segments: list[MotionSegment] = []
    cell_size = 0.18
    diagonal_unit = math.sqrt(2) * cell_size

    i = 0
    n = len(commands)

    while i < n:
        cmd = commands[i]

        # Check for diagonal zigzag opportunities:
        # Pattern 1: FORWARD -> TURN_R -> FORWARD -> TURN_L -> FORWARD
        # or Pattern 2: TURN_R -> FORWARD -> TURN_L -> FORWARD
        # Check if cell at i and cell at i+2 form a valid diagonal transition
        if i + 3 < n:
            c1, c2, c3, c4 = commands[i], commands[i + 1], commands[i + 2], commands[i + 3]
            # [TURN_R, FORWARD, TURN_L, FORWARD] or [TURN_L, FORWARD, TURN_R, FORWARD]
            is_right_zigzag = (
                c1 == MovementCommand.TURN_RIGHT
                and c2 == MovementCommand.FORWARD
                and c3 == MovementCommand.TURN_LEFT
                and c4 == MovementCommand.FORWARD
            )
            is_left_zigzag = (
                c1 == MovementCommand.TURN_LEFT
                and c2 == MovementCommand.FORWARD
                and c3 == MovementCommand.TURN_RIGHT
                and c4 == MovementCommand.FORWARD
            )

            if is_right_zigzag or is_left_zigzag:
                # Check clearance for cutting the corner
                start_c = poses[i].cell
                end_c = poses[i + 4].cell
                if is_diagonal_clear(grid, start_c, end_c):
                    entry_turn = (
                        MotionSegmentType.TURN_45_ENTRY_RIGHT
                        if is_right_zigzag
                        else MotionSegmentType.TURN_45_ENTRY_LEFT
                    )
                    segments.append(
                        MotionSegment(
                            segment_type=entry_turn,
                            distance_m=0.5 * math.pi * 0.045,
                            units_count=1,
                        )
                    )
                    segments.append(
                        MotionSegment(
                            segment_type=MotionSegmentType.DIAGONAL_SPRINT,
                            distance_m=diagonal_unit,
                            units_count=1,
                        )
                    )
                    exit_turn = (
                        MotionSegmentType.TURN_45_EXIT_LEFT
                        if is_right_zigzag
                        else MotionSegmentType.TURN_45_EXIT_RIGHT
                    )
                    segments.append(
                        MotionSegment(
                            segment_type=exit_turn,
                            distance_m=0.5 * math.pi * 0.045,
                            units_count=1,
                        )
                    )
                    i += 4
                    continue

        if cmd == MovementCommand.FORWARD:
            # Group consecutive forward moves into a single straight sprint
            fwd_count = 0
            while i < n and commands[i] == MovementCommand.FORWARD:
                fwd_count += 1
                i += 1
            segments.append(
                MotionSegment(
                    segment_type=MotionSegmentType.ORTHOGONAL_STRAIGHT,
                    distance_m=fwd_count * cell_size,
                    units_count=fwd_count,
                )
            )
        elif cmd == MovementCommand.TURN_LEFT:
            segments.append(
                MotionSegment(
                    segment_type=MotionSegmentType.TURN_90_LEFT,
                    distance_m=0.5 * math.pi * 0.09,
                    units_count=1,
                )
            )
            i += 1
        elif cmd == MovementCommand.TURN_RIGHT:
            segments.append(
                MotionSegment(
                    segment_type=MotionSegmentType.TURN_90_RIGHT,
                    distance_m=0.5 * math.pi * 0.09,
                    units_count=1,
                )
            )
            i += 1
        elif cmd == MovementCommand.TURN_AROUND:
            segments.append(
                MotionSegment(
                    segment_type=MotionSegmentType.TURN_180,
                    distance_m=math.pi * 0.09,
                    units_count=1,
                )
            )
            i += 1
        else:
            i += 1

    return segments


def evaluate_smoothed_trajectory_time(
    segments: list[MotionSegment],
    profile: KinematicProfile,
) -> float:
    """Compute physical traversal time for a smoothed trajectory under F1 kinematics."""
    if not segments:
        return 0.0

    total_time = 0.0
    current_v = 0.0
    v_turn = profile.max_cornering_velocity_mps

    for idx, seg in enumerate(segments):
        is_last = idx == len(segments) - 1
        v_target = 0.0 if is_last else v_turn

        if seg.segment_type in (
            MotionSegmentType.ORTHOGONAL_STRAIGHT,
            MotionSegmentType.DIAGONAL_SPRINT,
        ):
            seg_time, current_v = compute_straight_time(
                distance_m=seg.distance_m,
                v_start=current_v,
                v_end=v_target,
                profile=profile,
            )
            total_time += seg_time

        elif seg.segment_type in (
            MotionSegmentType.TURN_90_LEFT,
            MotionSegmentType.TURN_90_RIGHT,
        ):
            arc_len = 0.5 * math.pi * profile.turn_radius_m
            total_time += arc_len / v_turn if v_turn > 0 else 0.15
            current_v = v_turn

        elif seg.segment_type in (
            MotionSegmentType.TURN_45_ENTRY_LEFT,
            MotionSegmentType.TURN_45_ENTRY_RIGHT,
            MotionSegmentType.TURN_45_EXIT_LEFT,
            MotionSegmentType.TURN_45_EXIT_RIGHT,
        ):
            # 45-degree curve takes half the duration of a 90-degree curve
            arc_len = 0.25 * math.pi * profile.turn_radius_m
            total_time += arc_len / v_turn if v_turn > 0 else 0.08
            current_v = v_turn

        elif seg.segment_type == MotionSegmentType.TURN_180:
            arc_len = math.pi * profile.turn_radius_m
            total_time += arc_len / v_turn if v_turn > 0 else 0.3
            current_v = 0.0

    return total_time
