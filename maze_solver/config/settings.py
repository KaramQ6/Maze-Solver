"""Global configuration constants for MMRC26 Micromouse Contest.

All competition rules, dimensions, timing budgets, and algorithmic constants
must be defined here in a single location per MMRC26 Master Plan §7.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class MazeConfig:
    """Immutable configuration holding all MMRC26 rules and dimensions."""

    # Maze Dimensions (Rulebook §5.a)
    MAZE_ROWS: int = 10
    MAZE_COLS: int = 10
    CELL_SIZE_CM: float = 18.0
    WALL_THICKNESS_MM: float = 12.0
    CELL_PITCH_CM: float = CELL_SIZE_CM + WALL_THICKNESS_MM / 10.0
    WALL_HEIGHT_CM: float = 5.0
    LATTICE_POST_MM: float = 12.0

    # Destination Zone: 4 center cells on 10x10 grid (Master Plan §2.1)
    # (4,4), (4,5), (5,4), (5,5)
    GOAL_CELLS: tuple[tuple[int, int], ...] = (
        (4, 4),
        (4, 5),
        (5, 4),
        (5, 5),
    )

    # Timing Budget (Rulebook §2.3.a, §6.1.a)
    MATCH_BUDGET_SECONDS: float = 480.0

    # Robot Physical Footprint Constraints (Rulebook §4.e)
    MAX_ROBOT_LENGTH_CM: float = 25.0
    MAX_ROBOT_WIDTH_CM: float = 25.0

    # Algorithmic Planning Constants (Master Plan §4.2)
    # Cost multiplier for turns compared to straight 1-cell traversal
    TURN_PENALTY: float = 1.5

    # Representation of Infinity for unreachable flood-fill cells
    UNREACHABLE_DISTANCE: int = 9999
