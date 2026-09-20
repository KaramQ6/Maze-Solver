"""Core algorithmic modules for maze representation, flood fill, planning, and strategy."""

from maze_solver.core.floodfill import compute_distance_map, get_next_search_move
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.planner import plan_turn_weighted_path
from maze_solver.core.strategy import (
    MatchPhase,
    MatchResult,
    RunRecord,
    calculate_mmrc26_score,
    can_fit_repeat_run,
)
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RelativeDirection,
    RobotState,
    WallSensations,
)

__all__ = [
    "Cell",
    "Direction",
    "MatchPhase",
    "MatchResult",
    "MazeGrid",
    "MovementCommand",
    "RelativeDirection",
    "RobotState",
    "RunRecord",
    "WallSensations",
    "calculate_mmrc26_score",
    "can_fit_repeat_run",
    "compute_distance_map",
    "get_next_search_move",
    "plan_turn_weighted_path",
]
