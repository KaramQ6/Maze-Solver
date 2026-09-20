"""Core algorithmic modules for maze representation, flood fill, planning, and strategy."""

from maze_solver.core.diagonal_planner import (
    MotionSegment,
    MotionSegmentType,
    evaluate_smoothed_trajectory_time,
    is_diagonal_clear,
    smooth_path_to_diagonals,
)
from maze_solver.core.floodfill import compute_distance_map, get_next_search_move
from maze_solver.core.kinematics import (
    KinematicProfile,
    compute_straight_time,
    compute_turn_time,
    evaluate_trajectory_kinematics,
)
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.planner import plan_turn_weighted_path
from maze_solver.core.return_explorer import (
    compute_return_distance_map,
    get_next_return_move,
    run_return_trip,
)
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
    "KinematicProfile",
    "MatchPhase",
    "MatchResult",
    "MazeGrid",
    "MotionSegment",
    "MotionSegmentType",
    "MovementCommand",
    "RelativeDirection",
    "RobotState",
    "RunRecord",
    "WallSensations",
    "calculate_mmrc26_score",
    "can_fit_repeat_run",
    "compute_distance_map",
    "compute_return_distance_map",
    "compute_straight_time",
    "compute_turn_time",
    "evaluate_smoothed_trajectory_time",
    "evaluate_trajectory_kinematics",
    "get_next_return_move",
    "get_next_search_move",
    "is_diagonal_clear",
    "plan_turn_weighted_path",
    "run_return_trip",
    "smooth_path_to_diagonals",
]
