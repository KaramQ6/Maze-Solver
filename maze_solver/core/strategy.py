"""Pure strategy and scoring logic for MMRC26 Micromouse Contest.

Strictly decoupled from simulation or hardware runtimes.
Per MMRC26 Official Rulebook §6.3 & Master Plan §1:
  Final Score = (Total Successful Runs / Official Time) * 1000
"""

from dataclasses import dataclass, field
from enum import Enum

from maze_solver.config.settings import MazeConfig


class MatchPhase(Enum):
    """Tournament match phases per Master Plan §4.3."""

    PHASE_A_SEARCH = "Phase A (Search)"
    PHASE_B_DECISION = "Phase B (Decision)"
    PHASE_C_SPEED_RUN = "Phase C (Speed Run)"
    PHASE_D_REPEAT = "Phase D (Repeat)"


@dataclass
class RunRecord:
    """Record of an individual run attempt within the 8-minute match."""

    run_number: int
    phase: MatchPhase
    run_time: float
    success: bool
    commands_count: int


@dataclass
class MatchResult:
    """Comprehensive outcome of an 8-minute MMRC26 match."""

    total_successful_runs: int
    official_time: float
    final_score: float
    total_elapsed_seconds: float
    runs: list[RunRecord] = field(default_factory=list)


def calculate_mmrc26_score(successful_runs: int, official_time: float) -> float:
    """Calculate the official MMRC26 Final Score (§6.3).

    Formula: Final Score = (Total Successful Runs / Official Time) * 1000
    """
    if successful_runs <= 0 or official_time <= 0.0 or official_time == float("inf"):
        return 0.0
    return (successful_runs / official_time) * 1000.0


def can_fit_repeat_run(
    elapsed_time: float,
    speed_run_time: float,
    reposition_time: float = 5.0,
    match_budget: float = MazeConfig.MATCH_BUDGET_SECONDS,
) -> bool:
    """Determine if another repeat run can be completed within the remaining match budget."""
    total_needed = reposition_time + speed_run_time
    return (elapsed_time + total_needed) <= match_budget
