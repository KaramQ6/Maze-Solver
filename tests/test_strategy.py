"""Unit tests for MMRC26 Match Strategy and Scoring Formula."""

import pytest

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.strategy import MatchPhase, calculate_mmrc26_score
from maze_solver.core.types import Cell, Direction, MovementCommand
from maze_solver.simulator.match_simulator import MatchSimulator
from maze_solver.simulator.maze_generator import generate_island_maze
from maze_solver.simulator.virtual_mouse import VirtualMouse


def test_scoring_formula_conforms_to_rulebook_examples() -> None:
    """Verify formula: Final Score = (Total Runs / Official Time) * 1000 (§6.3)."""
    # Example 1 from rulebook §6.3:
    # 4 runs, fastest 25 seconds -> (4 / 25) * 1000 = 160.0
    assert calculate_mmrc26_score(4, 25.0) == pytest.approx(160.0, rel=1e-3)

    # Example 2 from rulebook §6.3:
    # 1 run, fastest 18 seconds -> (1 / 18) * 1000 = 55.55...
    assert calculate_mmrc26_score(1, 18.0) == pytest.approx(55.555, rel=1e-2)

    # Zero runs or invalid time -> 0.0
    assert calculate_mmrc26_score(0, 10.0) == 0.0
    assert calculate_mmrc26_score(4, 0.0) == 0.0


def test_full_match_run_execution() -> None:
    """Verify that a full 480-second match executes through all phases and maximizes score."""
    maze = generate_island_maze(seed=42)
    sim = MatchSimulator(
        ground_truth=maze,
        start_cell=Cell(9, 0),
        initial_heading=Direction.NORTH,
        match_budget_seconds=480.0,
        reposition_time_seconds=3.0,
    )

    result = sim.run_match()

    # Match must complete successfully
    assert result.total_successful_runs >= 2
    assert result.official_time > 0.0
    assert result.final_score > 0.0
    assert result.total_elapsed_seconds <= 480.0

    # Verify phases
    phases = [r.phase for r in result.runs]
    assert MatchPhase.PHASE_A_SEARCH in phases
    assert MatchPhase.PHASE_C_SPEED_RUN in phases
    if len(result.runs) > 2:
        assert MatchPhase.PHASE_D_REPEAT in phases

    # Speed run official time must be faster than search run
    search_time = result.runs[0].run_time
    speed_time = result.runs[1].run_time
    assert speed_time < search_time
    assert result.official_time <= speed_time


def test_full_match_with_kinematics_and_return_trip() -> None:
    """Verify match simulation with F1 kinematics, suction downforce, and return-trip mapping."""
    from maze_solver.core.kinematics import KinematicProfile

    maze = generate_island_maze(seed=42)
    profile = KinematicProfile(suction_multiplier=3.0, max_velocity_mps=4.0)

    sim = MatchSimulator(
        ground_truth=maze,
        start_cell=Cell(9, 0),
        initial_heading=Direction.NORTH,
        match_budget_seconds=480.0,
        reposition_time_seconds=3.0,
        enable_return_trip=True,
        kinematic_profile=profile,
    )

    result = sim.run_match()
    assert result.total_successful_runs >= 2
    assert result.official_time > 0.0
    assert result.final_score > 0.0
    assert result.total_elapsed_seconds <= 480.0


def test_planned_run_only_succeeds_at_goal() -> None:
    maze = MazeGrid()
    sim = MatchSimulator(maze)
    mouse = VirtualMouse(Cell(9, 0), Direction.NORTH)

    success, _ = sim._execute_planned_run([MovementCommand.FORWARD], mouse)

    assert success is False


def test_failed_run_still_consumes_time_for_completed_moves() -> None:
    maze = MazeGrid()
    maze.set_wall(Cell(8, 0), Direction.NORTH)
    sim = MatchSimulator(maze)
    mouse = VirtualMouse(Cell(9, 0), Direction.NORTH)

    success, elapsed = sim._execute_planned_run(
        [MovementCommand.FORWARD, MovementCommand.FORWARD], mouse
    )

    assert not success
    assert elapsed == pytest.approx(sim.speed_cell_time)
    assert mouse.pose.cell == Cell(8, 0)


@pytest.mark.parametrize("seed", [0, 2, 4])
def test_speed_run_reuses_verified_route(seed: int) -> None:
    maze = generate_island_maze(seed=seed)
    sim = MatchSimulator(maze)
    result = sim.run_match()

    assert result.runs[0].success
    assert result.runs[1].success
    assert result.total_successful_runs >= 2


def test_search_finishing_after_match_deadline_is_not_banked() -> None:
    sim = MatchSimulator(generate_island_maze(seed=42), match_budget_seconds=0.1)

    result = sim.run_match()

    assert result.total_successful_runs == 0
    assert result.runs[0].success is False
