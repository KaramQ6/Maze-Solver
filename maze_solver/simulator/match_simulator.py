"""Simulation environment orchestrating the full 480-second MMRC26 tournament match."""

from maze_solver.config.settings import MazeConfig
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
    RobotState,
)
from maze_solver.simulator.virtual_mouse import VirtualMouse


class MatchSimulator:
    """Simulates a complete 8-minute competition match with Phase A, B, C, and D."""

    def __init__(
        self,
        ground_truth: MazeGrid,
        start_cell: Cell | None = None,
        initial_heading: Direction = Direction.NORTH,
        match_budget_seconds: float = MazeConfig.MATCH_BUDGET_SECONDS,
        reposition_time_seconds: float = 5.0,
        speed_cell_time: float = 0.12,
        turn_penalty: float = MazeConfig.TURN_PENALTY,
    ) -> None:
        if start_cell is None:
            start_cell = Cell(9, 0)
        self.ground_truth = ground_truth
        self.start_cell = start_cell
        self.initial_heading = initial_heading
        self.match_budget_seconds = match_budget_seconds
        self.reposition_time_seconds = reposition_time_seconds
        self.speed_cell_time = speed_cell_time
        self.turn_penalty = turn_penalty

        self.discovered_grid = MazeGrid(ground_truth.rows, ground_truth.cols)
        self.discovered_grid.initialize_competition_defaults(start_cell, initial_heading)

        self.elapsed_time: float = 0.0
        self.runs: list[RunRecord] = []
        self.official_time: float = float("inf")
        self.successful_runs: int = 0

    def _execute_planned_run(
        self, commands: list[MovementCommand], mouse: VirtualMouse
    ) -> tuple[bool, float]:
        """Execute a planned command sequence and compute execution time."""
        cell_moves = 0
        turn_moves = 0
        for cmd in commands:
            if cmd == MovementCommand.FORWARD:
                cell_moves += 1
            else:
                turn_moves += 1

            ok = mouse.apply_command(cmd, self.ground_truth)
            if not ok:
                return False, 0.0

        run_time = (cell_moves * self.speed_cell_time) + (
            turn_moves * self.speed_cell_time * self.turn_penalty
        )
        return True, run_time

    def run_match(self) -> MatchResult:
        """Execute full match simulation through all phases."""
        from maze_solver.simulator.runner import run_search

        run_count = 0

        # ==========================================================
        # Phase A: Initial Search Run (Explore & Bank Run #1)
        # ==========================================================
        run_count += 1
        search_sim = run_search(
            ground_truth=self.ground_truth,
            start_cell=self.start_cell,
            initial_heading=self.initial_heading,
            max_steps=500,
        )

        self.elapsed_time += search_sim.run_time_estimate
        if search_sim.success:
            self.successful_runs += 1
            self.official_time = search_sim.run_time_estimate
            self.runs.append(
                RunRecord(
                    run_number=run_count,
                    phase=MatchPhase.PHASE_A_SEARCH,
                    run_time=search_sim.run_time_estimate,
                    success=True,
                    commands_count=search_sim.total_steps,
                )
            )
            # Reconstruct discovered knowledge from search path
            for state in search_sim.path:
                sensations = VirtualMouse(state.cell, state.heading).sense_walls(self.ground_truth)
                self.discovered_grid.update_from_sensations(state, sensations)
        else:
            self.runs.append(
                RunRecord(
                    run_number=run_count,
                    phase=MatchPhase.PHASE_A_SEARCH,
                    run_time=search_sim.run_time_estimate,
                    success=False,
                    commands_count=search_sim.total_steps,
                )
            )
            return MatchResult(
                total_successful_runs=0,
                official_time=0.0,
                final_score=0.0,
                total_elapsed_seconds=self.elapsed_time,
                runs=self.runs,
            )

        # Reposition robot to start corner
        self.elapsed_time += self.reposition_time_seconds

        # ==========================================================
        # Phase B: Decision & Turn-Weighted Path Planning
        # ==========================================================
        speed_path = plan_turn_weighted_path(
            grid=self.discovered_grid,
            start_state=RobotState(self.start_cell, self.initial_heading),
            goals=MazeConfig.GOAL_CELLS,
            turn_penalty=self.turn_penalty,
        )

        if not speed_path:
            return MatchResult(
                total_successful_runs=self.successful_runs,
                official_time=self.official_time,
                final_score=calculate_mmrc26_score(self.successful_runs, self.official_time),
                total_elapsed_seconds=self.elapsed_time,
                runs=self.runs,
            )

        # ==========================================================
        # Phase C: Speed Run (Candidate for Official Time)
        # ==========================================================
        run_count += 1
        speed_mouse = VirtualMouse(self.start_cell, self.initial_heading)
        ok, speed_run_time = self._execute_planned_run(speed_path, speed_mouse)

        if ok:
            self.successful_runs += 1
            self.official_time = min(self.official_time, speed_run_time)
            self.elapsed_time += speed_run_time
            self.runs.append(
                RunRecord(
                    run_number=run_count,
                    phase=MatchPhase.PHASE_C_SPEED_RUN,
                    run_time=speed_run_time,
                    success=True,
                    commands_count=len(speed_path),
                )
            )
        else:
            self.runs.append(
                RunRecord(
                    run_number=run_count,
                    phase=MatchPhase.PHASE_C_SPEED_RUN,
                    run_time=0.0,
                    success=False,
                    commands_count=len(speed_path),
                )
            )
            return MatchResult(
                total_successful_runs=self.successful_runs,
                official_time=self.official_time,
                final_score=calculate_mmrc26_score(self.successful_runs, self.official_time),
                total_elapsed_seconds=self.elapsed_time,
                runs=self.runs,
            )

        # ==========================================================
        # Phase D: Repeat Verified Path for Maximum Score
        # ==========================================================
        while can_fit_repeat_run(
            self.elapsed_time,
            speed_run_time,
            self.reposition_time_seconds,
            self.match_budget_seconds,
        ):
            self.elapsed_time += self.reposition_time_seconds
            run_count += 1
            repeat_mouse = VirtualMouse(self.start_cell, self.initial_heading)
            rep_ok, rep_time = self._execute_planned_run(speed_path, repeat_mouse)

            if rep_ok:
                self.successful_runs += 1
                self.elapsed_time += rep_time
                self.runs.append(
                    RunRecord(
                        run_number=run_count,
                        phase=MatchPhase.PHASE_D_REPEAT,
                        run_time=rep_time,
                        success=True,
                        commands_count=len(speed_path),
                    )
                )
            else:
                break

        return MatchResult(
            total_successful_runs=self.successful_runs,
            official_time=self.official_time,
            final_score=calculate_mmrc26_score(self.successful_runs, self.official_time),
            total_elapsed_seconds=self.elapsed_time,
            runs=self.runs,
        )
