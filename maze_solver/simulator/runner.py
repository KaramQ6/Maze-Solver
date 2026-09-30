"""Simulation runner executing MMRC26 maze exploration and match runs."""

import argparse
import sys
from dataclasses import dataclass, field

from maze_solver.config.settings import MazeConfig
from maze_solver.core.floodfill import (
    compute_distance_map,
    get_next_search_move,
)
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RobotState,
)
from maze_solver.simulator.maze_generator import generate_island_maze
from maze_solver.simulator.renderer import render_maze_state
from maze_solver.simulator.virtual_mouse import VirtualMouse


@dataclass
class SimulationResult:
    """Detailed summary of a completed simulation run."""

    success: bool
    total_steps: int
    cell_moves: int
    turn_moves: int
    run_time_estimate: float
    visited_cells: set[Cell] = field(default_factory=set)
    path: list[RobotState] = field(default_factory=list)
    remaining_distance_to_goal: int = 0


def run_search(
    ground_truth: MazeGrid,
    start_cell: Cell | None = None,
    initial_heading: Direction = Direction.NORTH,
    max_steps: int = 500,
    render: bool = False,
    noise_rate: float = 0.0,
    seed: int | None = None,
) -> SimulationResult:
    """Execute Phase A: Initial Search/Mapping Run.

    The robot explores the unknown maze using Base Flood Fill,
    sensing walls at each step, updating its internal map, and
    navigating towards the center island goal.
    """
    if start_cell is None:
        start_cell = Cell(9, 0)

    # 1. Initialize robot's internal knowledge (optimistic defaults per Master Plan §3.1)
    discovered_grid = MazeGrid(ground_truth.rows, ground_truth.cols)
    discovered_grid.initialize_competition_defaults(start_cell, initial_heading)

    # 2. Initialize virtual mouse in the physical maze
    mouse = VirtualMouse(start_cell, initial_heading, noise_rate=noise_rate, seed=seed)

    visited_cells: set[Cell] = {start_cell}
    path: list[RobotState] = [mouse.pose]
    cell_moves: int = 0
    turn_moves: int = 0
    goal_set = set(MazeConfig.GOAL_CELLS)

    # Base timing calibration (Master Plan §4.2):
    # Base cell forward time ~ 0.25s, turn time = 0.25s * TURN_PENALTY
    cell_time = 0.25
    turn_time = cell_time * MazeConfig.TURN_PENALTY

    for step in range(max_steps):
        current_pose = mouse.pose
        visited_cells.add(current_pose.cell)

        # A. Sense physical walls
        sensations = mouse.sense_confirmed_walls(ground_truth)

        # B. Update internal map
        discovered_grid.update_from_sensations(current_pose, sensations)

        # C. Recompute flood fill distance map (exact topological shortest path)
        distance_map = compute_distance_map(discovered_grid, MazeConfig.GOAL_CELLS)

        # D. Check goal arrival
        if (current_pose.cell.row, current_pose.cell.col) in goal_set:
            if render:
                print(f"\n[GOAL REACHED] in {step} steps!")
                print(render_maze_state(discovered_grid, current_pose, distance_map))
            time_estimate = (cell_moves * cell_time) + (turn_moves * turn_time)
            return SimulationResult(
                success=True,
                total_steps=step,
                cell_moves=cell_moves,
                turn_moves=turn_moves,
                run_time_estimate=time_estimate,
                visited_cells=visited_cells,
                path=path,
                remaining_distance_to_goal=0,
            )

        # E. Determine next command
        command, _ = get_next_search_move(
            current_pose,
            discovered_grid,
            distance_map,
            MazeConfig.GOAL_CELLS,
            visited_cells=visited_cells,
        )

        if command == MovementCommand.HALT:
            break

        if render and step % 10 == 0:
            print(
                f"\nStep {step} | Cmd: {command.value} | "
                f"Cell: ({current_pose.cell.row}, {current_pose.cell.col})"
            )
            print(
                render_maze_state(discovered_grid, current_pose, distance_map, show_distances=False)
            )

        # F. Apply command to virtual mouse
        if command == MovementCommand.FORWARD:
            cell_moves += 1
        else:
            turn_moves += 1

        success = mouse.apply_command(command, ground_truth)
        if not success:
            # Physical collision occurred
            break

        path.append(mouse.pose)

    # If loop ended without reaching goal
    final_dist_map = compute_distance_map(ground_truth, MazeConfig.GOAL_CELLS)
    rem_dist = final_dist_map[mouse.pose.cell.row][mouse.pose.cell.col]
    time_estimate = (cell_moves * cell_time) + (turn_moves * turn_time)

    return SimulationResult(
        success=False,
        total_steps=max_steps,
        cell_moves=cell_moves,
        turn_moves=turn_moves,
        run_time_estimate=time_estimate,
        visited_cells=visited_cells,
        path=path,
        remaining_distance_to_goal=rem_dist,
    )


def main() -> None:
    """CLI entrypoint to run simulation."""
    parser = argparse.ArgumentParser(description="MMRC26 Micromouse Simulation Runner")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for maze generation")
    parser.add_argument("--render", action="store_true", help="Render step-by-step ASCII view")
    parser.add_argument("--max-steps", type=int, default=500, help="Maximum simulation steps")
    parser.add_argument(
        "--match", action="store_true", help="Run full 480s match with Phase D repeats"
    )
    parser.add_argument(
        "--suction", action="store_true", help="Enable vacuum suction fan & F1 kinematics"
    )
    parser.add_argument(
        "--return-trip", action="store_true", help="Enable autonomous return-trip mapping"
    )
    args = parser.parse_args()

    print(f"Generating MMRC26 Island-Goal maze (seed={args.seed})...")
    ground_truth = generate_island_maze(seed=args.seed)

    if args.match:
        from maze_solver.core.kinematics import KinematicProfile
        from maze_solver.simulator.match_simulator import MatchSimulator

        profile = None
        if args.suction:
            print("Engaging Vacuum Suction Fan (3.0x downforce, max 4.5 m/s, 15 m/s^2)...")
            profile = KinematicProfile(
                suction_multiplier=3.0,
                max_velocity_mps=4.5,
                max_accel_mps2=15.0,
            )

        print("Executing Full 8-Minute Tournament Match (Phases A, B, C, D)...")
        mgr = MatchSimulator(
            ground_truth=ground_truth,
            enable_return_trip=args.return_trip,
            kinematic_profile=profile,
        )
        match_result = mgr.run_match()

        print("\n=== MMRC26 Match Tournament Results ===")
        print(f"Total Successful Runs : {match_result.total_successful_runs}")
        print(f"Official Time (Best)  : {match_result.official_time:.2f}s")
        print(f"Final Score           : {match_result.final_score:.2f}")
        print(f"Match Time Consumed   : {match_result.total_elapsed_seconds:.1f}s / 480.0s")
        print("\nRun Breakdown:")
        for r in match_result.runs:
            status = "SUCCESS" if r.success else "FAILED"
            print(
                f"  Run #{r.run_number:02d}: {r.phase.value:<20} | "
                f"Time: {r.run_time:5.2f}s | {status}"
            )
        return

    print("Executing Search Run (Layer 1 Base Flood Fill)...")
    result = run_search(
        ground_truth=ground_truth,
        render=args.render,
        max_steps=args.max_steps,
        seed=args.seed,
    )

    print("\n--- Simulation Summary ---")
    print(f"Result: {'SUCCESS' if result.success else 'FAILED'}")
    print(
        f"Total Steps: {result.total_steps} "
        f"({result.cell_moves} forward, {result.turn_moves} turns)"
    )
    print(f"Estimated Run Time: {result.run_time_estimate:.2f}s")
    print(f"Unique Cells Visited: {len(result.visited_cells)} / 100")
    if not result.success:
        print(f"Remaining Distance: {result.remaining_distance_to_goal}")
        sys.exit(1)


if __name__ == "__main__":
    main()
