"""Kinematics and physics simulation engine for high-speed Micromouse vehicles.

Implements real-world physical dynamics:
- Trapezoidal and triangular velocity profiles (acceleration, cruising, braking).
- Centripetal lateral acceleration limits for cornering (a_lat = v^2 / R).
- Ground effect vacuum suction fan downforce modeling (F_downforce = k_suction * m * g).
- Formula 1 lateral cornering dynamics (up to 3-6 Gs with suction enabled).
"""

import math
from dataclasses import dataclass

from maze_solver.config.settings import MazeConfig
from maze_solver.core.types import MovementCommand

GRAVITY = 9.80665  # m/s^2


@dataclass(frozen=True)
class KinematicProfile:
    """Vehicle dynamic parameters and physical limits."""

    max_velocity_mps: float = 3.5
    max_accel_mps2: float = 12.0
    friction_coefficient: float = 0.8
    suction_multiplier: float = 3.0  # Downforce = multiplier * m * g
    cell_size_m: float = MazeConfig.CELL_PITCH_CM / 100.0
    turn_radius_m: float = 0.09

    @property
    def max_lateral_accel_mps2(self) -> float:
        """Maximum achievable lateral acceleration without sliding."""
        effective_normal_multiplier = 1.0 + self.suction_multiplier
        return self.friction_coefficient * effective_normal_multiplier * GRAVITY

    @property
    def max_cornering_velocity_mps(self) -> float:
        """Maximum cornering velocity through a circular arc turn."""
        max_v_sq = self.max_lateral_accel_mps2 * self.turn_radius_m
        safe_v = math.sqrt(max_v_sq)
        return min(safe_v, self.max_velocity_mps)


def compute_straight_time(
    distance_m: float,
    v_start: float,
    v_end: float,
    profile: KinematicProfile,
) -> tuple[float, float]:
    """Calculate traversal time and peak velocity over a straight corridor.

    Uses a trapezoidal or triangular velocity profile.
    Returns (time_seconds, v_exit).
    """
    if distance_m <= 0.0:
        return 0.0, v_start

    a = profile.max_accel_mps2
    v_max = profile.max_velocity_mps

    # Distance to accelerate from v_start to v_max
    d_accel = (v_max**2 - v_start**2) / (2.0 * a) if v_max > v_start else 0.0
    # Distance to decelerate from v_max to v_end
    d_decel = (v_max**2 - v_end**2) / (2.0 * a) if v_max > v_end else 0.0

    if d_accel + d_decel <= distance_m:
        # Full trapezoidal profile: accelerates to v_max, cruises, decelerates to v_end
        d_cruise = distance_m - (d_accel + d_decel)
        t_accel = (v_max - v_start) / a if v_max > v_start else 0.0
        t_cruise = d_cruise / v_max
        t_decel = (v_max - v_end) / a if v_max > v_end else 0.0
        total_time = t_accel + t_cruise + t_decel
        return total_time, v_end

    # Triangular profile: cannot reach v_max
    v_peak_sq = (2.0 * a * distance_m + v_start**2 + v_end**2) / 2.0
    if v_peak_sq < 0.0:
        v_peak = max(v_start, v_end)
    else:
        v_peak = min(math.sqrt(v_peak_sq), v_max)

    t_accel = max(0.0, (v_peak - v_start) / a)
    t_decel = max(0.0, (v_peak - v_end) / a)
    total_time = t_accel + t_decel
    return total_time, v_end


def compute_turn_time(profile: KinematicProfile) -> float:
    """Calculate time to execute a continuous 90-degree sweeping arc turn."""
    # Arc length for quarter circle (90 degrees): L = (pi / 2) * R
    arc_length = 0.5 * math.pi * profile.turn_radius_m
    v_turn = profile.max_cornering_velocity_mps
    return arc_length / v_turn if v_turn > 0 else 0.2


def evaluate_trajectory_kinematics(
    commands: list[MovementCommand],
    profile: KinematicProfile,
) -> float:
    """Evaluate physical run time for a command sequence using continuous kinematics."""
    if not commands:
        return 0.0

    total_time = 0.0
    current_velocity = 0.0
    v_turn = profile.max_cornering_velocity_mps

    i = 0
    num_commands = len(commands)

    while i < num_commands:
        cmd = commands[i]

        if cmd == MovementCommand.FORWARD:
            # Count contiguous forward moves (straight corridor)
            straight_cells = 0
            while i < num_commands and commands[i] == MovementCommand.FORWARD:
                straight_cells += 1
                i += 1

            corridor_distance = straight_cells * profile.cell_size_m

            # Target exit velocity: if next action is a turn, slow down to v_turn; if at goal, 0.0
            if i < num_commands and commands[i] != MovementCommand.HALT:
                v_target = v_turn
            else:
                v_target = 0.0

            seg_time, current_velocity = compute_straight_time(
                distance_m=corridor_distance,
                v_start=current_velocity,
                v_end=v_target,
                profile=profile,
            )
            total_time += seg_time

        elif cmd in (MovementCommand.TURN_LEFT, MovementCommand.TURN_RIGHT):
            turn_time = compute_turn_time(profile)
            total_time += turn_time
            current_velocity = v_turn
            i += 1

        elif cmd == MovementCommand.TURN_AROUND:
            # 180-degree turnaround takes twice a 90-degree turn
            turn_time = 2.0 * compute_turn_time(profile)
            total_time += turn_time
            current_velocity = 0.0
            i += 1

        else:
            i += 1

    return total_time
