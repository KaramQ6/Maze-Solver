"""Autonomous error detection and physical recovery state machine.

Monitors sensor telemetry and motor encoders to detect:
- Mechanical stall / wheel spin (wheels spinning without physical advance).
- Wheel slip / yaw drift (unexpected rotation during straight-line travel).
- Collision or wall contact.
Provides deterministic recovery action plans to re-center the mouse within the 18cm cell.
"""

from enum import Enum
from typing import NamedTuple

from maze_solver.core.types import MovementCommand


class RecoveryState(Enum):
    """Lifecycle state of the autonomous recovery controller."""

    NORMAL = "NORMAL"
    STALL_DETECTED = "STALL_DETECTED"
    SLIP_DETECTED = "SLIP_DETECTED"
    RECENTERING = "RECENTERING"
    RECOVERED = "RECOVERED"


class TelemetryFrame(NamedTuple):
    """Odometry and sensor observation for a single control step."""

    command: MovementCommand
    encoder_delta_cm: float
    gyro_yaw_rate_dps: float
    front_dist_cm: float
    left_dist_cm: float
    right_dist_cm: float


class RecoveryAction(NamedTuple):
    """Corrective motion primitive to restore localization and center alignment."""

    action_type: str  # "HALT", "REVERSE", "SQUARE_WALL", "RESUME"
    distance_cm: float
    angle_deg: float


class FaultDetector:
    """Detects physical motion anomalies from multi-sensor telemetry."""

    def __init__(self, stall_cycles_threshold: int = 3) -> None:
        self.stall_threshold = stall_cycles_threshold
        self._stall_counter: int = 0
        self._last_front_dist: float | None = None

    def evaluate(self, frame: TelemetryFrame) -> RecoveryState:
        """Evaluate telemetry to identify motor stall, slip, or nominal operation."""
        # 1. Slip Detection: excessive yaw rate during a forward command
        if frame.command == MovementCommand.FORWARD:
            if abs(frame.gyro_yaw_rate_dps) > 120.0:  # Excessive sudden rotational velocity
                self._stall_counter = 0
                return RecoveryState.SLIP_DETECTED

        # 2. Stall Detection: encoders report motion, but front distance doesn't decrease near wall
        if frame.command == MovementCommand.FORWARD and frame.encoder_delta_cm > 1.5:
            if self._last_front_dist is not None:
                dist_change = abs(self._last_front_dist - frame.front_dist_cm)
                # Close to a front wall and not making progress despite encoder movement
                if frame.front_dist_cm < 12.0 and dist_change < 0.3:
                    self._stall_counter += 1
                    if self._stall_counter >= self.stall_threshold:
                        return RecoveryState.STALL_DETECTED
                else:
                    self._stall_counter = 0
            else:
                if frame.front_dist_cm < 12.0:
                    self._stall_counter = 1
                    if self._stall_counter >= self.stall_threshold:
                        return RecoveryState.STALL_DETECTED
            self._last_front_dist = frame.front_dist_cm
        else:
            self._stall_counter = 0
            self._last_front_dist = frame.front_dist_cm

        return RecoveryState.NORMAL


class RecoveryController:
    """Manages the step-by-step recovery and re-centering protocol."""

    def __init__(self) -> None:
        self.state = RecoveryState.NORMAL
        self.fault_detector = FaultDetector()
        self._recovery_step: int = 0

    def process_telemetry(
        self, frame: TelemetryFrame
    ) -> tuple[RecoveryState, RecoveryAction | None]:
        """Process incoming telemetry and produce corrective actions when faults occur."""
        detected = self.fault_detector.evaluate(frame)

        if self.state == RecoveryState.NORMAL:
            if detected != RecoveryState.NORMAL:
                self.state = detected
                self._recovery_step = 1
                return self.state, RecoveryAction(
                    action_type="HALT", distance_cm=0.0, angle_deg=0.0
                )
            return RecoveryState.NORMAL, None

        # Execute multi-step recovery sequence
        if self._recovery_step == 1:
            # Step 1: Back away from the obstacle/contact point
            self._recovery_step = 2
            self.state = RecoveryState.RECENTERING
            return self.state, RecoveryAction(
                action_type="REVERSE", distance_cm=-2.5, angle_deg=0.0
            )

        if self._recovery_step == 2:
            # Step 2: Angular re-alignment using side sensors if available
            lateral_imbalance = frame.left_dist_cm - frame.right_dist_cm
            self._recovery_step = 3
            if abs(lateral_imbalance) > 1.5:
                # Steer slightly toward the more open side to center in the 18cm cell
                corrective_angle = 5.0 if lateral_imbalance > 0 else -5.0
                return self.state, RecoveryAction(
                    action_type="SQUARE_WALL", distance_cm=0.0, angle_deg=corrective_angle
                )
            return self.state, RecoveryAction(
                action_type="SQUARE_WALL", distance_cm=0.0, angle_deg=0.0
            )

        # Step 3: Complete recovery and signal resumption of normal navigation
        self.state = RecoveryState.RECOVERED
        self._recovery_step = 0
        return self.state, RecoveryAction(action_type="RESUME", distance_cm=0.0, angle_deg=0.0)

    def reset(self) -> None:
        """Reset the recovery state machine to nominal operation."""
        self.state = RecoveryState.NORMAL
        self._recovery_step = 0
