"""Defensive sensor filtering and debouncing engine.

Implements Exponential Moving Average (EMA) smoothing, hysteresis thresholding,
and post-reflection rejection to convert noisy analog IR/ToF sensor readings
into high-confidence WallSensations.
"""

from dataclasses import dataclass
from typing import NamedTuple

from maze_solver.core.types import WallSensations


class RawDistances(NamedTuple):
    """Raw distance readings from physical range sensors in centimeters."""

    front: float
    left: float
    right: float


@dataclass(frozen=True)
class SensorFilterConfig:
    """Tuning parameters for the defensive sensor filter."""

    alpha: float = 0.65
    wall_detect_threshold_cm: float = 12.0
    wall_clear_threshold_cm: float = 15.5
    min_consecutive_cycles: int = 2


class SensorFilter:
    """Filters noisy distance sensor readings with hysteresis and debouncing."""

    def __init__(self, config: SensorFilterConfig | None = None) -> None:
        self.config = config or SensorFilterConfig()

        # Filtered distance state (initialized to wide open)
        self._filtered_front: float = 30.0
        self._filtered_left: float = 30.0
        self._filtered_right: float = 30.0

        # Debounced boolean states
        self._wall_front: bool = False
        self._wall_left: bool = False
        self._wall_right: bool = False

        # Persistence counters for noise debouncing
        self._count_front: int = 0
        self._count_left: int = 0
        self._count_right: int = 0

    def update(self, raw: RawDistances) -> WallSensations:
        """Process a raw sensor cycle and emit debounced WallSensations."""
        alpha = self.config.alpha

        # 1. Update Exponential Moving Average
        self._filtered_front = alpha * raw.front + (1.0 - alpha) * self._filtered_front
        self._filtered_left = alpha * raw.left + (1.0 - alpha) * self._filtered_left
        self._filtered_right = alpha * raw.right + (1.0 - alpha) * self._filtered_right

        # 2. Update individual channels with hysteresis and debounce
        self._wall_front, self._count_front = self._update_channel(
            self._filtered_front, self._wall_front, self._count_front
        )
        self._wall_left, self._count_left = self._update_channel(
            self._filtered_left, self._wall_left, self._count_left
        )
        self._wall_right, self._count_right = self._update_channel(
            self._filtered_right, self._wall_right, self._count_right
        )

        return WallSensations(
            front=self._wall_front,
            left=self._wall_left,
            right=self._wall_right,
        )

    def _update_channel(
        self,
        filtered_val: float,
        current_state: bool,
        consecutive_count: int,
    ) -> tuple[bool, int]:
        """Apply hysteresis and cycle-persistence to a single distance channel."""
        detect_thresh = self.config.wall_detect_threshold_cm
        clear_thresh = self.config.wall_clear_threshold_cm
        min_cycles = self.config.min_consecutive_cycles

        raw_state_candidate = filtered_val < (clear_thresh if current_state else detect_thresh)

        if raw_state_candidate != current_state:
            new_count = consecutive_count + 1
            if new_count >= min_cycles:
                return raw_state_candidate, 0
            return current_state, new_count

        return current_state, 0

    @property
    def filtered_distances(self) -> RawDistances:
        """Return the current smoothed distance estimates."""
        return RawDistances(
            front=self._filtered_front,
            left=self._filtered_left,
            right=self._filtered_right,
        )
