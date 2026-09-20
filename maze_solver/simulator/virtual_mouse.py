"""Virtual Micromouse simulator tracking physical pose, sensing, and odometry."""

import random

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import (
    Cell,
    Direction,
    MovementCommand,
    RobotState,
    WallSensations,
)


class VirtualMouse:
    """Simulates the physical robot within the ground-truth maze environment."""

    def __init__(
        self,
        start_cell: Cell,
        initial_heading: Direction,
        noise_rate: float = 0.0,
        seed: int | None = None,
    ) -> None:
        self.pose = RobotState(start_cell, initial_heading)
        self.noise_rate = noise_rate
        self.collision_count: int = 0
        self._rng = random.Random(seed)

    def sense_walls(self, ground_truth: MazeGrid) -> WallSensations:
        """Query ground-truth maze for walls facing front, left, and right."""
        current_cell = self.pose.cell
        heading = self.pose.heading

        front_wall = ground_truth.has_wall(current_cell, heading)
        left_wall = ground_truth.has_wall(current_cell, heading.turn_left())
        right_wall = ground_truth.has_wall(current_cell, heading.turn_right())

        # Inject sensor false positives/negatives if noise_rate > 0
        if self.noise_rate > 0.0:
            if self._rng.random() < self.noise_rate:
                front_wall = not front_wall
            if self._rng.random() < self.noise_rate:
                left_wall = not left_wall
            if self._rng.random() < self.noise_rate:
                right_wall = not right_wall

        return WallSensations(front=front_wall, left=left_wall, right=right_wall)

    def apply_command(self, command: MovementCommand, ground_truth: MazeGrid) -> bool:
        """Execute a discrete movement command.

        Returns True if command executed successfully, False on collision.
        """
        if command == MovementCommand.HALT:
            return True

        if command == MovementCommand.FORWARD:
            target_cell = self.pose.cell.neighbor(self.pose.heading)
            if ground_truth.has_wall(
                self.pose.cell, self.pose.heading
            ) or not ground_truth.is_valid_cell(target_cell):
                self.collision_count += 1
                return False

            self.pose = RobotState(target_cell, self.pose.heading)
            return True

        if command == MovementCommand.TURN_LEFT:
            self.pose = RobotState(self.pose.cell, self.pose.heading.turn_left())
            return True

        if command == MovementCommand.TURN_RIGHT:
            self.pose = RobotState(self.pose.cell, self.pose.heading.turn_right())
            return True

        if command == MovementCommand.TURN_AROUND:
            self.pose = RobotState(self.pose.cell, self.pose.heading.turn_around())
            return True

        return False
