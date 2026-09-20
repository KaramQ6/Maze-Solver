"""Fundamental domain types for the Micromouse maze solver."""

from enum import Enum, IntEnum
from typing import NamedTuple


class Direction(IntEnum):
    """Absolute compass headings for the robot and maze walls."""

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def turn_right(self) -> "Direction":
        """Return heading after a 90-degree clockwise turn."""
        return Direction((self.value + 1) % 4)

    def turn_left(self) -> "Direction":
        """Return heading after a 90-degree counter-clockwise turn."""
        return Direction((self.value - 1) % 4)

    def turn_around(self) -> "Direction":
        """Return heading after a 180-degree turnaround."""
        return Direction((self.value + 2) % 4)

    def opposite(self) -> "Direction":
        """Return the opposite cardinal direction."""
        return self.turn_around()

    @property
    def delta(self) -> tuple[int, int]:
        """Return (row_offset, col_offset) for moving 1 cell in this direction."""
        if self == Direction.NORTH:
            return (-1, 0)
        if self == Direction.EAST:
            return (0, 1)
        if self == Direction.SOUTH:
            return (1, 0)
        return (0, -1)


class RelativeDirection(Enum):
    """Relative sensor or movement directions from the robot's local frame."""

    FRONT = "FRONT"
    RIGHT = "RIGHT"
    BACK = "BACK"
    LEFT = "LEFT"


class Cell(NamedTuple):
    """2D coordinate location within the maze grid (0-indexed)."""

    row: int
    col: int

    def neighbor(self, direction: Direction) -> "Cell":
        """Calculate coordinates of the adjacent cell in the given direction."""
        d_row, d_col = direction.delta
        return Cell(self.row + d_row, self.col + d_col)


class RobotState(NamedTuple):
    """Robot state defining discrete position and heading."""

    cell: Cell
    heading: Direction


class MovementCommand(Enum):
    """Discrete motion primitives sent to the robot drivetrain."""

    FORWARD = "FORWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    TURN_AROUND = "TURN_AROUND"
    HALT = "HALT"


class WallSensations(NamedTuple):
    """Local wall presence detected by onboard distance/IR sensors."""

    front: bool
    left: bool
    right: bool
