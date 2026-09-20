"""Edge-based maze grid representation preventing wall desynchronization.

Per MMRC26 Master Plan §3.1, walls are stored on shared boundaries:
  horizontal_walls[11][10] -> boundary between row r-1 and row r, at column c
  vertical_walls[10][11]   -> boundary between col c-1 and col c, at row r
"""

from maze_solver.config.settings import MazeConfig
from maze_solver.core.types import (
    Cell,
    Direction,
    RobotState,
    WallSensations,
)


class MazeGrid:
    """Represents the 10x10 maze layout using edge-based single-source-of-truth storage."""

    def __init__(self, rows: int = MazeConfig.MAZE_ROWS, cols: int = MazeConfig.MAZE_COLS) -> None:
        self.rows = rows
        self.cols = cols

        # horizontal_walls[r][c] is the wall between (r-1, c) and (r, c)
        # Dimensions: (rows + 1) x cols -> 11 x 10
        self.horizontal_walls: list[list[bool]] = [
            [False for _ in range(self.cols)] for _ in range(self.rows + 1)
        ]

        # vertical_walls[r][c] is the wall between (r, c-1) and (r, c)
        # Dimensions: rows x (cols + 1) -> 10 x 11
        self.vertical_walls: list[list[bool]] = [
            [False for _ in range(self.cols + 1)] for _ in range(self.rows)
        ]

    def is_valid_cell(self, cell: Cell) -> bool:
        """Check if cell is within the maze grid boundaries."""
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def has_wall(self, cell: Cell, direction: Direction) -> bool:
        """Check if a wall exists adjacent to cell in the specified direction."""
        if not self.is_valid_cell(cell):
            return True

        if direction == Direction.NORTH:
            return self.horizontal_walls[cell.row][cell.col]
        if direction == Direction.SOUTH:
            return self.horizontal_walls[cell.row + 1][cell.col]
        if direction == Direction.WEST:
            return self.vertical_walls[cell.row][cell.col]
        if direction == Direction.EAST:
            return self.vertical_walls[cell.row][cell.col + 1]

        return True

    def set_wall(self, cell: Cell, direction: Direction, present: bool = True) -> bool:
        """Set wall state on the shared boundary.

        Returns True if the wall state was modified.
        """
        if not self.is_valid_cell(cell):
            return False

        changed = False
        if direction == Direction.NORTH:
            if self.horizontal_walls[cell.row][cell.col] != present:
                self.horizontal_walls[cell.row][cell.col] = present
                changed = True
        elif direction == Direction.SOUTH:
            if self.horizontal_walls[cell.row + 1][cell.col] != present:
                self.horizontal_walls[cell.row + 1][cell.col] = present
                changed = True
        elif direction == Direction.WEST:
            if self.vertical_walls[cell.row][cell.col] != present:
                self.vertical_walls[cell.row][cell.col] = present
                changed = True
        elif direction == Direction.EAST:
            if self.vertical_walls[cell.row][cell.col + 1] != present:
                self.vertical_walls[cell.row][cell.col + 1] = present
                changed = True

        return changed

    def is_passable(self, cell: Cell, direction: Direction) -> bool:
        """Return True if robot can traverse from cell in direction without hitting a wall."""
        if not self.is_valid_cell(cell):
            return False
        if self.has_wall(cell, direction):
            return False
        target = cell.neighbor(direction)
        return self.is_valid_cell(target)

    def passable_neighbors(self, cell: Cell) -> list[tuple[Cell, Direction]]:
        """Return all adjacent cells that can be traversed from cell."""
        neighbors: list[tuple[Cell, Direction]] = []
        for direction in Direction:
            if self.is_passable(cell, direction):
                neighbors.append((cell.neighbor(direction), direction))
        return neighbors

    def initialize_competition_defaults(
        self, start_corner: Cell, initial_heading: Direction
    ) -> None:
        """Initialize known physical constraints per MMRC26 Master Plan §2.1 & §3.1.

        - Outer perimeter is 100% enclosed by solid walls.
        - Start square is enclosed on 3 sides, with only one open exit (initial_heading).
        - All interior walls start as passable (optimistic default).
        """
        # Outer boundary walls (North and South)
        for col in range(self.cols):
            self.horizontal_walls[0][col] = True
            self.horizontal_walls[self.rows][col] = True

        # Outer boundary walls (West and East)
        for row in range(self.rows):
            self.vertical_walls[row][0] = True
            self.vertical_walls[row][self.cols] = True

        # Start square walls (bounded on 3 sides, exit in initial_heading)
        for direction in Direction:
            if direction != initial_heading:
                self.set_wall(start_corner, direction, present=True)

    def update_from_sensations(self, state: RobotState, sensations: WallSensations) -> bool:
        """Update grid from robot's local sensor readings.

        Returns True if any new wall was detected.
        """
        heading = state.heading
        changed = False

        # Front wall
        if sensations.front and self.set_wall(state.cell, heading, True):
            changed = True

        # Left wall
        if sensations.left and self.set_wall(state.cell, heading.turn_left(), True):
            changed = True

        # Right wall
        if sensations.right and self.set_wall(state.cell, heading.turn_right(), True):
            changed = True

        return changed
