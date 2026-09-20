"""ASCII terminal renderer for the 10x10 Micromouse maze environment."""

from maze_solver.config.settings import MazeConfig
from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import Cell, Direction, RobotState


def render_maze_state(
    grid: MazeGrid,
    robot_state: RobotState | None = None,
    distance_map: list[list[int]] | None = None,
    show_distances: bool = False,
) -> str:
    """Render an ASCII representation of the 10x10 maze grid.

    Posts are represented by '+'.
    Horizontal walls are '---', open is '   '.
    Vertical walls are '|', open is ' '.
    Robot heading is rendered as '^', '>', 'v', '<'.
    Center goal cells are marked with '[G]' or '[0]'.
    """
    lines: list[str] = []
    goal_set = set(MazeConfig.GOAL_CELLS)

    for r in range(grid.rows):
        # 1. Render North horizontal wall row
        h_line = ""
        for c in range(grid.cols):
            h_line += "+"
            if grid.horizontal_walls[r][c]:
                h_line += "---"
            else:
                h_line += "   "
        h_line += "+"
        lines.append(h_line)

        # 2. Render Cell contents and vertical walls
        c_line = ""
        for c in range(grid.cols):
            # Left vertical wall
            if grid.vertical_walls[r][c]:
                c_line += "|"
            else:
                c_line += " "

            cell = Cell(r, c)
            # Cell body
            if robot_state and robot_state.cell == cell:
                arrows = {
                    Direction.NORTH: " ^ ",
                    Direction.EAST: " > ",
                    Direction.SOUTH: " v ",
                    Direction.WEST: " < ",
                }
                c_line += arrows[robot_state.heading]
            elif (r, c) in goal_set:
                if show_distances and distance_map:
                    c_line += f" {distance_map[r][c]:1d} "
                else:
                    c_line += " * "
            elif show_distances and distance_map:
                dist = distance_map[r][c]
                if dist >= MazeConfig.UNREACHABLE_DISTANCE:
                    c_line += " X "
                else:
                    c_line += f"{dist:3d}"
            else:
                c_line += "   "

        # Far right vertical wall
        if grid.vertical_walls[r][grid.cols]:
            c_line += "|"
        else:
            c_line += " "
        lines.append(c_line)

    # 3. Render final South horizontal wall row
    h_line = ""
    for c in range(grid.cols):
        h_line += "+"
        if grid.horizontal_walls[grid.rows][c]:
            h_line += "---"
        else:
            h_line += "   "
    h_line += "+"
    lines.append(h_line)

    return "\n".join(lines)
