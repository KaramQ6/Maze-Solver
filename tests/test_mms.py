import io
import os
from unittest.mock import patch

import pytest

from maze_solver.core.maze_grid import MazeGrid
from maze_solver.core.types import Cell, Direction, RobotState
from maze_solver.simulator.maze_generator import generate_island_maze
from mms_main import MMS_API, MMSResetRequested, read_mms_start, run_mms_solver


@pytest.fixture(autouse=True)
def clear_mms_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    for key in ("MMS_START_X", "MMS_START_Y", "MMS_START_HEADING", "MMS_MMRC26"):
        monkeypatch.delenv(key, raising=False)


def test_mms_move_failure_is_not_accepted_as_progress() -> None:
    with patch.object(MMS_API, "_command", return_value="crash"):
        with pytest.raises(RuntimeError, match="crash"):
            MMS_API.move_forward()


def test_default_pose_works_with_unmodified_mms() -> None:
    assert read_mms_start(10, 10) == RobotState(Cell(9, 0), Direction.NORTH)
    with patch.object(MMS_API, "_command") as command:
        MMS_API.set_motion_mode("speed")
        MMS_API.check_reset()
    command.assert_not_called()


@pytest.mark.parametrize("heading", ["n", "e", "s", "w"])
def test_selected_pose_uses_bottom_left_coordinates(heading: str) -> None:
    with patch.dict(os.environ, MMS_START_X="7", MMS_START_Y="2", MMS_START_HEADING=heading):
        pose = read_mms_start(10, 12)
    assert pose == RobotState(Cell(9, 7), Direction("nesw".index(heading)))


@pytest.mark.parametrize(
    "values",
    [
        {"MMS_START_X": "1"},
        {"MMS_START_X": "-1", "MMS_START_Y": "0", "MMS_START_HEADING": "n"},
        {"MMS_START_X": "10", "MMS_START_Y": "0", "MMS_START_HEADING": "e"},
        {"MMS_START_X": "0", "MMS_START_Y": "10", "MMS_START_HEADING": "w"},
        {"MMS_START_X": "0", "MMS_START_Y": "0", "MMS_START_HEADING": "ne"},
        {"MMS_START_X": "1.5", "MMS_START_Y": "0", "MMS_START_HEADING": "n"},
    ],
)
def test_invalid_start_is_rejected(values: dict[str, str]) -> None:
    with patch.dict(os.environ, values), pytest.raises(ValueError, match="MMS start"):
        read_mms_start(10, 10)


def test_motion_mode_does_not_read_an_acknowledgement() -> None:
    with patch.dict(os.environ, MMS_MMRC26="1"), patch.object(MMS_API, "_command") as command:
        MMS_API.set_motion_mode("speed")
    command.assert_called_once_with(["setMotionMode", "speed"])


def test_reset_acknowledged_before_another_movement() -> None:
    with (
        patch.dict(os.environ, MMS_MMRC26="1"),
        patch.object(MMS_API, "_command", side_effect=[True, "ack"]) as command,
        pytest.raises(MMSResetRequested),
    ):
        MMS_API.move_forward()
    assert [call.args[0] for call in command.call_args_list] == [["wasReset"], ["ackReset"]]


@pytest.mark.parametrize("response", ["", "unexpected\n"])
def test_protocol_eof_and_invalid_boolean_are_errors(response: str) -> None:
    with (
        patch("sys.stdin", io.StringIO(response)),
        patch("sys.stdout", io.StringIO()),
        pytest.raises(RuntimeError),
    ):
        MMS_API.wall_left()


class SimulatedProtocol:
    """Exercise the whole bridge while independently tracking its physical mouse."""

    def __init__(self, grid: MazeGrid, start: RobotState) -> None:
        self.grid = grid
        self.start = start
        self.pose = start
        self.mode = "search"
        self.modes: list[str] = []
        self.speed_starts: list[RobotState] = []
        self.reset_pending = False
        self.reset_after_first_move = False
        self.reset_count = 0

    def command(self, args: list[object], return_type: object = None) -> object:
        name = args[0]
        if name == "mazeWidth":
            return self.grid.cols
        if name == "mazeHeight":
            return self.grid.rows
        if name == "wasReset":
            return self.reset_pending
        if name == "ackReset":
            self.pose = self.start
            self.reset_pending = False
            self.reset_count += 1
            return "ack"
        if name == "setMotionMode":
            self.mode = str(args[1])
            self.modes.append(self.mode)
            if self.mode == "speed":
                self.speed_starts.append(self.pose)
            return None
        heading = self.pose.heading
        sensor_headings = {
            "wallFront": heading,
            "wallLeft": heading.turn_left(),
            "wallRight": heading.turn_right(),
            "wallBack": heading.opposite(),
        }
        if name in sensor_headings:
            return self.grid.has_wall(self.pose.cell, sensor_headings[str(name)])
        if name == "moveForward":
            assert not self.grid.has_wall(self.pose.cell, heading), "Physical mouse hit a wall"
            self.pose = RobotState(self.pose.cell.neighbor(heading), heading)
            if self.reset_after_first_move:
                self.reset_pending = True
                self.reset_after_first_move = False
            return "ack"
        if name in ("turnLeft", "turnRight"):
            next_heading = heading.turn_left() if name == "turnLeft" else heading.turn_right()
            self.pose = RobotState(self.pose.cell, next_heading)
            return "ack"
        assert name in ("setColor", "setText", "setWall")
        return None


@pytest.mark.parametrize(
    "start", [RobotState(Cell(0, 9), Direction.SOUTH), RobotState(Cell(7, 3), Direction.WEST)]
)
def test_solver_returns_and_speed_runs_from_selected_pose(start: RobotState) -> None:
    protocol = SimulatedProtocol(generate_island_maze(seed=17), start)
    with (
        patch.dict(
            os.environ,
            MMS_MMRC26="1",
            MMS_START_X=str(start.cell.col),
            MMS_START_Y=str(9 - start.cell.row),
            MMS_START_HEADING="nesw"[start.heading],
        ),
        patch.object(MMS_API, "_command", side_effect=protocol.command),
        patch.object(MMS_API, "log"),
    ):
        run_mms_solver()
    assert protocol.speed_starts == [start]
    assert protocol.mode == "speed"
    assert protocol.pose.cell in {Cell(4, 4), Cell(4, 5), Cell(5, 4), Cell(5, 5)}


def test_solver_discards_old_pose_after_reset() -> None:
    start = RobotState(Cell(0, 9), Direction.SOUTH)
    protocol = SimulatedProtocol(generate_island_maze(seed=17), start)
    protocol.reset_after_first_move = True
    with (
        patch.dict(
            os.environ, MMS_MMRC26="1", MMS_START_X="9", MMS_START_Y="9", MMS_START_HEADING="s"
        ),
        patch.object(MMS_API, "_command", side_effect=protocol.command),
        patch.object(MMS_API, "log"),
    ):
        run_mms_solver()
    assert protocol.reset_count == 1
    assert protocol.modes == ["search", "search", "speed"]
    assert protocol.speed_starts == [start]
