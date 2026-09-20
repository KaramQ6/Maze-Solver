"""Verification test proving test harness operation."""

from maze_solver import __version__


def test_harness_sanity() -> None:
    """Verify test harness and package version loading."""
    assert __version__ == "0.1.0"
