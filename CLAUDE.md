# Maze-Solver — Project Guidelines

## Mission & Competition Scope
- **Event**: Micromouse Maze Solver Contest 2026 (MMRC26) hosted by IEEE RAS HTU Student Branch Chapter.
- **Score Formula**: `Final Score = (Total Successful Runs / Official Time) * 1000`
- **Total Match Window**: 480 continuous seconds. Timer never pauses between runs.
- **Maze Specs**: 10x10 cells, 18 cm x 18 cm inside dimension, 1.2 cm wall thickness, 5 cm wall height.
- **Goal**: 4 center cells `(4,4), (4,5), (5,4), (5,5)` with exactly one entrance (Island Goal, detached from outer walls).
- **Rule Constraints**: No wall-hugging algorithms (leads to infinite loop). No hardcoded internal maze walls.

## Tech Stack & Commands
- **Language**: Python 3.11+
- **Commands**:
  - Run Tests: `pytest -v`
  - Check Linter: `ruff check .`
  - Format Check: `ruff format --check .`
  - Format Apply: `ruff format .`
  - Typecheck: `mypy maze_solver tests`

## Architecture Rules
- Edge-based storage (`horizontal_walls[11][10]`, `vertical_walls[10][11]`) guarantees single source of truth for walls.
- Decoupled pure logic: `maze_solver/core/` contains no hardware dependencies, IO side-effects, or random logic.
- Deterministic decisions: Tie-breaks prioritize maintaining current heading.
- Strategy: Complete search run (banks Run #1) -> Compute turn-weighted path -> Repeat verified path to maximize score.
