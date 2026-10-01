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
- **Customized native MMS**: build workflow and extension contracts are owned by `tools/mms/CLAUDE.md`; user setup is in `docs/MMS_SETUP.md`.
- **Commands**:
  - Run Tests: `pytest -v`
  - Check Linter: `ruff check maze_solver tests mms_main.py convert_mazes.py tools/register_mazes_in_mms.py`
  - Format Check: `ruff format --check maze_solver tests mms_main.py convert_mazes.py tools/register_mazes_in_mms.py`
  - Format Apply: `ruff format maze_solver tests mms_main.py convert_mazes.py tools/register_mazes_in_mms.py`
  - Typecheck: `mypy maze_solver tests`
  - C99 Firmware Tests (GNU Make/GCC): `make -C firmware test`
  - ESP32-C3 Build (PlatformIO): `pio run -e esp32_c3_supermini`
  - C3 hardware regression tests: `make -C firmware test-c3`

## Architecture Rules
- Edge-based storage (`horizontal_walls[11][10]`, `vertical_walls[10][11]`) guarantees single source of truth for walls.
- Decoupled pure logic: `maze_solver/core/` contains no hardware dependencies, IO side-effects, or random logic.
- Deterministic decisions: Tie-breaks prioritize maintaining current heading.
- Strategy: Complete search run (banks Run #1) -> Compute turn-weighted path -> Repeat verified path to maximize score.
- Embedded motion and sensor calls return success/failure; update pose or credit a run only after confirmed movement and goal arrival.
- ESP32 speed runs must use visited cells; an unvisited goal cell is not a verified entrance.
- Physical cell-center pitch is 192 mm (180 mm interior + 12 mm wall), distinct from the interior cell size.
- Simulation only credits trajectories the mouse executes; diagonal geometry is not an ESP32 motion primitive.
- Team hardware is C3 SuperMini/TB6612/MPU6500/3x VL53L0X without encoders. Follow docs/HARDWARE_C3.md; motor pins are confirmed, sensor pins proposed. No timed-motion success without measured displacement.
