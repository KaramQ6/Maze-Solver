# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- Confirm firmware motion and sensor reads before advancing robot pose or banking a run; abort on timeouts, invalid ranges, or incomplete speed plans. Require both wheel encoders to reach the physical cell pitch.
- Constrain speed runs to verified cells and require actual goal arrival in simulation; MMS rejects failed moves and probes the first unknown shortcut cell.
- Bound embedded A* heap to one entry per state, keep planning buffers off the loop stack, and avoid silently truncating long paths; align CI with the active branch and add firmware checks.
- Confirm exploration walls with three readings, sampling twice more only when channels disagree; this reduces permanent mapping errors from transient noise.

### Added
- ESP32-C3 SuperMini/TB6612FNG/MPU6500 profile without encoders, ToF-referenced cell motion, hardware-path regression tests, and an SVG wiring diagram with confirmed/proposed nets.
- Zero-allocation C99 Embedded Firmware library (`firmware/`) with a 42-byte bit-packed 10x10 maze grid, static queue BFS floodfill, bounded turn-weighted A* planner, and kinematics.
- Comprehensive unit test suite in pure C (`test_firmware.c`) verified with MinGW/GCC `-Wall -Wextra -Werror -pedantic`.
- Defensive Sensor Debouncing & Filtering Engine (`sensor_filter.py`) utilizing Exponential Moving Average (EMA) and hysteresis thresholds to reject transient post-reflection noise.
- Autonomous Physical Error Recovery & Re-centering State Machine (`recovery.py`) with motor stall detection, wheel slip detection, reverse back-off, and wall-touch alignment.
- Interactive Web Dashboard (`dashboard/`) with HTML5 Canvas 10x10 maze editor, distance heatmap, orthogonal and Fosbury diagonal path overlays, F1 G-force gauges, and live 480-second tournament match runner.
- Official 16-point MMRC26 Readiness Checklist & Technical Defense Pack (`docs/READINESS_CHECKLIST.md`) auditing all eligibility, robot specifications, competition strategy, and paperwork rules with an interactive audit tab in the web dashboard.
- Physics & Kinematics Engine (`kinematics.py`) modeling real F1 dynamics: trapezoidal acceleration profiles, cornering speed limits ($a_{\text{lat}} = v^2/R$), and vacuum suction downforce ($>3\text{ G}$ lateral grip).
- Return-Trip Exploration Engine (`return_explorer.py`) enabling autonomous secondary mapping from center back to start square to discover long straight corridors.
- Diagonal Traversal & Turn Smoothing Planner (`diagonal_planner.py`) cutting $45^\circ$ across open lattice posts ("Fosbury Flop"), reducing path distance by 29.3% and eliminating stop-and-pivot braking.
- CLI flags `--suction` and `--return-trip` in `runner.py` for tournament simulation comparisons.
- Layer 2 Turn-Weighted State-Space Path Planner (`planner.py`) implementing A* over `(Cell, Direction)` states with admissible Manhattan heuristic and physical turn penalties.
- Pure tournament strategy & scoring module (`strategy.py`) implementing the official MMRC26 scoring formula `(Total Runs / Official Time) * 1000`.
- Match simulation engine (`match_simulator.py`) managing the continuous 480-second window across Phase A (Search), Phase B (Decision), Phase C (Speed Run), and Phase D (Score Maximizing Repeats).
- CLI `--match` tournament execution mode displaying run breakdowns, official times, and final scores.
- Procedural MMRC26-compliant Island-Goal maze generator (`maze_generator.py`) guaranteeing strictly 1 entrance, center wall detachment, and full lattice-point attachment (§5.d).
- Virtual robot simulator (`virtual_mouse.py`) with odometry, physical collision detection, and configurable sensor noise injection.
- Real-time ASCII terminal visualizer (`renderer.py`) displaying walls, robot pose, and flood-fill distance maps.
- Simulation runner (`runner.py`) orchestrating Phase A Search Run execution, tracking match steps, and calculating estimated run times.
- Comprehensive test suite covering maze generator constraints, robot physical interaction, and end-to-end goal reaching.
- Core edge-based maze representation preventing wall desynchronization (`horizontal_walls`, `vertical_walls`).
- Layer 1 base flood-fill algorithm with deterministic heading tie-breaker.
- Initial project scaffold conforming to Engineering OS standards.
