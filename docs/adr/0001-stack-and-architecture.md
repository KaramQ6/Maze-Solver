# 0001 — Algorithmic Core Decoupling and Python 3.11 Architecture

Date: 2026-09-20 · Status: accepted

## Context
The Micromouse Maze Solver Contest 2026 (MMRC26) requires a robot to navigate a 10x10 maze under an 8-minute match timer with strict physical and scoring constraints. We need an algorithmic foundation that allows rapid algorithm development, visualization, noise simulation, and adversarial testing, while ensuring the logic can be ported without modification to embedded C/C++ running on microcontrollers (e.g. STM32, RP2040, ESP32). Furthermore, the competition awards a "Best Code Award" evaluated through mandatory code audit.

## Decision
We chose a pure, decoupled architecture implemented in Python 3.11 for the core algorithm and simulator:
1. All core data structures use static-dimensional edge arrays (`horizontal_walls[11][10]`, `vertical_walls[10][11]`) instead of dynamic graph objects, perfectly mirroring microcontroller memory buffers.
2. The algorithmic core (`maze_solver/core/`) accepts purely typed domain data (`Cell`, `Direction`, wall perceptions) and emits deterministic movement commands (`MovementCommand`). It has zero hardware or GUI dependencies.
3. Tests and strict typing (`mypy --strict`, `ruff`) enforce memory layout constraints and algorithmic determinism.

## Consequences
- **Easier**: Rapid prototyping, exhaustive simulation against hundreds of generated mazes, automated CI verification, and guaranteed reproducibility.
- **Harder**: To maintain 1:1 portability to embedded microcontrollers, core algorithms must avoid idiomatic Python features that do not map directly to C (such as dynamic dictionary lookups in hot loops, unbounded recursion, or heavy object allocations).
- **Exit Path**: The core algorithm logic translates 1:1 into static C structs and arrays for bare-metal firmware.
