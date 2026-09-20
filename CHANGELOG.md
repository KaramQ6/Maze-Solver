# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Procedural MMRC26-compliant Island-Goal maze generator (`maze_generator.py`) guaranteeing strictly 1 entrance, center wall detachment, and full lattice-point attachment (§5.d).
- Virtual robot simulator (`virtual_mouse.py`) with odometry, physical collision detection, and configurable sensor noise injection.
- Real-time ASCII terminal visualizer (`renderer.py`) displaying walls, robot pose, and flood-fill distance maps.
- Simulation runner (`runner.py`) orchestrating Phase A Search Run execution, tracking match steps, and calculating estimated run times.
- Comprehensive test suite covering maze generator constraints, robot physical interaction, and end-to-end goal reaching.
- Core edge-based maze representation preventing wall desynchronization (`horizontal_walls`, `vertical_walls`).
- Layer 1 base flood-fill algorithm with deterministic heading tie-breaker.
- Initial project scaffold conforming to Engineering OS standards.
