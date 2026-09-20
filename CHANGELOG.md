# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial project scaffold conforming to Engineering OS standards.
- Core edge-based maze representation preventing wall desynchronization (`horizontal_walls`, `vertical_walls`).
- Layer 1 base flood-fill algorithm with deterministic heading tie-breaker.
- Unit test suite for maze grid, boundaries, and flood-fill navigation.
- GitHub Actions CI workflow for linting, typechecking, and tests.
