# Customized MMS

- Upstream: mackorone/mms at `4ed1df48c5223b6b8b422d6e6cf2569ef47d8d8c` (MIT).
- `source/`, `sdk/`, `build-deps/`, `build/`, and `bin/` are generated, ignored directories.
- Durable source changes live in `patches/mmrc26.patch`; additional C++ files live in `extensions/`.
- Build with `python tools/mms/build_mms.py --setup` on Windows; subsequent builds omit `--setup`.
- Native regression checks: `python tools/mms/build_mms.py --test`.
- Qt 6.8.3 and its matching MinGW 13.1 toolchain are project-local. Do not use the system 32-bit GCC.
- The build uses a temporary `subst` drive and explicit Qt prefix to handle Unicode workspace paths.
- Start coordinates use MMS coordinates: `(0,0)` is bottom-left. Only cardinal headings are supported by the Python solver.
- The native UI owns persistent start/timing settings. `MMS_START_X`, `MMS_START_Y`, `MMS_START_HEADING`, and `MMS_MMRC26=1` are passed to the algorithm at launch.
- `setMotionMode search|speed` is a no-response extension. The bridge sends it only with `MMS_MMRC26=1`.
- Timing values are per-cell/per-90-degree measurements, not CPU specifications. Snapshot duration when a primitive starts; pause does not advance it.
