# MMRC26 Micromouse — Master Plan & Technical Foundation

Competition: Micromouse Maze Solver Contest 2026 (MMRC26)
Organizer: IEEE Robotics and Automation Society (RAS) Student Branch Chapter, Al-Hussein Technical University (HTU)
Source of truth: Official MMRC26 Rulebook v1 (mmrchtu.tech)

---

## 1. Strategic Principle (read this before writing any code)

The entire design should be driven by one formula:

```
Final Score = (Total Successful Runs / Official Time) * 1000
```

Where **Official Time** is the single fastest successful Run Time achieved during the 8-minute Match, and **Total Successful Runs** is every run in that match that reached the destination zone, regardless of how slow it was.

Two consequences follow directly from this, and both should shape the architecture, not just the strategy:

1. **A completed Search Run already counts as a successful run.** The rulebook defines a "Run" as any single continuous attempt from Start Square to Destination Zone (§2.3.b), and a Search Run is explicitly one type of Run (§2.4.b). If your slow, cautious mapping run reaches the center, that is Run #1 on the scoreboard, even though it will almost never be your Official Time. This means the mapping phase is never "wasted" time as long as it succeeds — failure to complete it is the only real loss.

2. **Once you have a verified path, every additional successful repeat of it is a guaranteed, low-risk score increase**, while chasing a faster time only helps if you actually beat your current best AND you don't burn so much match time testing it that you lose repeat-run opportunities. Concretely: going from N to N+1 successful runs multiplies your score by (N+1)/N. Improving Official Time from T to T' multiplies the *entire* score by T/T'. Early on, both matter a lot. Once you have a solid, repeatable path, the marginal, low-risk move is almost always "run it again," not "try to shave off another tenth of a second."

**Design rule derived from this:** build for determinism and repeatability first, raw speed second. A slightly slower robot that never fails a repeat run will outscore a faster robot that only completes one clean run.

---

## 2. Hard Constraints (verbatim facts from the official rulebook — do not deviate)

### 2.1 The Maze
- 10 × 10 grid of unit cells. Each cell is 18 cm × 18 cm measured from the inside of the walls, with 5% tolerance assumed on the physical build.
- Wall thickness: 12 mm. Wall height: 5 cm.
- Lattice points (wall-post intersections): 1.2 cm × 1.2 cm.
- Outer wall fully encloses the entire maze (§5.a) — meaning **every border cell's outward-facing wall is known with certainty before the match starts.** This is a structural guarantee of the competition format itself, not specific knowledge of the disclosed maze, so it is legitimate to hard-code as a default initial state rather than something you must sense.
- Start Square: one of the four corners, bounded on three sides by walls (the fourth side, the only exit, is what you sense/confirm at power-on — do not assume it is always the same relative direction, since the specific corner is only "specified" at event time, per §1). **The algorithm must be start-corner-agnostic**: accept start position and initial heading as run-time inputs, never hardcoded.
- Destination Zone: the 4 center cells. On a 10×10 grid (0-indexed 0–9), that is cells (4,4), (4,5), (5,4), (5,5). It has exactly one entrance (§2.2.d).
- **Island Goal configuration**: the walls forming the center are completely detached from the outer perimeter wall (§2.2.g). The maze is explicitly **not** Simply Connected. The rulebook states directly that wall-hugging algorithms will loop forever and never find the goal (§2.4.a, §5.e). This rules out wall-following as a viable strategy entirely — it is not a design choice, it is a rule constraint.
- At least one wall is attached to every lattice point (§5.d) — a maze-generation guarantee, not something you need to encode.
- Multiple physical paths to the destination are expected (§5.e).
- Wall colors/floor color and friction are explicitly **not guaranteed** (§5.b) — do not tune sensor thresholds or drivetrain calibration against assumed colors or a fixed friction coefficient.

### 2.2 The Robot
- Max 25 cm in length or width. No height restriction.
- Must be fully self-contained: onboard battery and onboard logic only. No remote control, no wireless data tether, no external processing.
- No combustion-based energy source.
- Must not leave any part of its body in the maze, and must not jump, fly, climb, scratch, cut, burn, mark, or damage the maze walls. Any violation is immediate disqualification (§4.f).

### 2.3 The Match & Timing — two separate clocks, do not conflate them
- **Match clock**: one continuous 8-minute (480 second) window per team, starting the moment the contest administrator grants maze access. This clock does not pause between runs, and any time spent repositioning or adjusting the robot between runs is inside this window (§6.1.a, §6.1.b).
- **Run clock**: starts the instant the robot's front edge crosses the start line (the boundary between the start cell and the next cell clockwise), and stops the instant the front edge crosses the finish line (entrance to the destination cell). This is what produces a Run Time.
- A **Touch** (any operator contact with the robot after a run has started) immediately aborts that run. A robot that has already crossed the finish line can be picked up freely without affecting that run's recorded time (§6.1.c).
- After the maze layout is disclosed, no maze information may be manually fed into the robot — only switch/dip-switch configuration changes are allowed. All source code is collected and audited by judges for compliance (§6.1.d). Build clean from day one; do not plan to "clean up" hardcoded maze data later.
- Ambient lighting, temperature, and humidity are explicitly not guaranteed to be consistent — do not assume IR sensor behavior will match your test environment exactly (§6.1.e).

### 2.4 Team & Format
- Solo or team of up to 3.
- Optional 5-minute design presentation before the match, if the schedule allows.
- Tournament structure: Phase 1 Qualifiers (single maze, sequential, standard 8-minute window) → Phase 2 Knockout (single-elimination bracket, two teams racing simultaneously on identical side-by-side mazes).

### 2.5 Scoring & Tie-Breaking
- Final Score = (Total Successful Runs ÷ Official Time) × 1000.
- Robots that enter the center within their 8-minute window always outrank robots that don't, regardless of score formula edge cases.
- Robots that never reach the center are ranked by *remaining distance on the flood-fill distance chart* at the point they stopped (§6.3) — this is worth noting precisely because it means the organizers' own tie-break logic is a literal flood-fill distance map from the goal. This independently confirms flood fill is the intended mental model for this contest, not just a good implementation choice.
- Ties beyond this are decided solely by judges' discretion.

---

## 3. Maze Representation (data structures)

### 3.1 Wall storage — use edge-based storage, not per-cell duplicated flags
A naive per-cell 4-bit wall mask (N/E/S/W) creates a desync risk: a wall between cell (r,c) and (r,c+1) is the *same physical wall*, but if stored independently on both cells' masks, a sensor update on one side can silently disagree with the other side. Store walls as two boundary arrays instead, with each physical wall existing in exactly one place:

```
horizontal_walls[11][10]   # boundary between row r-1 and row r, at column c
vertical_walls[10][11]     # boundary between column c-1 and column c, at row r
```

Derive a per-cell "can I move this direction" check on demand from these arrays. This guarantees there is exactly one source of truth per physical wall.

**Pre-load at initialization** (not sensed, per §2.1 above):
- All four outer boundary rows/columns of both arrays = wall present.
- The 3 known walls around the disclosed start corner.

Everything else starts as "unknown" — treat unknown as *passable* until a sensor reading proves otherwise. This optimistic default is what drives the search run to actually explore rather than freezing on assumed walls, and is the standard convention in flood-fill micromouse implementations.

### 3.2 Distance map
A 10×10 integer array, recomputed via breadth-first search seeded from the 4 goal cells outward, using only edges currently marked passable. Cells with genuinely unknown status but not yet proven blocked are treated as passable for this computation (again, optimistic default).

---

## 4. Algorithm: Two-Layer Modified Flood Fill

### 4.1 Layer 1 — Base Flood Fill (topological distance, used during search)

```
function flood_fill(goal_cells, horizontal_walls, vertical_walls):
    distance = 2D array of size 10x10, filled with infinity
    queue = empty deque

    for cell in goal_cells:
        distance[cell] = 0
        queue.push_back(cell)

    while queue is not empty:
        current = queue.pop_front()
        for neighbor in passable_neighbors(current, horizontal_walls, vertical_walls):
            if distance[neighbor] > distance[current] + 1:
                distance[neighbor] = distance[current] + 1
                queue.push_back(neighbor)

    return distance
```

**Movement policy during search:** at each cell, sense all available walls, update the boundary arrays, and if a newly discovered wall could invalidate the current distance map, recompute it. On a 10×10 grid (100 cells) a full recompute is computationally trivial on any microcontroller (sub-millisecond), so implement the simple full recompute first — do not build the more complex localized/incremental recompute until profiling shows it is actually necessary. Move to the accessible neighbor with the strictly lowest distance value. Define one fixed tie-break rule up front (recommended: prefer continuing in the current heading over turning, since this also reduces physical turns during exploration) — never leave a tie unresolved or randomly broken, since that makes behavior non-deterministic and harder to debug.

### 4.2 Layer 2 — Turn-Weighted Path Planning (used for the speed run)

Plain flood-fill distance treats every cell-to-cell move as equal cost, which gives the topologically shortest path, not the fastest path in time — every turn costs real deceleration/acceleration that a cell-count metric ignores. This is what "Modified Flood Fill" refers to in practice.

To account for this correctly, once the map is fully known, **do not just walk the distance gradient again.** Instead, switch to a weighted search over an expanded state space of `(cell, heading)` pairs — 100 cells × 4 headings = 400 states, trivially small to search exhaustively with Dijkstra or A*:

```
edge_cost(from_state, to_state):
    base = 1.0
    if to_state.heading != from_state.heading:
        base += TURN_PENALTY
    return base
```

Run Dijkstra (or A* with Manhattan distance as the heuristic) over this state graph from `(start_cell, start_heading)` to any state where `cell` is one of the 4 goal cells. The resulting path minimizes real turn-adjusted cost, not raw cell count. `TURN_PENALTY` should be tuned empirically against your actual drivetrain's deceleration/re-acceleration time, not guessed — this is a physical calibration constant, log it in one config location (see §7).

**Diagonal movement:** the rulebook does not mention diagonal traversal at all. Default assumption for v1 is orthogonal-only movement. Do not build diagonal-cutting logic unless you've separately confirmed with the organizers that it's permitted and your chassis geometry can physically execute it.

### 4.3 Phase structure and the exploration decision point

```
Phase A — Initial Search Run
  Goal: reach the center via flood-fill guidance, sensing walls along the way.
  Priority: completion and safety, not speed. This run already banks 1 successful run if it finishes.

Phase B — Decision Point (time-boxed)
  Check: how much of the map is still unknown, and how much Match time remains?
  If a large unexplored region borders your current best path AND there is clear time budget,
  optionally run one supplementary exploration pass to look for a genuinely shorter route.
  Otherwise, skip straight to Phase C. Do not explore "just in case" — every second here is a
  second not spent banking guaranteed successful runs.

Phase C — Speed Run
  Compute the turn-weighted path (§4.2) over the known map and execute it at your tuned max
  safe speed. This produces your Official Time if it beats the search run's time (it will).

Phase D — Repeat for Score
  Re-run the same verified path for the remainder of the Match window. Only deviate from it if
  you have concrete evidence (not speculation) that a faster route exists and you can validate
  it without risking your remaining run budget.
```

**Decision rule for Phase B/D trade-offs:** treat every remaining second as either (a) spent chasing a faster Official Time, which multiplies your *entire* score if successful but is wasted if it fails, or (b) spent banking another guaranteed repeat, which additively increases your score with near-zero risk. Default to (b) unless you have concrete, sensed evidence (not a hunch) that (a) will pay off.

---

## 5. Match Time Budget

Total budget: 480 seconds, starting from maze access grant, including all repositioning time between runs (§6.1.a/b — this is not optional, it is a rule).

Framework (fill in with real numbers once you have bench-test data):

| Segment | Budget | Notes |
|---|---|---|
| Search Run | variable, track live | If this exceeds ~50% of remaining budget without reaching the goal, consider this a red flag — see §9 risk register |
| Manual reposition to start (per run) | must be minimized | Counts against the 480s; if your operator process is slow, it directly reduces your Total Successful Runs count |
| Speed Run (Official Time candidate) | your tuned best | This is your denominator |
| Repeats | remaining budget ÷ (speed run time + reposition time) | This is your primary score lever once a reliable path exists |

Track elapsed Match time in software/logs (even if not sensor-verifiable on the robot itself, verify manually with a stopwatch during practice) so the operator knows in real time whether to keep repeating or force a stop before time runs out mid-run (an incomplete run in progress when time expires does not help you).

---

## 6. Development Roadmap

1. **Simulator first.** Build a Python simulator: 10×10 grid, edge-based wall storage (§3.1), a virtual robot with position/heading state, and virtual sensors with configurable noise/false-positive rate. Do not touch hardware until the algorithm is proven here.
2. **Algorithm core as a pure module.** Implement flood fill (§4.1) and the weighted state-space planner (§4.2) with zero dependency on whether the caller is the simulator or real hardware — the interface should be: sensor readings in, wall-array updates out, next-move commands out.
3. **Adversarial maze testing.** Generate multiple Island Goal mazes, including ones specifically designed to trap a naive wall-follower, and confirm your algorithm solves all of them, not just one easy case.
4. **Sensor noise injection.** Test the algorithm's behavior when the simulator occasionally reports a false wall or misses a real one — this is where recovery logic (§7) gets validated before it matters on real hardware.
5. **Port to hardware**, swapping only the sensor-read/motor-command interface layer, keeping the algorithm core untouched.
6. **Bench-test on a partial physical maze** before attempting a full run.
7. **Instrument every run** — log path taken, per-cell timing, any sensor anomalies — from the first hardware test onward, not just at the competition.

---

## 7. Best Practices

- **Separation of concerns.** The maze-solving logic must not contain a single hardware-specific call. Simulator and real robot both implement the same sensor/motor interface. This is also directly relevant to the rulebook's "Best Code Award" (§7, awards section), which is explicitly judged on elegant logic and clean mapping technique during a mandatory code audit.
- **No hardcoded maze data, anywhere, ever** — the rulebook checks for this explicitly (§6.1.d). Build clean from the first commit.
- **Deterministic decisions only.** Every tie-break, every fallback, must follow one documented fixed rule. No randomness in navigation logic.
- **Defensive sensor handling.** Real IR/wall sensors near cell boundaries will produce false readings. Require a debounce or confidence threshold (e.g., N consecutive consistent readings) before trusting a wall update, and test this specifically with injected noise in the simulator (§6, step 4).
- **Explicit recovery behavior.** Define up front what happens if a sensor reading contradicts the existing map. A silent freeze burns Match time with zero benefit; a defined recovery routine (e.g., re-verify, then trust the fresher reading and re-flood-fill) protects your run count.
- **One config location.** Cell size, wall thickness, TURN_PENALTY, max speed, sensor thresholds — all constants in one file, not scattered through the codebase. Recalibration should be a one-file edit.
- **Modular versioning.** Keep simulator, algorithm core, and hardware driver layer as separate modules so a hardware revision never risks breaking already-validated algorithm logic.
- **Log schema, decided in advance.** At minimum log per run: start timestamp, end timestamp (or failure point), path taken, whether it was a Search or Speed run, and any sensor anomaly flags. This is what makes a failed practice run debuggable instead of mysterious.

---

## 8. Risk Register

| Risk | Impact | Mitigation |
|---|---|---|
| Search run never completes (gets stuck/loops) | Zero runs banked, wastes Match time | Guarantee termination via flood-fill-driven movement only, never random wandering; hard timeout with forced-abort fallback |
| Sensor false positive near cell boundary | Corrupts map, causes bad path decisions | Debounce/confidence threshold before trusting a wall reading (§7) |
| TURN_PENALTY untuned or guessed | Speed run path is "optimal" on paper but slow in reality | Empirically calibrate against real drivetrain deceleration before competition |
| Operator repositioning between runs is slow | Directly reduces Total Successful Runs | Practice and time the reposition process; streamline physical handling |
| Chasing a faster path late in the Match with no concrete evidence | Risk of losing repeat-run time for no score gain | Follow the Phase D decision rule in §4.3 — default to repeating the known-good path |
| Hardcoded or assumed color/friction calibration | Breaks on event-day floor/lighting per §2.1 | Calibrate sensors adaptively at the start of the match, not from home-test constants |
| Any maze data manually fed into code after disclosure | Instant rules violation on code audit | Zero hardcoded maze data from day one (§7) |

---

## 9. Terminology Quick Reference (official rulebook definitions)

| Term | Meaning |
|---|---|
| Match | The full 8-minute window of maze access per team |
| Run | One continuous attempt, Start Square → Destination Zone |
| Run Time | Recorded time for one successful Run |
| Official Time | The single fastest Run Time in the Match |
| Touch | Any operator contact with the robot after a Run has started (aborts that run) |
| Abort | Operator manually ending an active Run |
| Search Run | Initial, typically slower run that builds the internal map |
| Speed Run | Subsequent run using the known map at maximum safe speed |
| Simply Connected Maze | A maze with no closed loops or isolated wall sections (NOT this competition's format) |
| Island Goal | The center walls are fully detached from the outer perimeter (this competition's actual format) |
| Lattice Point | The wall-post intersection at cell corners |

---

## 10. Open Items — confirm before finalizing hardware

- Diagonal movement: not mentioned in the rulebook; confirm with organizers if you plan to rely on it, otherwise build orthogonal-only.
- MCU, motor driver, and sensor hardware selection — this plan is intentionally hardware-agnostic until that decision is made.
- Solo vs. team of up to 3 — affects whether algorithm and hardware/mechanical work can run in parallel.
