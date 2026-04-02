# Project Research Summary

**Project:** Swingup Competition Scoring Script
**Domain:** MATLAB post-hoc grading tool — SDI file parsing, signal alignment, metric computation, leaderboard generation
**Researched:** 2026-04-02
**Confidence:** HIGH

## Executive Summary

This project is a one-shot, instructor-run MATLAB scoring script for a 5-team inverted pendulum swingup competition. Teams submit `.mldatx` Simulation Data Inspector session files; each file contains two runs — a hardware run logged at 2 kHz via the STEVAL-EDUKIT01 and a simulation run from the team's digital twin. The scoring script must load each file, identify the two runs, let the scorer map signals interactively, compute swingup time and SMAPE accuracy metrics, apply the rubric (competitive ranking for 4 stepper teams, absolute threshold scoring for 1 BLDC team), and export a leaderboard table to CSV/Excel. The entire toolchain is native MATLAB/Simulink with no additional toolboxes; the `Simulink.sdi.*` API is the only supported mechanism for reading `.mldatx` files.

The recommended approach is a single Live Script `.m` file (`scripts/score_competition.m`) with local functions at the bottom — matching the project's existing script conventions. A top-level `cfg` struct holds all thresholds. Five pure-computation local functions (`load_team_data`, `score_attempt`, `aggregate_team`, `rank_stepper`, `score_bldc`) do all numeric work; one interactive function (`pick_signals`) handles signal selection via `listdlg`. The critical path is: correct signal selection drives start-time detection, which drives time alignment, which drives both swingup time and SMAPE. An error at any earlier stage corrupts all later metrics silently.

The dominant risks are all about silent wrong answers, not crashes. SDI session contamination across team files (no `clear` before `load`), swapped hardware/simulation run identification, unhandled angle wrapping at ±π, and a naively chosen start-time threshold can each produce plausible-looking but incorrect scores. These pitfalls are all preventable with defensive checks and are fully documented in PITFALLS.md. The tool needs no GUI, no database, no automated signal detection without confirmation, and no simulation re-runs — keeping complexity proportionate to the one-event-per-year usage context.

## Key Findings

### Recommended Stack

The entire tool runs on the MATLAB/Simulink stack already present in the project (R2025b). The `Simulink.sdi.*` API is the load/query path for `.mldatx` files — there is no file-level parser alternative. Signal arithmetic uses plain `double` arrays extracted from `timeseries` objects via `.Values.Data` / `.Values.Time`; the timetable pipeline is unnecessary overhead for this use case. `interp1(..., 'linear')` aligns simulation signals onto the 2 kHz hardware time grid; `'previous'` interpolation should be used for ZOH command signals. `table` + `writetable` handles leaderboard construction and CSV/Excel export; `tiledlayout` + `exportgraphics` handles per-attempt comparison figures.

**Core technologies:**

- `Simulink.sdi.load` / `getAllRunIDs` / `getRun` / `getSignalByIndex`: read `.mldatx` and enumerate runs and signals — the only supported `.mldatx` interface
- `Simulink.sdi.clear('Export', false)`: mandatory session isolation before each file load to prevent run ID cross-contamination
- `Simulink.sdi.Run.getLatest()`: identify the simulation run (most recently created); the other run is hardware
- `interp1(t_sim, y_sim, t_hw, 'linear', NaN)`: resample simulation signal onto hardware time grid for SMAPE
- `table` / `sortrows` / `writetable`: leaderboard construction and CSV/Excel export
- `tiledlayout` / `nexttile` / `exportgraphics`: multi-panel figures per attempt (R2019b+ standard)

No additional toolboxes are needed beyond Simulink (base).

### Expected Features

**Must have (table stakes):**

- Load `.mldatx` via SDI API with session isolation — without `clear` before each `load`, runs from different teams bleed together silently
- Separate hardware run from simulation run within each file — mixing them corrupts every downstream metric
- Manual signal selection with smart sort — signal names vary between teams; auto-detection without confirmation is explicitly out of scope
- Start-time detection via first sustained non-zero accel/torque command — establishes the common t=0 for hardware and simulation
- Time alignment of hardware and simulation signals to common t=0 — SMAPE on unaligned signals is meaningless
- Swingup success check: q2 crosses ±π AND holds within ±2° for >= 1 second — per PROJECT.md rubric
- Participation check: |q2| > π/2 sustained for at least 10 samples — 1-point threshold
- Swingup time computation — primary ranking metric for stepper teams
- SMAPE computation on q2 (hardware vs simulation) — primary accuracy metric
- Configurable SMAPE window: `'fixed'` / `'angle'` / `'swingup'` — three modes per PROJECT.md
- Best-per-metric aggregation across multiple attempts per team
- Stepper team competitive ranking (time: 2/1/0.5/0 pt; SMAPE: 2/1/0.5/0.5 pt)
- BLDC team absolute SMAPE scoring (0-40%=4pt, 40-80%=3pt, 80-120%=2pt, 120-160%=1pt, >160%=0pt)
- Participation point (1pt, all teams)
- Final MATLAB table + CSV/Excel export

**Should have (instructor value):**

- Overlay plot: hardware q2 vs simulation q2 per attempt with SMAPE window shading — instant visual sanity check
- Signal preview before confirmation — prevents the most likely scoring error (wrong channel selection)
- Per-team diagnostic printout — makes grades defensible to students
- Config struct at top of script — reusable next year without code edits
- Graceful skip on corrupt/unloadable files with `try/catch` and status logging

**Defer permanently:**

- Leaderboard summary bar chart — marginal value over the printed table for a 5-team event
- Interpolation mode selection (linear default is sufficient for smooth q2 signals)
- GUI (App Designer), database storage, real-time scoring, web dashboard — disproportionate for one event per year

### Architecture Approach

The tool uses layered function decomposition inside a single Live Script entry point (`scripts/score_competition.m`), with all helper functions as local functions at the bottom of the same file. This is idiomatic for the project (matches `RRpendulum_swingup_controldesign.m`), keeps scoring logic co-located with narrative annotations, and avoids path management overhead. A `cfg` struct defined in Section 1 is the single source of truth for all thresholds; it is passed by value to every helper function with no side effects. Interactive signal selection (`pick_signals` via `listdlg`) is the only function with UI side effects; all numeric computation is in pure functions callable in isolation.

**Major components:**

1. **`cfg` struct (inline, Section 1)** — all thresholds, paths, team names, motor types; single place to modify between grading sessions
2. **`load_team_data()`** — `Simulink.sdi.clear` + `load`, enumerate runs, identify hw vs sim by name heuristic + signal density + `getLatest()` fallback; returns `{hwRun, simRun}` pairs
3. **`pick_signals()`** — interactive `listdlg` for accel_cmd and q2 selection with smart keyword sort; runs per attempt inside the team loop
4. **`score_attempt()`** — pure computation: extract arrays, detect start time with hysteresis threshold, align to t=0, check participation, check swingup success, compute swingup time, resample simulation onto hardware grid, compute SMAPE in configured window; returns `attempt` struct
5. **`aggregate_team()`** — reduce N attempt structs to best-per-metric team struct
6. **`rank_stepper()` / `score_bldc()`** — points assignment; these two functions are the only divergence between motor types
7. **`make_table()` / `plot_attempts()` / `export_csv()`** — output generation

### Critical Pitfalls

1. **SDI session contamination (CRITICAL)** — `Simulink.sdi.load` appends to the existing session; without `Simulink.sdi.clear` before each load, run IDs from different teams coexist and indexing breaks silently. Prevention: always call `clear` immediately before `load`; assert `numel(runIDs) == 2` after loading.

2. **Archive/recent run order not guaranteed (CRITICAL)** — `getAllRunIDs` ordering depends on the `setAppendRunToTop` preference at export time, which teams control. Assuming index 1 = hardware is fragile. Prevention: use `Run.getLatest()` to identify the simulation run; cross-check by comparing signal sample density (hardware at 2 kHz vs simulation at coarser Ts).

3. **Angle wrapping at ±π inflates SMAPE (CRITICAL)** — hardware and simulation may independently report the upright position as +π or -π; `|π - (-π)| = 2π` produces a massive per-sample error. Prevention: compute angular difference as `mod(hw - sim + pi, 2*pi) - pi` per sample, or apply `unwrap` + mean offset correction within the stabilization window only.

4. **SMAPE division by zero near q2 = 0 (CRITICAL)** — samples where both signals are near zero produce `NaN` that propagates into the mean. Prevention: restrict SMAPE window to after the detected start time; exclude samples where `|hw| + |sim| < epsilon` (1e-3 rad) from the mean.

5. **Start-time threshold fires too early on noise (CRITICAL)** — a fixed `> 0` threshold mis-fires on sensor noise, shifting every downstream metric. Prevention: use hysteresis — start event is first sample where `|cmd| > 5% of peak command` for at least 5 consecutive samples. Make threshold configurable.

6. **`timeseries` object arithmetic silently auto-resamples (MODERATE)** — `hw_ts - sim_ts` merges time grids without warning, hiding the interpolation. Prevention: always extract `.Data` and `.Time` arrays explicitly; never subtract `timeseries` objects directly.

7. **Hardware signal data is `single` precision (MODERATE)** — Embedded Coder targets output `single`; cast to `double` immediately after extraction with `double(ts.Data)`.

## Implications for Roadmap

Based on the dependency chain identified in FEATURES.md and the build order from ARCHITECTURE.md, the natural phase structure is four phases following strict data-flow dependencies.

### Phase 1: SDI Loading and Run Identification

**Rationale:** Everything downstream depends on correctly loading a `.mldatx` and identifying which run is hardware and which is simulation. This is the foundation; build and validate it first with a real team file before writing any metric logic.

**Delivers:** `load_team_data()` function that reliably returns `{hwRun, simRun}` pairs for any well-formed submission file; assertion on run count; session isolation pattern.

**Addresses:** Table stakes features 1-2 (load `.mldatx`, separate hw/sim runs).

**Avoids:** Pitfall 1 (SDI session contamination), Pitfall 2 (wrong run order), Pitfall 10 (single precision — cast to double here).

**Research flag:** Standard — SDI API is well-documented with HIGH confidence sources. No phase research needed.

---

### Phase 2: Interactive Signal Selection and Time Alignment

**Rationale:** Signal selection is the highest error-leverage point in the entire pipeline. A wrong accel_cmd pick corrupts start-time detection, which corrupts time alignment, which corrupts both swingup time and SMAPE. Validate the interactive UX and alignment logic with real data before building metric computation.

**Delivers:** `pick_signals()` with smart sort and `listdlg` confirmation; `score_attempt()` partial — start-time detection via hysteresis threshold, time alignment to t=0; participation check; signal overlay plot for visual validation.

**Addresses:** Table stakes features 3-4-5 (signal selection, start-time detection, time alignment); differentiator features (signal preview, overlay plot).

**Avoids:** Pitfall 5 (start-time noise threshold), Pitfall 7 (spurious swingup crossings), Pitfall 12 (participation check wrap artifacts).

**Research flag:** Standard — `listdlg` and `interp1` are core MATLAB. No phase research needed.

---

### Phase 3: Metric Computation (Swingup Time and SMAPE)

**Rationale:** The two primary metrics are the highest-risk numerical logic. Build and validate them independently using synthetic timeseries before integrating with the interactive pipeline. This is explicitly the recommendation from ARCHITECTURE.md (start with `score_attempt` using synthetic data).

**Delivers:** Complete `score_attempt()` — swingup success detection with sustained 1-second window, swingup time computation, SMAPE with configurable window, angle-difference formula for wrapping, epsilon guard for zero-denominator samples, `interp1` resampling with time-span clipping.

**Addresses:** Table stakes features 6-10 (swingup success, participation, swingup time, SMAPE, SMAPE window modes).

**Avoids:** Pitfall 3 (SMAPE division by zero), Pitfall 4 (angle wrapping), Pitfall 6 (interp1 NaN out-of-range), Pitfall 7 (multiple crossings), Pitfall 8 (ZOH interpolation for command signals), Pitfall 9 (timeseries auto-resampling), Pitfall 11 (unwrap on large oscillations).

**Research flag:** Standard — math is deterministic; all MATLAB functions are core with HIGH confidence. No phase research needed.

---

### Phase 4: Scoring, Aggregation, and Output

**Rationale:** Once metrics are validated, the scoring rubric application and output generation are straightforward. The stepper/BLDC fork is the only divergence point and it is a simple conditional.

**Delivers:** `aggregate_team()`, `rank_stepper()`, `score_bldc()`, `make_table()`, `export_csv()`, `plot_attempts()`. Final leaderboard table exported to CSV and Excel. Per-team diagnostic printout. Config struct finalized with all thresholds.

**Addresses:** Table stakes features 11-15 (best-per-metric aggregation, stepper ranking, BLDC absolute scoring, participation point, table export); differentiator features (diagnostic printout, attempt validity flags, config struct, graceful skip on corrupt files).

**Avoids:** Pitfall 13 (non-ASCII team names in file paths — use scorer-provided IDs, not `run.Name`), Pitfall 14 (SMAPE range misunderstood — document formula and 0-200% range in output).

**Research flag:** Standard — `table`, `sortrows`, `writetable`, `tiledlayout`, `exportgraphics` are well-documented core MATLAB. No phase research needed.

---

### Phase Ordering Rationale

- **Strict data-flow dependency:** Each phase's output is a direct input to the next. Loading without run identification produces nothing useful; signal selection without loading is impossible; metrics without alignment are meaningless; scoring without metrics cannot run.
- **Error isolation strategy:** Building phases in this order means at the end of Phase 2 the scorer can visually validate alignment before any scoring happens. A wrong alignment is caught before it silently corrupts grades.
- **Architecture supports incremental build:** `score_attempt()` is designed in ARCHITECTURE.md to be callable with synthetic data, which allows Phase 3 to be developed and tested before the Phase 2 interactive pieces are complete.
- **All four phases have standard, well-documented patterns.** No phase requires external API research or toolbox-specific investigation. The full stack is native MATLAB/Simulink.

### Research Flags

Phases with standard patterns (no phase research needed):

- **Phase 1:** SDI API is the documented standard for `.mldatx`; behavior is fully covered in MathWorks official docs.
- **Phase 2:** `listdlg`, `interp1`, and signal alignment are standard MATLAB. Hysteresis threshold pattern is standard DSP.
- **Phase 3:** SMAPE is mathematical; `interp1`, `unwrap`, and angle arithmetic are core MATLAB.
- **Phase 4:** `table`, `writetable`, `tiledlayout`, `exportgraphics` are core MATLAB with no versioning concerns above R2020a (project uses R2025b).

No phases require `/gsd:research-phase` during planning.

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | All APIs are official MathWorks documented functions; `Simulink.sdi.*` confirmed as only `.mldatx` interface; no toolbox ambiguity |
| Features | HIGH | Requirements sourced directly from PROJECT.md and codebase analysis; scoring rubric numbers are exact |
| Architecture | HIGH | Local-function Live Script pattern matches existing project scripts; data flow dependencies are unambiguous |
| Pitfalls | HIGH | SDI behavioral pitfalls from official docs; math pitfalls from first principles; angle wrapping and interp1 NaN behavior confirmed from MathWorks documentation |

**Overall confidence:** HIGH

### Gaps to Address

- **Run discrimination heuristic needs validation with a real team file:** The name-pattern and signal-density approach for identifying hw vs sim run is MEDIUM confidence because archive status is not a first-class API property. The `getLatest()` + `setdiff` pattern should be validated against at least one actual `.mldatx` file before the script is used for scoring.

- **Accel/torque threshold scale is team-specific:** The default `5% of peak command` threshold is a starting value. The scorer should inspect the command signal for at least one attempt per team before the competition to confirm it is not set too high (misses start) or too low (fires on noise). Expose in `cfg` and document in the script.

- **SMAPE window choice affects grade ordering:** All three window modes (`'fixed'`, `'angle'`, `'swingup'`) are implemented but the fairest choice for this specific competition is not definitively resolved from requirements alone. The `'swingup'` default (score only until the pendulum is caught) is most defensible, but if any team achieves swingup without meeting the 1-second hold criterion, `'angle'` becomes the appropriate fallback for their SMAPE. This policy decision should be confirmed with the instructor before the competition day.

## Sources

### Primary (HIGH confidence)

- `C:/Users/u0130154/MATLAB/projects/digtwin_labo/.planning/PROJECT.md` — scoring rubric, feature requirements
- [Simulink.sdi.load — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.load.html) — load behavior and append mode
- [Simulink.sdi.Run — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.run.html) — run properties, `getLatest`
- [Simulink.sdi.Signal — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.signal.html) — signal extraction, `Values` timeseries
- [Simulink.sdi.getAllRunIDs — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.getallrunids.html) — run enumeration
- [Simulink.sdi.clear — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.clear.html) — session isolation
- [writetable — MathWorks](https://www.mathworks.com/help/matlab/ref/writetable.html) — CSV/Excel export
- [tiledlayout — MathWorks](https://www.mathworks.com/help/matlab/ref/tiledlayout.html) — figure layout
- [exportgraphics — MathWorks](https://www.mathworks.com/help/matlab/ref/exportgraphics.html) — figure export
- [interp1 — MathWorks](https://www.mathworks.com/help/matlab/ref/interp1.html) — resampling, NaN out-of-range behavior
- [unwrap — MathWorks](https://www.mathworks.com/help/matlab/ref/unwrap.html) — angle unwrapping caveats

### Secondary (MEDIUM confidence)

- [Simulink.sdi.Run.getAllSignals — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.run.getallsignals.html) — confirmed via search (direct page returned 403 during research)
- [Simulink.sdi.setAppendRunToTop — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.setappendruntotop.html) — run ordering behavior; explains why position-based identification is fragile
- [Symmetric Mean Absolute Percentage Error — Wikipedia](https://en.wikipedia.org/wiki/Symmetric_mean_absolute_percentage_error) — formula and 0-200% range confirmation

### Tertiary (LOW confidence)

- Community pattern: `getLatest()` + `setdiff` for hw/sim discrimination — follows logically from documented `getLatest` behavior but has not been validated against an actual two-run `.mldatx` file in this project

---
*Research completed: 2026-04-02*
*Ready for roadmap: yes*
