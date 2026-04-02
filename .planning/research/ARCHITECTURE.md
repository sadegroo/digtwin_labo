# Architecture Patterns

**Domain:** MATLAB scoring pipeline for swingup competition (.mldatx processing)
**Researched:** 2026-04-02

---

## Recommended Architecture

Use a **layered function decomposition** inside a single Live Script entry point,
with pure helper functions as local functions at the bottom of the same file.
Do NOT split into multiple files at this scale (5 teams, one-shot grading tool).
A single `.m` Live Script with local functions is idiomatic for this project,
keeps the scoring logic co-located with its narrative annotations, and matches
the pattern already used in `RRpendulum_swingup_controldesign.m`.

### Overview

```
scripts/score_competition.m          <- entry point (Live Script .m)
    |
    +--> Section 1: cfg struct       (all thresholds, paths, team list)
    +--> Section 2: per-team loop
    |       calls load_team_data()
    |       calls pick_signals()     (interactive, per .mldatx file)
    |       calls score_attempt()    (pure computation)
    |       calls aggregate_team()   (best-per-metric reduction)
    +--> Section 3: rank + points
    |       calls rank_stepper()
    |       calls score_bldc()
    +--> Section 4: output
            calls make_table()
            calls plot_attempts()
            calls export_csv()
```

All functions after the `%% Local Functions` section break at the bottom
of the script are local functions — visible only within the file, no extra
files on the path needed.

---

## Component Boundaries

| Component | Responsibility | Input | Output | Communicates With |
|-----------|---------------|-------|--------|-------------------|
| **cfg struct** (inline) | Single source of truth for all thresholds and paths | Hardcoded defaults | `cfg` struct | All components via pass-through |
| **`load_team_data()`** | Load `.mldatx`, enumerate runs, return run objects | `zipPath`, `cfg` | `struct array` of `{hwRun, simRun}` per attempt | `Simulink.sdi.*` API |
| **`pick_signals()`** | Interactive selection of `accel_cmd` and `q2` from a run | `Simulink.sdi.Run`, signal role string | `Simulink.sdi.Signal` | `listdlg`, `load_team_data` output |
| **`score_attempt()`** | Compute all metrics for one attempt | `hwSig_cmd`, `hwSig_q2`, `simSig_q2`, `cfg` | `attempt` struct (`t_start`, `t_swingup`, `smape`, `success`, `participated`) | `rank_stepper`, `score_bldc` |
| **`aggregate_team()`** | Reduce N attempts to best-per-metric | Array of `attempt` structs | `team` struct (`best_time`, `best_smape`, `participated`) | leaderboard builder |
| **`rank_stepper()`** | Assign competitive points to 4 stepper teams | Array of `team` structs | Points array | table builder |
| **`score_bldc()`** | Assign absolute SMAPE points to BLDC team | Single `team` struct, `cfg` | Points scalar | table builder |
| **`make_table()`** | Assemble final MATLAB table | All team+points data | `results` table | output section |
| **`plot_attempts()`** | Time-aligned q2 overlay + SMAPE window shading | `attempt` struct, `cfg` | Figure handles | output section |
| **`export_csv()`** | Write `results` table to CSV/Excel | `results` table, `cfg.output_dir` | File on disk | — |

---

## Data Flow

```
.mldatx files (zip per team)
        |
        v
[load_team_data]
  Simulink.sdi.load() per file
  Simulink.sdi.getAllRunIDs() -> run list
  Identify hw vs sim run by: run Name heuristic OR
    signal count difference OR run order (archived=hw, recent=sim)
  Returns: hwRun[], simRun[] (one pair per attempt)
        |
        v
[pick_signals]  <- INTERACTIVE (runs once per attempt file)
  getAllSignals() on hwRun -> signal name list
  Sort by keyword match: "accel","cmd","torque" first for cmd signal
                         "q2","theta","pend" first for angle signal
  listdlg() -> scorer selects accel_cmd signal and q2 signal
  Same selection re-used for simRun (same signal names assumed)
  Returns: hwSig_cmd, hwSig_q2, simSig_q2  (Simulink.sdi.Signal objects)
        |
        v
[score_attempt]  <- PURE COMPUTATION, no UI
  Extract timeseries: sig.Values (timeseries object)
  t_start: first t where |accel_cmd| > cfg.accel_thresh
  Align both signals to t=0 at t_start
  Participation check: max(|q2_hw|) > pi/2
  Success check: q2_hw crosses +-pi AND stays within cfg.catch_deg
                 for >= cfg.catch_duration_s
  t_swingup: t_start to first +-pi crossing
  SMAPE window: configurable via cfg.smape_window
    'fixed':    t_start to t_start + cfg.smape_fixed_s
    'angle':    t_start until |q2_hw| first exceeds cfg.smape_angle_rad
    'swingup':  t_start to t_swingup (success required for this mode)
  Interpolate simSig_q2 onto hw time grid (interp1, linear)
  SMAPE = mean(2*|hw-sim| ./ (|hw|+|sim|+eps)) * 100  [percent]
  Returns: attempt struct
        |
        v
[aggregate_team]
  For time ranking: best t_swingup among successful attempts
  For SMAPE ranking: best (lowest) smape among participated attempts
                     (successful swingup NOT required for SMAPE)
  participated: any attempt where participation check passed
  Returns: team struct
        |
        v
[rank_stepper / score_bldc]
  rank_stepper: sort 4 teams by best_time -> 2/1/0.5/0 pts
                sort 4 teams by best_smape -> 2/1/0.5/0.5 pts
  score_bldc:   lookup cfg.bldc_smape_thresholds for absolute pts
  participation point: 1 pt if team.participated
  Returns: points struct per team
        |
        v
[make_table / plot_attempts / export_csv]
  MATLAB table: team, motor_type, best_time, best_smape,
                time_pts, smape_pts, participation_pt, total_pts
  Plots: per team, one figure per attempt showing hw q2 vs sim q2
         vertical lines at t_start and t_swingup
         shaded SMAPE window
  CSV: writetable(results, fullfile(cfg.output_dir, 'scores.csv'))
```

---

## Configuration Struct (at top of script)

All thresholds live in a single `cfg` struct defined in Section 1. This is
the correct pattern — no magic numbers scattered in computation functions.

```matlab
cfg.data_dir           = fullfile(prj.RootFolder, 'data', 'competition');
cfg.output_dir         = fullfile(prj.RootFolder, 'data', 'competition', 'results');
cfg.teams              = {'Team1','Team2','Team3','Team4','Team5_BLDC'};
cfg.motor_type         = {'stepper','stepper','stepper','stepper','BLDC'};
cfg.accel_thresh       = 0.01;         % rad/s^2 or Nm, above = activity
cfg.catch_deg          = 2.0;          % degrees, success window around pi
cfg.catch_duration_s   = 1.0;          % seconds within catch window
cfg.smape_window       = 'swingup';    % 'fixed' | 'angle' | 'swingup'
cfg.smape_fixed_s      = 5.0;          % used if smape_window='fixed'
cfg.smape_angle_rad    = pi/2;         % used if smape_window='angle'
cfg.bldc_smape_thresholds = [40 80 120 160]; % breakpoints in percent
cfg.bldc_smape_points     = [4  3   2   1  0];
```

The `cfg` struct is passed by value to every helper function. Functions do
not modify `cfg` — they read from it. This avoids hidden state.

---

## Interactive Element Placement

Signal selection (`pick_signals`) must run **inside the per-team, per-attempt
loop**, not as a pre-processing step, because:

1. Signal names differ between teams and between attempts within a team.
2. The scorer must confirm each mapping visually — bulk auto-map is out of scope.
3. `listdlg` blocks execution and returns control when the scorer dismisses it.

Placement: Section 2 of the Live Script, inside the attempt loop.
The function displays the run name and attempt number as the dialog title
so the scorer always knows which file they are mapping.

Smart sorting within `pick_signals`:

```
Priority keywords for accel_cmd:  ["accel", "cmd", "torque", "tau", "u"]
Priority keywords for q2:         ["q2", "theta2", "pend", "theta", "angle"]
Sorting: signals whose Name contains any priority keyword float to top.
         Case-insensitive. Remaining signals sorted alphabetically.
```

This reduces the scorer's click burden while keeping manual confirmation.

---

## Two Scoring Paths

The two paths diverge only at the **points assignment** stage, not earlier.
`score_attempt` and `aggregate_team` are shared and motor-type-agnostic.
The fork is implemented by a single `if` branch in Section 3:

```
for each team
    if cfg.motor_type == 'stepper'
        -> feed into rank_stepper() together with all other stepper teams
    else  (BLDC)
        -> feed into score_bldc() independently
```

`rank_stepper` receives all 4 stepper team structs at once (needs cross-team
ranking). `score_bldc` receives only the BLDC team struct (absolute lookup).
Both return a uniform points struct so `make_table` treats them identically.

---

## Patterns to Follow

### Pattern 1: Config-first, compute-later
**What:** Define `cfg` in one block at the top before any SDI calls.
**When:** Always — parameters change between grading sessions.
**Why:** Instructor may want to re-run with a different SMAPE window
(e.g., `'fixed'` vs `'swingup'`). Only one place to change.

### Pattern 2: Pure computation functions
**What:** `score_attempt`, `aggregate_team`, `rank_stepper`, `score_bldc`
take structs in, return structs out, have no side effects, no `figure` calls.
**When:** All numeric computation.
**Why:** Testable in isolation. Safe to re-run without UI side effects.

### Pattern 3: Explicit run disambiguation
**What:** When loading a `.mldatx`, identify hw vs sim run by inspecting
`run.Name` (SDI sets name to model name for simulation runs) and by run
order (archived run = older = hardware, recent run = simulation).
Fallback: show run names in a `listdlg` and let scorer pick if heuristic
is ambiguous.
**When:** `load_team_data`.
**Why:** The SDI `Run` object has no `IsArchived` flag exposed in the API.
Run order and name are the only reliable discriminators.

### Pattern 4: Interpolation before SMAPE
**What:** Always interpolate the simulation q2 signal onto the hardware
time grid using `interp1(sim_t, sim_q2, hw_t, 'linear', 'extrap')` before
computing SMAPE.
**When:** `score_attempt`, inside SMAPE window.
**Why:** Hardware runs at 2 kHz; simulation time step may differ. Comparing
without resampling produces incorrect SMAPE (point-count mismatch).

---

## Anti-Patterns to Avoid

### Anti-Pattern 1: Global or persistent variables for state
**What:** Using `global cfg` or `persistent` inside helpers.
**Why bad:** Makes functions depend on external state; breaks re-runs and
testing. All configuration is passed explicitly as arguments.
**Instead:** Pass `cfg` struct as argument to every function that needs it.

### Anti-Pattern 2: Auto-detection of signal roles without confirmation
**What:** Silently mapping signals based on name matching alone.
**Why bad:** Teams name signals differently; silent mis-map produces wrong
scores with no audit trail. One silent error invalidates a team's grade.
**Instead:** Always call `listdlg` for confirmation even when the best
candidate is obvious. Smart sorting reduces friction; confirmation is
mandatory.

### Anti-Pattern 3: Computing SMAPE before time alignment
**What:** Using raw signal timestamps as-is.
**Why bad:** The hardware run clock and simulation clock start at different
absolute times. Raw SMAPE on unaligned signals is meaningless.
**Instead:** Always extract `t_start` from the accel_cmd signal, shift
both signals to t=0, then compute SMAPE in the aligned window.

### Anti-Pattern 4: Monolithic script with inline computation
**What:** One 400-line script with no function boundaries.
**Why bad:** Debugging a mis-scored team requires re-running the entire
script. `score_attempt` is the most complex function and needs to be
callable in isolation for validation.
**Instead:** Local functions with clear signatures even though they live
in the same file. MATLAB executes them as proper function scopes.

---

## Build Order (Phase Dependency)

Dependencies flow strictly downward. Build in this order:

```
1. cfg struct definition
        (no dependencies)

2. load_team_data()
        depends on: cfg (paths, data_dir)
        validates: SDI API access, mldatx file format

3. pick_signals()
        depends on: load_team_data output (Run objects)
        validates: signal name sorting, listdlg UX

4. score_attempt()
        depends on: pick_signals output (Signal objects), cfg (thresholds)
        this is the highest-risk function — most numerical logic

5. aggregate_team()
        depends on: score_attempt (attempt structs)

6. rank_stepper() and score_bldc()
        depends on: aggregate_team (team structs), cfg (BLDC thresholds)

7. make_table(), plot_attempts(), export_csv()
        depends on: all above
```

Start with Step 4 (`score_attempt`) using synthetic timeseries data before
Steps 2-3 are complete. This lets you validate the metric math independently
of the SDI loading and UI interaction.

---

## Scalability Considerations

This is a one-shot grading tool for 5 teams. Scalability is not a concern.
The relevant constraint is correctness under the interactive workflow:

| Concern | Approach |
|---------|----------|
| Scorer makes wrong signal selection | `pick_signals` shows run name + attempt number in dialog title; selection can be re-run by re-executing Section 2 for that team |
| Team has more than 3 attempts | Loop handles N attempts; aggregation selects best automatically |
| Hardware and simulation have different signal counts | `pick_signals` enumerates signals from hw run for cmd, from either run for q2 |
| SMAPE window choice changes grade | `cfg.smape_window` change + re-run of Section 2-3 is sufficient |
| SDI session state polluted between teams | Call `Simulink.sdi.clear()` before each `load_team_data` call to prevent run ID accumulation |

---

## Sources

- [Simulink.sdi.Run — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.run.html) (MEDIUM confidence — 403 on direct fetch, confirmed via search)
- [getAllSignals method](https://www.mathworks.com/help/simulink/slref/simulink.sdi.run.getallsignals.html) (MEDIUM confidence)
- [Simulink.sdi.load](https://www.mathworks.com/help/simulink/slref/simulink.sdi.load.html) (HIGH confidence via search results)
- [Simulink.sdi.Signal](https://www.mathworks.com/help/simulink/slref/simulink.sdi.signal.html) (HIGH confidence — timeseries export confirmed)
- [listdlg](https://www.mathworks.com/help/matlab/ref/listdlg.html) (HIGH confidence)
- Project convention: `scripts/RRpendulum_swingup_controldesign.m` (local file — high confidence)
