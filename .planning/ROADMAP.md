# Roadmap: Swingup Competition Scoring Script

## Overview

A single MATLAB Live Script (`scripts/score_competition.m`) built in four data-flow-dependent phases. The script operates as an interactive session loop: the scorer loads one `.mldatx` file at a time, assigns it to a team, and the script immediately processes it (signal mapping, alignment, metrics) and updates a running leaderboard. When all files are loaded, the scorer issues a finalize command that triggers competitive ranking and exports results. Phase 1 establishes the session loop and SDI loading foundation; Phase 2 adds interactive signal selection, time alignment, and per-file overlay plots; Phase 3 computes swingup metrics and SMAPE; Phase 4 applies the scoring rubric and exports the leaderboard at finalization. Each phase is verifiable with real team data before the next begins.

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

Decimal phases appear between their surrounding integers in numeric order.

- [x] **Phase 1: SDI Loading and Session Loop** - Incremental file-by-file loading with team assignment, session state accumulation, and finalize command (completed 2026-04-02)
- [ ] **Phase 2: Signal Selection, Alignment, and Per-File Plots** - Interactive signal mapping, hysteresis start-time detection, t=0 alignment, and immediate overlay plot per attempt
- [ ] **Phase 3: Metric Computation** - Swingup success, swingup time, and SMAPE with angle-wrapping and window guards
- [ ] **Phase 4: Scoring and Output** - Rubric application at finalization, leaderboard table, CSV/Excel export, and diagnostic printouts

## Phase Details

### Phase 1: SDI Loading and Session Loop
**Goal**: The script reliably loads any well-formed team `.mldatx` file one at a time, assigns it to a named team on scorer input, accumulates results in a session state struct, and provides a finalize command that signals the end of the session
**Depends on**: Nothing (first phase)
**Requirements**: LOAD-01, LOAD-02, LOAD-03, LOAD-04, LOAD-05, LOAD-06
**Success Criteria** (what must be TRUE):
  1. Loading a single `.mldatx` file yields exactly 2 runs and the script asserts on any other count
  2. Hardware run and simulation run are correctly identified without relying on position index (identified by signal density or `Run.getLatest()`)
  3. A corrupt or unloadable file is skipped with a logged error; the session continues and the scorer can provide the next file
  4. After loading each file, the scorer is prompted for a team name and the result is appended to that team's entry in the session state struct; loading a second file for the same team adds a second attempt, it does not overwrite the first
  5. Typing the finalize command ends the session loop and makes the accumulated session state available for competitive ranking; the leaderboard is not computed until this command is issued
**Plans:** 2/2 plans complete
Plans:
- [x] 01-01-PLAN.md — Script skeleton with cfg struct, session init, SDI load-and-assert, SimMode run discrimination
- [x] 01-02-PLAN.md — Interactive session loop with team assignment, attempt appending, and finalize detection

### Phase 2: Signal Selection, Alignment, and Per-File Plots
**Goal**: For each file as it is loaded, the scorer interactively picks the correct signals, visually confirms the selection, has both signals aligned to a common t=0, and immediately sees an overlay plot of that attempt — all before the next file is loaded
**Depends on**: Phase 1
**Requirements**: SIGM-01, SIGM-02, SIGM-03, ALGN-01, ALGN-02, OUTP-03
**Success Criteria** (what must be TRUE):
  1. Signal selection dialog shows all signals with likely candidates (accel, torque, cmd, q2, theta, pend) sorted to the top
  2. Scorer picks one accel/torque command signal and one q2 signal per attempt before any metrics are computed
  3. A preview plot of the selected signals is shown before the scorer confirms the mapping
  4. Start time is detected using a hysteresis threshold (configurable, defaults to 5% of peak command over 5 consecutive samples) that does not fire on sensor noise
  5. After alignment, both hardware and simulation q2 signals start from t=0 at the first non-zero command
  6. An overlay plot of the aligned hardware q2 vs simulation q2 is displayed immediately after each file is processed, so the scorer can visually verify the attempt before loading the next file
**Plans:** 2 plans
Plans:
- [x] 02-01-PLAN.md — Signal selection, preview, and alignment local functions (select_signals, score_names, extract_signal, plot_preview, align_signals)
- [x] 02-02-PLAN.md — Overlay figure functions and session loop wiring (create_overlay_figure, update_overlay_figure, draw_attempt_subplots, full flow integration)

### Phase 3: Metric Computation
**Goal**: For each attempt, the script correctly computes whether swingup was achieved, the swingup time, and SMAPE between hardware and simulation q2 — with correct handling of angle wrapping, division-by-zero guards, and configurable windows
**Depends on**: Phase 2
**Requirements**: METR-01, METR-02, METR-03, METR-04, METR-05, METR-06, METR-07
**Success Criteria** (what must be TRUE):
  1. Swingup success requires q2 to cross +/-pi AND hold within +/-2deg of +/-pi continuously for at least 1 second; a brief crossing followed by a fall is not counted as success
  2. Participation is detected when |q2| exceeds pi/2 for at least 10 consecutive samples (noise-robust)
  3. Swingup time is measured from aligned t=0 to the first sustained +/-pi crossing on the hardware signal
  4. SMAPE is computed using angular difference formula to handle wrapping near +/-pi, with samples excluded where the denominator is below epsilon (1e-3 rad)
  5. SMAPE window mode is selectable from `cfg` as `'fixed'`, `'angle'`, or `'swingup'`; the chosen mode is applied consistently across all teams
**Plans:** 1 plan
Plans:
- [x] 03-01-PLAN.md — compute_metrics local function (swingup detection, participation, SMAPE) and session loop wiring

### Phase 4: Scoring and Output
**Goal**: When the scorer finalizes the session, the script applies the full scoring rubric across the accumulated session state, produces a ranked leaderboard table, exports to CSV and Excel, and prints per-team diagnostic summaries that make grades defensible
**Depends on**: Phase 3
**Requirements**: SCOR-01, SCOR-02, SCOR-03, SCOR-04, SCOR-05, SCOR-06, OUTP-01, OUTP-02, OUTP-04, OUTP-05
**Success Criteria** (what must be TRUE):
  1. Best-per-metric scoring selects fastest swingup time and lowest SMAPE independently across all attempts per team (they may come from different attempts)
  2. Four stepper teams are ranked on time (2/1/0.5/0 pt) and on SMAPE (2/1/0.5/0.5 pt) with ties handled by equal rank assignment
  3. BLDC team receives absolute SMAPE score (4/3/2/1/0 pt on 0-40/40-80/80-120/120-160/160+ % bands) and is not ranked against stepper teams
  4. Output MATLAB table has columns Team, BestSwingupTime, BestSMAPE, TimePoints, SMAPEPoints, ParticipationPoint, TotalPoints, Rank and is exported to both CSV and `.xlsx` at finalization
  5. Per-team diagnostic summary prints N attempts loaded, best time, best SMAPE, and participation status to the command window
  6. All tunable parameters (thresholds, SMAPE window mode, angle tolerances, team names, team types) are defined in a `cfg` struct at the top of the script
**Plans:** 2 plans
Plans:
- [ ] 04-01-PLAN.md — Scoring rubric core: cfg fields, best-per-metric aggregation, dense ranking, BLDC band scoring, leaderboard table, diagnostics display
- [ ] 04-02-PLAN.md — CSV/xlsx export, grouped bar chart, and end-to-end human verification

## Progress

**Execution Order:**
Phases execute in numeric order: 1 -> 2 -> 3 -> 4

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. SDI Loading and Session Loop | 2/2 | Complete   | 2026-04-02 |
| 2. Signal Selection, Alignment, and Per-File Plots | 2/2 | Complete | 2026-04-03 |
| 3. Metric Computation | 0/1 | Planned | - |
| 4. Scoring and Output | 0/2 | Planned | - |
