# Swingup Competition Scoring Script

## What This Is

A MATLAB scoring tool for the digital twin swingup competition. It processes team submissions (`.mldatx` files from Simulink Data Inspector) one at a time in an interactive session loop, validates successful swingups via sustained-hold detection, computes swingup times and simulation accuracy (angular SMAPE with wrapping guards), and produces a ranked leaderboard with CSV/xlsx export and bar chart visualizations. Used by the instructor to grade 5 teams (4 stepper + 1 BLDC) after the competition. Shipped as a single 1,379-line Live Script with 25 local functions.

## Core Value

Correctly and fairly score every team's swingup attempts — accurate time extraction, proper signal alignment, and transparent SMAPE computation — so grades are defensible.

## Requirements

### Validated

- ✓ Existing MATLAB/Simulink project with pendulum models, parameters, and SDI logging — existing
- ✓ Hardware interface library (`resources/lib/digtwin_labo_lib.slx`) synced with MCU firmware — existing
- ✓ State vector convention `z = [q1; v1; q2; v2]` established across all scripts — existing
- ✓ Load and parse `.mldatx` files via Simulink Data Inspector API — v1.0 Phase 1
- ✓ Separate hardware run from simulation run via SimMode discrimination — v1.0 Phase 1
- ✓ Manual signal mapping with smart sorting (likely candidates first) — v1.0 Phase 2
- ✓ Detect first non-zero command and align hardware/simulation signals — v1.0 Phase 2
- ✓ Check successful swingup: q2 crosses ±π AND stays within ±2° for ≥1 second — v1.0 Phase 3
- ✓ Check participation: pendulum angle exceeds 90° (|q2| > π/2) — v1.0 Phase 3
- ✓ Compute swingup time: start time to first ±π crossing — v1.0 Phase 3
- ✓ Angular SMAPE with wrapping guards and D-09 hybrid window policy — v1.0 Phase 3
- ✓ Best-per-metric scoring across attempts — v1.0 Phase 4
- ✓ Stepper team dense ranking on time and SMAPE — v1.0 Phase 4
- ✓ BLDC team absolute SMAPE band scoring — v1.0 Phase 4
- ✓ Participation point awarded — v1.0 Phase 4
- ✓ Leaderboard MATLAB table with CSV/xlsx export — v1.0 Phase 4
- ✓ Grouped bar chart score breakdown — v1.0 Phase 4

### Active

(None — v1.0 complete. New requirements will be defined for next milestone.)

### Out of Scope

- Real-time scoring during the competition — this is post-hoc analysis
- Automatic signal name detection without manual confirmation — too error-prone with varying team setups
- Video or visual verification of attempts — trust the logged data
- Modifying team submissions or re-running simulations
- SMAPE window mode selection UI — D-09 hybrid policy (max of 5s and time-to-90deg) applied unconditionally; confirmed adequate for competition fairness

## Context

- **Competition setting**: Last lesson of the digital twin lab course. 5 teams, 20 minutes each on hardware.
- **Data format**: `.mldatx` files from Simulink Data Inspector, each containing 1 hardware run + 1 simulation run (one run per file, two files per attempt). Signals include at minimum: acceleration/torque command and pendulum states (q1, v1, q2, v2).
- **Angle convention**: q2=0 is pendulum hanging down, q2=±π is inverted (upright).
- **Sample rate**: Hardware runs at Ts=1/2000 (2 kHz). Simulation may differ — `interp1` resampling used for SMAPE.
- **Teams**: 4 stepper motor teams (ranked competitively against each other), 1 BLDC team (scored on absolute SMAPE scale).
- **Signal naming**: Varies between teams. Scorer picks signals manually via keyword-scored `listdlg`.
- **Current state (v1.0 shipped)**: Single-file architecture `scripts/score_competition.m` (1,379 lines, 25 local functions). Fully functional interactive scoring session with leaderboard and export.

## Constraints

- **Toolbox**: Must use Simulink Data Inspector API (`Simulink.sdi.*`) — this is how `.mldatx` files are structured
- **MATLAB version**: R2025b (current project version)
- **Existing project**: Script lives in the existing `digtwin_labo` project, under `scripts/`
- **Plain-text .m format**: New scripts must use Live Script `.m` format per project conventions

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Manual signal mapping with smart sorting | Signal names vary between teams; auto-detection unreliable | ✓ Good — keyword-scored listdlg works reliably |
| D-09 hybrid SMAPE window (max of 5s, time-to-90deg) | Ensures fair comparison regardless of swingup speed | ✓ Good — supersedes configurable window (METR-05) |
| Best-per-metric scoring | Fastest time and best SMAPE can come from different attempts | ✓ Good — implemented in aggregate_best |
| BLDC team scored absolutely, not competitively | Only 1 BLDC team, unfair to rank against 4 stepper teams | ✓ Good — separate band scoring |
| Single-file architecture (no +package or class) | 5 teams scored once per year; simplicity over modularity | ✓ Good — 25 local functions manageable |
| Two-file workflow (hw + sim separate .mldatx) | SDI runs cleaner with one run per file | ✓ Good — SimMode discrimination reliable |
| Conv-based sustained-hold detection | Backdates swingup time to hold entry, not hold completion | ✓ Good — fairer timing |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd:transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd:complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-04-06 after v1.0 milestone*
