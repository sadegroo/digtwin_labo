# Swingup Competition Scoring Script

## What This Is

A MATLAB scoring tool for the digital twin swingup competition. It processes team submissions (`.mldatx` files from Simulink Data Inspector), validates successful swingups, computes swingup times and simulation accuracy (SMAPE), and produces a ranked leaderboard with plots. Used by the instructor to grade 5 teams (4 stepper + 1 BLDC) after the competition.

## Core Value

Correctly and fairly score every team's swingup attempts — accurate time extraction, proper signal alignment, and transparent SMAPE computation — so grades are defensible.

## Requirements

### Validated

- ✓ Existing MATLAB/Simulink project with pendulum models, parameters, and SDI logging — existing
- ✓ Hardware interface library (`resources/lib/digtwin_labo_lib.slx`) synced with MCU firmware — existing
- ✓ State vector convention `z = [q1; v1; q2; v2]` established across all scripts — existing

### Active

- [ ] Load and parse `.mldatx` files via Simulink Data Inspector API
- [ ] Separate hardware run (archive) from simulation run (recent) in each `.mldatx`
- [ ] Manual signal mapping with smart sorting (likely candidates first) for accel_cmd/torque and q2
- [ ] Detect first non-zero acceleration/torque command (start time) for time alignment
- [ ] Align hardware and simulation signals on the start time
- [ ] Check successful swingup: q2 crosses ±π AND stays within ±2° for ≥1 second
- [ ] Check participation: pendulum angle exceeds 90° (|q2| > π/2)
- [ ] Compute swingup time: start time to first ±π crossing
- [ ] Compute SMAPE on q2 between hardware and simulation with configurable window (fixed time, angle threshold, or swingup completion)
- [ ] Process a single team's zip of `.mldatx` files
- [ ] Aggregate all teams into a ranked leaderboard
- [ ] Stepper team ranking: fastest swingup time (1st=2pt, 2nd=1pt, 3rd=0.5pt, 4th=0pt)
- [ ] Stepper team ranking: lowest SMAPE (1st=2pt, 2nd=1pt, 3rd-4th=0.5pt)
- [ ] BLDC team absolute SMAPE scoring: 0-40%=4pt, 40-80%=3pt, 80-120%=2pt, 120-160%=1pt, 160-200%=0pt
- [ ] Participation point: 1pt if |q2| > 90°
- [ ] Best-per-metric scoring: fastest time and lowest SMAPE may come from different attempts
- [ ] SMAPE can be scored from any attempt where pendulum > 90° (successful swingup not required)
- [ ] Output: MATLAB table with all scores + comparison plots
- [ ] Output: CSV/Excel export for sharing

### Out of Scope

- Real-time scoring during the competition — this is post-hoc analysis
- Automatic signal name detection without manual confirmation — too error-prone with varying team setups
- Video or visual verification of attempts — trust the logged data
- Modifying team submissions or re-running simulations

## Context

- **Competition setting**: Last lesson of the digital twin lab course. 5 teams, 20 minutes each on hardware.
- **Data format**: `.mldatx` files from Simulink Data Inspector, each containing 1 hardware run (archived) + 1 simulation run (recent). Signals include at minimum: acceleration/torque command and pendulum states (q1, v1, q2, v2).
- **Angle convention**: q2=0 is pendulum hanging down, q2=±π is inverted (upright).
- **Sample rate**: Hardware runs at Ts=1/2000 (2 kHz). Simulation may differ — interpolation needed for SMAPE.
- **Teams**: 4 stepper motor teams (ranked competitively against each other), 1 BLDC team (scored on absolute SMAPE scale).
- **Signal naming**: Varies between teams. Script must let scorer pick signals manually, with intelligent sorting to surface likely candidates (e.g., signals containing "accel", "q2", "theta").

## Constraints

- **Toolbox**: Must use Simulink Data Inspector API (`Simulink.sdi.*`) — this is how `.mldatx` files are structured
- **MATLAB version**: R2025b (current project version)
- **Existing project**: Script lives in the existing `digtwin_labo` project, under `scripts/`
- **Plain-text .m format**: New scripts must use Live Script `.m` format per project conventions

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Manual signal mapping with smart sorting | Signal names vary between teams; auto-detection unreliable | — Pending |
| Configurable SMAPE window | Instructor may want fixed time (5s), angle threshold (90°), or until swingup | — Pending |
| Best-per-metric scoring | Fastest time and best SMAPE can come from different attempts | — Pending |
| BLDC team scored absolutely, not competitively | Only 1 BLDC team, unfair to rank against 4 stepper teams | — Pending |

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
*Last updated: 2026-04-02 after initialization*
