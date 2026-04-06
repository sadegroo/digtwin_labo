---
phase: 03-metric-computation
plan: 01
subsystem: scoring
tags: [matlab, smape, swingup-detection, angular-wrapping, sustained-hold, metrics]

# Dependency graph
requires:
  - phase: 02-signal-selection-alignment-and-per-file-plots
    provides: aligned struct with hw_q2/sim_q2/t fields; file_q2_unit per-file unit
provides:
  - compute_metrics local function with participation, swingup, and SMAPE metrics
  - first_sustained_idx helper for conv-based sliding-window sustained-hold detection
  - compute_smape_angular helper with angular wrapping and denominator guard
  - yesno helper for diagnostic output
  - attempt.metrics struct populated before truncation in session loop
affects:
  - 04-scoring-rubric-and-leaderboard (consumes attempt.metrics fields for ranking)

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Sustained-hold detection via conv(double(mask), ones(1,N), 'valid') >= N"
    - "Angular SMAPE with mod(hw-sim+pi, 2*pi)-pi wrapping in numerator"
    - "Dynamic sample rate estimation via median(diff(t)) — no hardcoded 2000 Hz"
    - "D-09 hybrid SMAPE window: max(fixed_duration, time_to_first_90deg)"
    - "Local helper functions before %[appendix] marker following project Live Script pattern"

key-files:
  created: []
  modified:
    - scripts/score_competition.m

key-decisions:
  - "D-09 hybrid SMAPE window implemented unconditionally: max(5s, t_to_first_90deg); cfg.smape_window retained as legacy field"
  - "Participation uses 10-consecutive-sample filter per METR-02, not simple any() per D-14 — reconciled: D-14 defines threshold, METR-02 defines noise filter"
  - "epsilon = 1e-3 rad for SMAPE denominator guard: ~3x sensor noise floor (0.3 mrad/count at 2000 counts/rev)"
  - "Metrics computed before truncate_at_swingup so 1-second hold check and SMAPE window use full pre-truncation data"

patterns-established:
  - "Pattern: Insert metric computation between align_signals and truncate_at_swingup (D-06/D-07)"
  - "Pattern: Unit conversion to radians as first step inside compute_metrics — aligned struct stays in native unit for plots"
  - "Pattern: first_sustained_idx reusable for any sustained-hold check (used for both swingup N_hold and participation N_part=10)"

requirements-completed: [METR-01, METR-02, METR-03, METR-04, METR-05, METR-06, METR-07]

# Metrics
duration: 2min
completed: 2026-04-06
---

# Phase 3 Plan 01: Metric Computation Summary

**Angular-SMAPE with D-09 hybrid window, conv-based 1-second sustained swingup hold detection, and participation check wired into session loop before truncation**

## Performance

- **Duration:** 2 min
- **Started:** 2026-04-06T10:00:18Z
- **Completed:** 2026-04-06T10:02:38Z
- **Tasks:** 2
- **Files modified:** 1

## Accomplishments
- Added `compute_metrics(aligned, cfg, q2_unit)` implementing all 7 METR requirements in correct order
- Swingup detection uses conv-based sliding-window (not naive find) to enforce 1-second hold and backdate to hold entry (D-03/D-04)
- SMAPE uses `mod(hw-sim+pi, 2*pi)-pi` angular difference formula to handle +/-pi wrapping (D-12) with epsilon=1e-3 denominator guard (D-13)
- Metrics wired into session loop between align_signals and truncate_at_swingup, populating attempt.metrics for Phase 4

## Task Commits

Each task was committed atomically:

1. **Task 1: Add compute_metrics local function and helpers** - `b7bd322` (feat)
2. **Task 2: Wire compute_metrics into session loop and update cfg comment** - `951c922` (feat)

## Files Created/Modified
- `scripts/score_competition.m` - Added four local functions (first_sustained_idx, compute_smape_angular, yesno, compute_metrics) before %[appendix]; inserted compute_metrics call and updated cfg.smape_window comment

## Decisions Made
- Participation uses `first_sustained_idx(part_mask, 10)` not `any()` — METR-02 acceptance criterion specifies 10 consecutive samples for noise robustness; D-14 defines the threshold (pi/2), not the filter policy
- D-09 hybrid SMAPE window implemented unconditionally; cfg.smape_window retained as legacy field with updated comment documenting this
- epsilon = 1e-3 rad kept as internal constant inside compute_metrics (not promoted to cfg); can be exposed in Phase 4 if needed

## Deviations from Plan

None — plan executed exactly as written. All four functions match the plan specifications, the session loop insertion is at the correct position, and the cfg comment update is applied.

## Issues Encountered

None. The worktree branch base required correction (was pointing to main instead of dev_scoring_script HEAD at 4a2219a), which was resolved via git reset --soft before task execution began.

## User Setup Required

None — no external service configuration required. All changes are pure MATLAB arithmetic in existing project files.

## Next Phase Readiness

- `attempt.metrics` struct is populated with: `swingup_success`, `swingup_time`, `participation`, `smape_eligible`, `smape`, `smape_window_s`
- Phase 4 scoring rubric can consume these fields directly for team ranking and leaderboard generation
- No blockers

## Self-Check: PASSED

- FOUND: scripts/score_competition.m
- FOUND: 03-01-SUMMARY.md
- FOUND: b7bd322 (Task 1 commit)
- FOUND: 951c922 (Task 2 commit)

---
*Phase: 03-metric-computation*
*Completed: 2026-04-06*
