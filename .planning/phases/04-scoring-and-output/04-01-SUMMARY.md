---
phase: 04-scoring-and-output
plan: 01
subsystem: scoring
tags: [scoring, leaderboard, ranking, matlab-table, bldc, stepper]
dependency_graph:
  requires:
    - Phase 3 compute_metrics output (metrics struct with swingup_success, swingup_time, participation, smape, smape_eligible)
    - session.teams(i).attempts{j}.metrics populated by session loop
  provides:
    - compute_leaderboard(session, cfg) -> MATLAB table T with 8 columns
    - aggregate_best(team) -> best_time, best_smape, has_participation
    - assign_points_dense(values, point_table) -> points with sports-convention tie-breaking
    - score_bldc_smape(smape_pct, bands, band_pts) -> band points
    - print_diagnostics(session, cfg) -> command window per-team summary
    - disp_leaderboard(T, session, cfg) -> stepper table + BLDC section
  affects:
    - scripts/score_competition.m finalization section (lines 257-275)
    - cfg struct (lines 29-36, new scoring fields)
tech_stack:
  added: []
  patterns:
    - Dense ranking via sort + forward-scan rank assignment (no dedicated rank function)
    - Sports-convention tie-breaking: tied competitors share the same rank index
    - Upper-exclusive band scoring: smape_pct < bands(k) for BLDC absolute scoring
    - Best-per-metric aggregation: min(..., 'omitnan') over all attempts per team
key_files:
  created: []
  modified:
    - scripts/score_competition.m
decisions:
  - cfg scoring fields added after q2_keywords block (line 29), before q2 unit prompt
  - print_diagnostics reuses aggregate_best to avoid duplicating aggregation logic
  - disp_leaderboard uses disp(T(stepper_mask,:)) for stepper table (D-10 native MATLAB table display)
  - BLDC band label computed inline in disp_leaderboard using edges = [0, bands] construction
  - Worktree branch-base correction required git reset --soft; CLAUDE.md and test_load_file_pair.m restored in chore commit 71d4b65
metrics:
  duration_minutes: 30
  completed_date: "2026-04-06"
  tasks_completed: 2
  tasks_total: 2
  files_modified: 1
requirements_completed:
  - SCOR-01
  - SCOR-02
  - SCOR-03
  - SCOR-04
  - SCOR-05
  - SCOR-06
  - OUTP-01
  - OUTP-04
  - OUTP-05
---

# Phase 4 Plan 01: Scoring Core and Leaderboard Display Summary

**One-liner:** Dense-ranking stepper + absolute-band BLDC scoring with per-team diagnostics and split leaderboard display via 6 local functions added to score_competition.m.

## What Was Built

Added 6 cfg fields and 6 new local functions to `scripts/score_competition.m`, and replaced the finalization stub with a 3-step pipeline (diagnostics -> leaderboard computation -> display).

### Configuration Fields Added (lines 29-36)

| Field | Value | Purpose |
|-------|-------|---------|
| `cfg.time_points` | `[2, 1, 0.5, 0]` | Stepper time rank points 1st..4th (D-05) |
| `cfg.smape_points` | `[2, 1, 0.5, 0.5]` | Stepper SMAPE rank points 1st..4th (D-06) |
| `cfg.bldc_smape_bands` | `[40, 80, 120, 160]` | Upper-exclusive SMAPE band edges % (D-07) |
| `cfg.bldc_smape_pts` | `[4, 3, 2, 1, 0]` | Points per band (D-07) |
| `cfg.export_dir` | `'data'` | Output folder (D-13) |
| `cfg.export_stem` | `'competition_results'` | Filename base (D-13) |

### Local Functions Added

**`aggregate_best(team)`** (SCOR-01, D-01..D-03)
Iterates over all attempts, extracts fastest swingup time and lowest eligible SMAPE via `min(..., 'omitnan')`. Guards empty/all-NaN arrays. Returns `has_participation` flag.

**`assign_points_dense(values, point_table)`** (SCOR-02, SCOR-03, D-04..D-06)
Sports-convention dense ranking: sort ascending, forward-scan to assign ranks (ties share rank, no gaps after tie), map rank to point_table. Teams with NaN values receive 0 points.

**`score_bldc_smape(smape_pct, bands, band_pts)`** (SCOR-04, D-07)
Upper-exclusive band lookup: `smape_pct < bands(k)` returns `band_pts(k)`. NaN input returns 0. Falls through to `band_pts(end)` for 160+% case.

**`compute_leaderboard(session, cfg)`** (SCOR-01..06, OUTP-01, D-01..D-09)
Aggregates best values per team, applies stepper dense ranking on time and SMAPE, applies BLDC band scoring, adds participation point (1pt per team with any participation), computes totals, assigns stepper final ranks (descending on total, BLDC rank = NaN). Returns 8-column MATLAB table.

**`print_diagnostics(session, cfg)`** (OUTP-04, D-15)
Prints per-team summary to command window: N attempts, best time (formatted), best SMAPE, participation status. Reuses `aggregate_best`.

**`disp_leaderboard(T, session, cfg)`** (D-10, D-11)
Displays stepper teams as a MATLAB table (`disp(T(stepper_mask,:))`). Prints BLDC section separately with team name, best SMAPE, band label, SMAPE points, participation, total.

### Finalization Section Replaced

Old stub (7 lines, no scoring) replaced with:
```
session.finalized = true;
print_diagnostics(session, cfg);      % D-15
T = compute_leaderboard(session, cfg); % SCOR-01..06
disp_leaderboard(T, session, cfg);    % D-10, D-11
```
"Proceed to Phase 3" message removed (verified: grep count = 0).

## Commits

| Task | Commit | Description |
|------|--------|-------------|
| Task 1 | 1b87ea7 | feat(04-01): add cfg scoring fields and 4 scoring local functions |
| Worktree fix | 71d4b65 | chore: restore files accidentally dropped by worktree reset |
| Task 2 | b3f813c | feat(04-01): replace finalization stub with diagnostics and leaderboard display |

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] Worktree branch reset dropped tracked files**
- **Found during:** Pre-execution worktree branch correction
- **Issue:** `git reset --soft 0b1dce2` left deletions of `CLAUDE.md` and `scripts/test_load_file_pair.m` staged in the index; these were committed accidentally in Task 1.
- **Fix:** Restored both files from `0b1dce2` in a separate chore commit (71d4b65) immediately after Task 1.
- **Files modified:** `CLAUDE.md`, `scripts/test_load_file_pair.m`
- **Commit:** 71d4b65

## Known Stubs

None. The `attempt.metrics = struct()` placeholder at line 348 is intentional — it is overwritten by `compute_metrics` in the main session loop and does not affect scoring.

## Threat Flags

None. No new network endpoints, auth paths, file access patterns, or schema changes beyond what the plan's threat model already covers (T-04-01 accepted, T-04-02 accepted, T-04-03 mitigated in Plan 02).

## Self-Check: PASSED

- scripts/score_competition.m: FOUND
- .planning/phases/04-scoring-and-output/04-01-SUMMARY.md: FOUND
- Commit 1b87ea7 (Task 1): FOUND
- Commit 71d4b65 (worktree restore): FOUND
- Commit b3f813c (Task 2): FOUND
