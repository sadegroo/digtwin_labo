---
phase: 04-scoring-and-output
plan: 02
subsystem: scoring
tags: [export, visualization, csv, xlsx, bar-chart, matlab-table, finalization]
dependency_graph:
  requires:
    - Plan 04-01: compute_leaderboard output table T (8 columns)
    - Plan 04-01: cfg.export_dir and cfg.export_stem fields
    - Plan 04-01: session struct with teams and attempts
  provides:
    - export_results(T, cfg, session) -> timestamped CSV + xlsx + .mat in data/
    - plot_score_breakdown(T, cfg) -> grouped bar chart figure
  affects:
    - scripts/score_competition.m finalization section (Steps 4 and 5 added)
    - scripts/score_competition.m local functions (2 new functions before appendix)
tech_stack:
  added: []
  patterns:
    - datetime API for timestamping (not legacy datestr)
    - writetable for CSV and xlsx export
    - try/catch around xlsx write for locked-file robustness
    - grouped bar chart via bar(matrix, 'grouped')
key_files:
  created: []
  modified:
    - scripts/score_competition.m
decisions:
  - Session struct saved to .mat via passed argument (not evalin) for post-hoc inspection
  - xlsx export wrapped in try/catch with scorer:xlsxfail warning (T-04-03 mitigation)
  - Color scheme: blue=TimePoints, orange=SMAPEPoints, green=Participation
  - Total points annotated above each bar group for quick visual scan
metrics:
  duration: ~15 minutes
  completed: 2026-04-06
  tasks_completed: 2
  tasks_total: 2
  files_modified: 1
requirements_completed: [OUTP-02]
---

# Phase 4 Plan 02: Export and Visualization Summary

**One-liner:** CSV/xlsx export with timestamped filenames and grouped bar chart score breakdown wired into finalization pipeline.

## What Was Built

Two new local functions added to `scripts/score_competition.m` and wired into the finalization pipeline as Steps 4 and 5:

### `export_results(T, cfg, session)`

- Creates `cfg.export_dir` if missing (defensive `mkdir`)
- Generates timestamp via `datetime('now', 'Format', 'yyyyMMdd_HHmm')` (not legacy `datestr`)
- Writes `data/competition_results_YYYYMMDD_HHMM.csv` via `writetable` (always succeeds)
- Writes `data/competition_results_YYYYMMDD_HHMM.xlsx` in try/catch (T-04-03: Excel lock protection)
- Saves `data/competition_results_YYYYMMDD_HHMM.mat` with `session` and `T` for post-hoc inspection
- Session struct passed as 3rd argument (not `evalin` — reliable from local function scope)
- Both stepper and BLDC rows appear in exported table; BLDC Rank column is NaN (D-14)

### `plot_score_breakdown(T, cfg)`

- Builds N_teams x 3 score matrix: `[T.TimePoints, T.SMAPEPoints, T.ParticipationPoint]`
- Creates grouped bar chart (`bar(score_matrix, 'grouped')`)
- Color scheme: blue (time), orange (SMAPE), green (participation)
- x-axis labels: team names with 15-degree rotation
- Legend: 'Time Points', 'SMAPE Points', 'Participation'
- Total points annotated above each team's bar group in bold

### Finalization Pipeline (complete)

```
Step 1: print_diagnostics(session, cfg)
Step 2: T = compute_leaderboard(session, cfg)
Step 3: disp_leaderboard(T, session, cfg)
Step 4: export_results(T, cfg, session)       <- new
Step 5: plot_score_breakdown(T, cfg)          <- new
```

## Verification Results

All automated checks passed:

| Check | Result |
|-------|--------|
| `function export_results(T, cfg, session)` defined | Line 1273 |
| `function plot_score_breakdown(T, cfg)` defined | Line 1314 |
| `export_results(T, cfg, session)` called in finalization | Line 275 |
| `plot_score_breakdown(T, cfg)` called in finalization | Line 278 |
| `writetable(T,` count = 2 (CSV + xlsx) | Lines 1291, 1296 |
| `datetime('now', 'Format', 'yyyyMMdd_HHmm')` | Line 1287 |
| `warning('scorer:xlsxfail', ...)` | Line 1299 |
| `if ~exist(cfg.export_dir, 'dir')` | Line 1282 |
| `save(mat_file, 'session', 'T')` | Line 1306 |
| `bar(score_matrix, 'grouped')` | Line 1328 |
| `legend({'Time Points', 'SMAPE Points', 'Participation'}, ...)` | Line 1343 |
| `ylabel('Points')` | Line 1339 |
| `title('Competition Score Breakdown')` | Line 1340 |
| Both functions before `%[appendix]{"version":"1.0"}` | Lines 1273, 1314 < 1359 |
| Finalization order: print_diagnostics, compute_leaderboard, disp_leaderboard, export_results, plot_score_breakdown | Lines 266, 269, 272, 275, 278 |

## Deviations from Plan

- Added try/catch guards around finalization steps 1, 4, and 5 (not in original plan). During human verification, export and plot were not producing output. The try/catch wrappers ensure individual step failures don't block the rest of the pipeline. Fix committed as `d0a3f8d`.

## Known Stubs

None. Both functions are fully implemented and wired.

## Threat Flags

No new security-relevant surface introduced beyond what the plan's threat model covers (T-04-03 mitigated, T-04-04 and T-04-05 accepted).

## Self-Check: PASSED

- `scripts/score_competition.m` exists and is 1359 lines
- Commit `73bf415` contains the feat(04-02) changes
- Commit `c9fccce` restores files accidentally dropped by worktree reset
