---
phase: 02-signal-selection-alignment-and-per-file-plots
plan: "01"
subsystem: scoring-script
tags: [signal-selection, time-alignment, listdlg, interp1, matlab]
dependency_graph:
  requires: [01-01, 01-02]
  provides: [select_signals, score_names, extract_signal, plot_preview, align_signals]
  affects: [02-02]
tech_stack:
  added: []
  patterns: [keyword-scored listdlg, unique-t guard for SDI extraction, interp1 defensive resampling]
key_files:
  created: []
  modified:
    - scripts/score_competition.m
decisions:
  - cfg.cmd_keywords and cfg.q2_keywords defined with exact keyword lists from RESEARCH.md (D-02)
  - select_signals signature takes cmd_keywords/q2_keywords as arguments (not workspace access) for local function compatibility
  - align_signals delta subtracted (not added) for left-shift convention per Pitfall 4 (D-05)
  - unique(t) guard in extract_signal prevents interp1 failure on duplicate SDI timestamps (Pitfall 5)
metrics:
  duration: "2 minutes"
  completed: "2026-04-03T09:05:31Z"
  tasks_completed: 2
  files_modified: 1
---

# Phase 02 Plan 01: Signal Selection Functions and Time Alignment Summary

## One-liner

Keyword-sorted listdlg signal selection with team defaults, safe SDI extraction, 2-subplot preview confirm/repick, and first-nonzero-command time alignment with interp1 defensive resampling.

## What Was Built

Five new local functions and two new cfg fields added to `scripts/score_competition.m` as the computational foundation for Phase 2 session loop integration (Plan 02).

### Configuration additions

- `cfg.cmd_keywords = {'accel', 'torque', 'cmd', 'tau', 'ref', 'input'}` — drives command signal sorting heuristic
- `cfg.q2_keywords = {'q2', 'theta', 'pend', 'angle', 'phi', 'joint2'}` — drives q2 signal sorting heuristic

### New local functions

**`select_signals(hw_run, team, cmd_keywords, q2_keywords)`** — Presents two sequential `listdlg` dialogs (command signal, then q2 signal) with keyword-scored, descending-sorted signal lists. Pre-selects top-scored candidate for fresh teams; pre-selects last used signal name for teams with prior attempts (D-03). Returns `[]` on cancel.

**`score_names(names, keywords)`** — Case-insensitive substring scoring: for each name, counts how many keywords appear as substrings. Returns numeric score vector aligned with input names.

**`extract_signal(run_obj, signal_name)`** — Looks up signal by name in SDI run, extracts `timeseries.Time` and `squeeze(timeseries.Data)`, applies `unique(t)` to remove any duplicate timestamps (prevents `interp1` failure downstream). Returns `t=[]`, `data=[]` if signal not found.

**`plot_preview(hw_t, hw_cmd, hw_q2, cmd_name, q2_name)`** — Creates a temporary figure with two linked subplots: command signal (blue, top) and pendulum angle in degrees (red, bottom). Prompts scorer to confirm or type "repick". Returns logical `confirmed`. Closes figure before returning.

**`align_signals(hw_t, hw_cmd, hw_q2, sim_t, sim_cmd, sim_q2, delta)`** — Detects start time independently per run as first sample where `abs(cmd) > 0`. Aligns both time vectors to t=0. Subtracts `delta` from hw time axis (positive delta = left-shift hw = compensate hardware dead time). Crops both signals to common overlap (no NaN padding). Defensively resamples sim onto hw time grid via `interp1(..., 'linear')` if sample counts or timestamps differ by more than 1e-6 s. Returns struct with 7 fields: `t`, `hw_q2`, `sim_q2`, `hw_cmd`, `hw_t_start`, `sim_t_start`, `delta`.

## Commits

| Task | Commit | Description |
|------|--------|-------------|
| Task 1 | 8c42f34 | feat(02-01): add cfg keywords and signal selection + preview local functions |
| Task 2 | 4ef451f | feat(02-01): add align_signals local function for time alignment and cropping |

## Deviations from Plan

None - plan executed exactly as written.

## Known Stubs

None. All functions are complete implementations. No data flows to UI rendering from these stubs — the functions are utilities to be called by the session loop in Plan 02.

## Self-Check: PASSED

- `scripts/score_competition.m` exists: FOUND
- `8c42f34` exists: FOUND (git log confirms)
- `4ef451f` exists: FOUND (git log confirms)
- 6 local functions present: CONFIRMED (`load_attempt`, `select_signals`, `score_names`, `extract_signal`, `plot_preview`, `align_signals`)
- cfg.cmd_keywords defined: CONFIRMED
- cfg.q2_keywords defined: CONFIRMED
- `%[appendix]{"version":"1.0"}` at end: CONFIRMED (line 415)
