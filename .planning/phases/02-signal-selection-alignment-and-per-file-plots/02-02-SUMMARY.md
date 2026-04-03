---
phase: 02-signal-selection-alignment-and-per-file-plots
plan: "02"
subsystem: scoring-script
tags: [overlay-figure, uicontrol, session-loop, signal-alignment, attempt-browsing]
dependency_graph:
  requires: [02-01]
  provides: [create_overlay_figure, update_overlay_figure, draw_attempt_subplots, overlay_dropdown_callback, session-loop-phase2-wiring]
  affects: [02-03-checkpoint, 03-metric-computation]
tech_stack:
  added: []
  patterns: [uicontrol-popupmenu-not-uidropdown, delete-axes-not-clf, ancestor-not-gcf, linkaxes-3subplots, defensive-resample-before-align]
key_files:
  created: []
  modified:
    - scripts/score_competition.m
decisions:
  - uicontrol popupmenu used instead of uidropdown (standard figure, not uifigure)
  - delete(findobj(fig,'Type','axes')) instead of clf preserves uicontrol dropdown
  - Team assignment moved BEFORE signal selection so team defaults are available
  - Defensive resample (interp1) applied to both hw and sim q2 onto cmd time grid before align_signals
  - Sim run signal fallback: if name not found, offer manual listdlg re-pick
  - signal_ok flag pattern used to skip file cleanly after inner repick loop
metrics:
  duration: "10 minutes"
  completed: "2026-04-03T09:20:00Z"
  tasks_completed: 2
  files_modified: 1
---

# Phase 02 Plan 02: Overlay Figure and Session Loop Wiring Summary

## One-liner

Persistent 3-subplot overlay figure with uicontrol dropdown for attempt browsing, wired into the session loop for complete Phase 2 end-to-end flow.

## What Was Built

Four new local functions and a rewritten session loop body in `scripts/score_competition.m` that connect all Phase 2 building blocks (from Plan 01) into the working interactive flow.

### New local functions (Task 1)

**`create_overlay_figure()`** — Creates a persistent MATLAB `figure()` (not `uifigure`) at position 100,100,900,650 with a `uicontrol` popupmenu dropdown tagged `attempt_dropdown`. Stores attempt data in `fig.UserData` struct with `attempts` and `labels` cell arrays.

**`update_overlay_figure(fig, attempt, label)`** — Appends attempt to `fig.UserData`, updates dropdown `String` list to include all loaded attempts, sets `Value` to the newest entry, and calls `draw_attempt_subplots`.

**`draw_attempt_subplots(fig, attempt)`** — Brings figure to front, deletes only axes (`delete(findobj(fig,'Type','axes'))` — never `clf` which destroys the dropdown), then draws:
- Subplot 1 (top): hw q2 vs sim q2 in degrees, with `xline` t=0 annotation (including delta if nonzero)
- Subplot 2 (middle): command signal in magenta, with `xline` t=0
- Subplot 3 (bottom): q2 difference (hw - sim) in green, with `xline` t=0
- `linkaxes([ax1,ax2,ax3],'x')` for synchronized x-zoom across all three

**`overlay_dropdown_callback(src, ~)`** — Uses `ancestor(src,'figure')` (not `gcf`) and `get(src,'Value')` for 1-based index, then calls `draw_attempt_subplots` for the selected attempt.

Each function has a `%[text]` Live Script documentation header per CLAUDE.md conventions.

### Session loop and initialization changes (Task 2)

**Session init additions:**
- Three new fields per team: `last_cmd_signal`, `last_q2_signal`, `last_delta` for remembering defaults across attempts
- `overlay_fig = create_overlay_figure()` called once before the loop

**Rewritten session loop body (complete Phase 2 flow):**
1. File load (unchanged)
2. Team assignment via listdlg (moved BEFORE signal selection — needed for team defaults)
3. Signal selection repick loop: `select_signals` -> extract hw signals -> defensive resample q2 onto cmd time grid -> `plot_preview` -> confirm or repick
4. Sim run signal extraction with fallback: if signal name not found in sim run, manual listdlg pick
5. Defensive resample sim q2 onto sim cmd time grid
6. Manual delta prompt with `LEFT-SHIFT` convention, team default pre-filled
7. `align_signals` called with 7 arguments
8. Attempt struct extended: `attempt.signals` (cmd_name, q2_name, delta_s) and `attempt.aligned`
9. Team defaults updated: `last_cmd_signal`, `last_q2_signal`, `last_delta`
10. `update_overlay_figure` called to append attempt and redraw
11. Progress print and continue/done prompt (unchanged)

**Finalization:** Message updated from "Phase 2" to "Phase 3".

## Commits

| Task | Commit | Description |
|------|--------|-------------|
| Task 1 | a5ccbc0 | feat(02-02): add overlay figure local functions (create, update, draw, callback) |
| Task 2 | d29b420 | feat(02-02): wire Phase 2 functions into session loop and extend team/attempt structs |

## Deviations from Plan

None - plan executed exactly as written.

## Known Stubs

None. All 10 local functions are complete implementations. The `attempt.aligned` data is fully populated and ready for Phase 3 metric computation. `attempt.metrics` remains a placeholder struct (set by `load_attempt` in Phase 1) — this is intentional and will be filled in Phase 3.

## Self-Check: PASSED

- `scripts/score_competition.m` exists: FOUND
- `a5ccbc0` exists: FOUND (git log confirms)
- `d29b420` exists: FOUND (git log confirms)
- 10 local functions present: CONFIRMED (load_attempt, select_signals, score_names, extract_signal, plot_preview, align_signals, create_overlay_figure, update_overlay_figure, draw_attempt_subplots, overlay_dropdown_callback)
- `create_overlay_figure` uses uicontrol popupmenu: CONFIRMED
- `draw_attempt_subplots` uses delete not clf: CONFIRMED
- `overlay_dropdown_callback` uses ancestor not gcf: CONFIRMED
- Team defaults in session init: CONFIRMED (last_cmd_signal, last_q2_signal, last_delta)
- `overlay_fig = create_overlay_figure()` before session loop: CONFIRMED
- `attempt.aligned` assigned from align_signals: CONFIRMED
- `attempt.signals` substruct populated: CONFIRMED
- Finalization message says "Phase 3": CONFIRMED
- All `%[text]` documentation headers present: CONFIRMED
