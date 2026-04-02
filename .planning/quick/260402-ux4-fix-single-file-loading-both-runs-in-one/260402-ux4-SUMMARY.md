---
phase: quick
plan: 260402-ux4
subsystem: scoring-session-loop
tags: [session-loop, sdi-loading, run-discrimination, ux]
dependency_graph:
  requires: []
  provides: [single-file-loading]
  affects: [scripts/score_competition.m]
tech_stack:
  added: []
  patterns: [run-ordering-discrimination]
key_files:
  modified:
    - scripts/score_competition.m
decisions:
  - "Run ordering is primary hw/sim discrimination: ids(1)=archived=hw, ids(end)=most-recent=sim"
  - "SimMode is validation-only (warns if unexpected), not the discriminator"
  - "Confirm/swap prompt retained as human override for edge cases"
metrics:
  duration: "< 10 min"
  completed: "2026-04-02"
  tasks: 1
  files: 1
---

# Quick Task 260402-ux4: Fix Single-File Loading (Both Runs in One)

**One-liner:** Single-file session loop using run-ordering discrimination (ids(1)=hw, ids(end)=sim) instead of two-file workflow with SimMode-primary logic.

## What Changed

The original `score_competition.m` assumed hardware and simulation data arrived in **separate** `.mldatx` files (one run each). Reality: each `.mldatx` contains **both** runs — the archived run (hardware, `ids(1)`) and the most-recent run (simulation, `ids(end)`).

### Session loop (was two-file, now one-file)

| Before | After |
|--------|-------|
| Two `uigetfile` calls (hw then sim) | One `uigetfile` call per attempt |
| Cancel on sim silently retried | Single cancel path with done/retry |
| `load_file_pair(hw_file, sim_file)` | `load_attempt(fullfile(fdir, fname))` |
| PromptString showed two filenames | PromptString shows single filename |
| "Load another file pair?" | "Load another file?" |

### `load_attempt` function (renamed from `load_file_pair`)

| Before | After |
|--------|-------|
| Loaded two files, expected 2 runs (1 each) | Loads one file, asserts 2 runs |
| `SimMode` loop as primary discriminator | `ids(1)` = hw, `ids(end)` = sim (ordering) |
| Signal-count fallback when SimMode unclear | No fallback needed; ordering is deterministic |
| SimMode mismatch silently overrode | SimMode checked as warning-only validation |
| `attempt.hw_file` + `attempt.sim_file` | `attempt.file` (single field) |

## Deviations from Plan

None — plan executed exactly as written.

## Self-Check

- [x] `grep -c "uigetfile" scripts/score_competition.m` returns `1`
- [x] `function attempt = load_attempt(file)` present
- [x] `hw_id = ids(1)` and `sim_id = ids(end)` present
- [x] SimMode validation present (warning, not primary)
- [x] `attempt.file` field present
- [x] No references to `hw_file`, `sim_file`, or `load_file_pair`
- [x] Commit `07d7a11` exists

## Self-Check: PASSED
