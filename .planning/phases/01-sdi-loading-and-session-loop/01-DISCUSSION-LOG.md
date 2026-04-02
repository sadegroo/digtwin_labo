# Phase 1: SDI Loading and Session Loop - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-04-02
**Phase:** 01-sdi-loading-and-session-loop
**Areas discussed:** Run discrimination, Session input method, Team management
**Mode:** --auto (all selections made automatically using recommended defaults)

---

## Run Discrimination

| Option | Description | Selected |
|--------|-------------|----------|
| Pure heuristic (no confirmation) | Use signal density or creation order to auto-assign runs | |
| Heuristic with user confirmation | Apply heuristic, show summary, let scorer confirm/swap | ✓ |
| Always ask scorer | No heuristic — scorer manually picks which run is hardware | |

**User's choice:** [auto] Heuristic with user confirmation (recommended default)
**Notes:** STATE.md flags the heuristic as unvalidated against real `.mldatx` files. Adding confirmation step mitigates risk without adding significant overhead (scorer just confirms or swaps two runs).

---

## Session Input Method

| Option | Description | Selected |
|--------|-------------|----------|
| All text (`input()`) | Scorer types file paths and commands at command window | |
| All GUI (`uigetfile` + `listdlg`) | File dialog for files, list dialog for everything | |
| Hybrid | `uigetfile` for files, `input()` for team name and finalize | ✓ |

**User's choice:** [auto] Hybrid: `uigetfile` for file selection, `input()` for team name and finalize (recommended default)
**Notes:** File paths are error-prone to type; dialog is more ergonomic. Team name and finalize command are short text inputs where `input()` is natural. Finalize keyword: `'done'`.

---

## Team Management

| Option | Description | Selected |
|--------|-------------|----------|
| Free-text entry | Scorer types team name each time | |
| Predefined list with `listdlg` | Teams defined in `cfg`, scorer clicks to select | ✓ |
| Predefined list with `input()` | Teams in `cfg`, scorer types team name (validated) | |

**User's choice:** [auto] Predefined list in `cfg` with `listdlg` selection (recommended default)
**Notes:** Prevents typos, enforces 4-stepper + 1-BLDC constraint. `cfg.teams` stores name and type (`'stepper'`/`'bldc'`), which drives Phase 4 scoring logic.

---

## Claude's Discretion

- Internal session state struct layout (field names, nesting)
- `Simulink.sdi.clear` placement and cleanup strategy
- Exact run discrimination heuristic (researcher to investigate SDI API)

## Deferred Ideas

None — discussion stayed within phase scope.
