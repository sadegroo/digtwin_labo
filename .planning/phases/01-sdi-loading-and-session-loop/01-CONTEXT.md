# Phase 1: SDI Loading and Session Loop - Context

**Gathered:** 2026-04-02
**Status:** Ready for planning

<domain>
## Phase Boundary

This phase delivers the foundation: a session loop that loads `.mldatx` files one at a time via the Simulink Data Inspector API, discriminates hardware from simulation runs, assigns each file to a named team, accumulates results in a session state struct, and provides a finalize command to end the session. No signal mapping, alignment, metrics, or scoring happen here — those are Phases 2-4.

</domain>

<decisions>
## Implementation Decisions

### Run Discrimination
- **D-01:** Use a heuristic to identify hardware vs simulation runs (e.g., creation order from `Simulink.sdi.getRunIDList`, signal density comparison), then display a brief summary of each run (name, signal count, time range) and let the scorer confirm or swap the assignment. This guards against the unvalidated heuristic concern flagged in STATE.md.
- **D-02:** The script must assert exactly 2 runs per `.mldatx` file. Any other count triggers a warning and the file is skipped.

### Session Input Method
- **D-03:** Use `uigetfile('*.mldatx')` for file selection (ergonomic, prevents path typos). Use `input()` at the command window for team assignment and the finalize command.
- **D-04:** The finalize keyword is `'done'` — typing `done` at the team assignment prompt ends the session loop and triggers leaderboard computation.

### Team Management
- **D-05:** Define teams in `cfg.teams` as a struct array with fields `name` (string) and `type` (`'stepper'` or `'bldc'`). Default: 4 stepper teams + 1 BLDC team.
- **D-06:** On each file load, present the team list via `listdlg` so the scorer selects by clicking. This prevents typos and enforces the team roster defined in `cfg`.
- **D-07:** Multiple files for the same team append to that team's attempt list (no overwrite). The session state tracks attempts per team.

### Session State Structure
- **D-08:** Session state is a struct `session` with field `teams`, where each team entry holds: `name`, `type`, and `attempts` (a struct array). Each attempt stores the raw run IDs, file path, and a placeholder for metrics (populated in later phases).

### Error Handling
- **D-09:** Wrap file loading in `try/catch`. On failure, display the error message with `warning()`, skip the file, and continue the session loop. The scorer can retry with the next file.

### Claude's Discretion
- Internal struct layout details (field names, nesting depth) — Claude can choose the most practical structure as long as it supports the requirements.
- `Simulink.sdi.clear` call placement and SDI cleanup strategy — implementation detail.
- Exact heuristic for run discrimination (signal density vs creation order vs `getLatest()`) — researcher should investigate SDI API to determine the most reliable approach.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### SDI API
- No external spec — use MATLAB R2025b documentation for `Simulink.sdi.*` API (load, getRun, getRunIDList, clear, etc.)

### Project Context
- `.planning/REQUIREMENTS.md` — LOAD-01 through LOAD-06 define acceptance criteria
- `.planning/PROJECT.md` — Competition context (5 teams, 4 stepper + 1 BLDC, `.mldatx` format)
- `.planning/ROADMAP.md` — Phase 1 success criteria and dependencies

### Codebase
- `data/video.mldatx` — Example `.mldatx` file in the project (may be useful for testing SDI API calls)
- `CLAUDE.md` — Live Script `.m` format conventions, project open check, git commit rules

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- No existing scoring or SDI-related scripts — this is greenfield code
- `data/video.mldatx` exists as a sample `.mldatx` file for validating SDI API usage

### Established Patterns
- Scripts use `%[text]` Live Script format with `%%` section breaks
- Config structs are common (e.g., `params` struct in parameter scripts)
- `try/catch` used in robust functions (e.g., UKF Cholesky fallback)
- File naming: `RRpendulum_*` pattern, but this script serves a different purpose — `score_competition.m` per ROADMAP

### Integration Points
- Script lives in `scripts/` directory
- Must call `openProject()` check per CLAUDE.md convention
- Uses `Simulink.sdi.*` API (requires Simulink toolbox on path)

</code_context>

<specifics>
## Specific Ideas

No specific requirements — open to standard approaches. The interactive loop (file dialog + text input + listdlg) follows common MATLAB instructor-tool patterns.

</specifics>

<deferred>
## Deferred Ideas

None — discussion stayed within phase scope.

</deferred>

---

*Phase: 01-sdi-loading-and-session-loop*
*Context gathered: 2026-04-02*
