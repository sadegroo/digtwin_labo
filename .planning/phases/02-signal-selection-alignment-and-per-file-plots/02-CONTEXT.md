# Phase 2: Signal Selection, Alignment, and Per-File Plots - Context

**Gathered:** 2026-04-03
**Status:** Ready for planning

<domain>
## Phase Boundary

This phase delivers interactive signal selection, time alignment, and per-attempt visualization. For each loaded `.mldatx` file, the scorer picks the correct signals (accel/torque command + q2) from the hardware run, the script finds matching signals in the simulation run, aligns both runs to t=0 at the first nonzero command (with optional manual delta for hardware dead time), and displays a 3-subplot overlay plot with a dropdown to browse previous attempts. No metrics, scoring, or leaderboard computation happens here — those are Phases 3-4.

</domain>

<decisions>
## Implementation Decisions

### Signal Selection Flow
- **D-01:** Scorer picks 2 signals from the **hardware run only**: one accel/torque command signal and one q2 signal. The script then finds matching signal names in the simulation run automatically.
- **D-02:** Signal list presented via `listdlg` with keyword-based scoring. Each signal name is scored against keyword lists (`'accel','torque','cmd'` for command; `'q2','theta','pend'` for angle). Sorted by score descending, then alphabetical. Likely candidates float to top.
- **D-03:** If a team already has a previous attempt with signal mappings, **offer the previous signal names as default** (pre-select in listdlg). Scorer confirms or re-picks. Fresh selection for first attempt of each team.

### Start-Time Detection
- **D-04:** Start time = first sample where accel/torque command is **nonzero** (no percentage threshold, no consecutive-sample hysteresis). Simple and deterministic. Applied independently to each run (hw and sim).
- **D-05:** After auto-detection, scorer is offered a **manual time delta** (seconds) to left-shift the hardware signal. Compensates for hardware dead time between accel command and q2 response. Default = 0, press Enter to accept.
- **D-06:** Manual delta is **per-attempt with team default**: pre-filled with the last delta used for that team, scorer can accept or change.

### Alignment & Resampling
- **D-07:** After alignment (t=0 + manual delta), **crop both signals to their common time overlap**. No extrapolation, no NaN-padding.
- **D-08:** Normally both hw and sim are at **1 kHz** — no resampling needed. If rates differ, resample sim onto hw time grid via `interp1`. Defensive code, not the normal path.

### Overlay Plot Design
- **D-09:** Per-attempt plot has **3 subplots** sharing a common aligned time axis: (1) top: hw q2 vs sim q2 overlay, (2) middle: accel/torque command signal, (3) bottom: q2 difference (hw - sim).
- **D-10:** Single persistent figure window, **reused** for each new attempt. A **uicontrol dropdown** on the figure lists all loaded attempts (team name + filename). Selecting a previous attempt re-draws the 3 subplots.
- **D-11:** The plot shows the detected start time (t=0) and any manual delta as annotations/markers.

### Data Storage
- **D-12:** After signal selection and alignment, store the aligned time vector, aligned q2 signals (hw and sim), aligned command signal, detected start times, and manual delta in the attempt struct. This data feeds Phase 3 metric computation.

### Claude's Discretion
- Exact keyword lists and scoring weights for signal sorting — researcher should check actual signal names in example `.mldatx` files
- uicontrol dropdown implementation details (position, callback style)
- Color scheme and line styles for the overlay plot
- Interpolation method for `interp1` when resampling is needed (linear is fine)
- Whether the SIGM-03 preview plot (before confirming signal selection) is a separate figure or the same figure as the overlay plot

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### SDI API
- No external spec — use MATLAB R2025b documentation for `Simulink.sdi.*` API (getRun, getSignalsByIndex, getSignalByIndex, signal data extraction via `Simulink.sdi.Signal` properties)

### Project Context
- `.planning/REQUIREMENTS.md` — SIGM-01, SIGM-02, SIGM-03, ALGN-01, ALGN-02, OUTP-03 define acceptance criteria
- `.planning/PROJECT.md` — Competition context, angle convention (q2=0 down, q2=±π inverted), sample rate (1 kHz hw)
- `.planning/ROADMAP.md` — Phase 2 success criteria and dependencies

### Phase 1 Output
- `.planning/phases/01-sdi-loading-and-session-loop/01-CONTEXT.md` — Decisions D-01 through D-09 (run discrimination, session loop, team management, attempt struct)
- `scripts/score_competition.m` — Current script with session loop, `load_attempt` function, attempt struct layout

### Codebase
- `CLAUDE.md` — Live Script `.m` format conventions, project open check
- `example_data/combined_test.mldatx` — Example `.mldatx` file (may contain real signal names for testing keyword heuristic)

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `scripts/score_competition.m` — Phase 1 output with session loop and `load_attempt` function. Phase 2 logic inserts between the `load_attempt` call and team assignment, plus adds the overlay plot after team assignment.
- `load_attempt` returns `attempt` struct with `hw_run_id`, `sim_run_id`, `file`, `metrics` — Phase 2 will extend this struct with aligned signal data.

### Established Patterns
- `listdlg` for interactive selection (used for team assignment in Phase 1)
- `fprintf` + `input()` for command-line interaction (used throughout Phase 1)
- `try/catch` with `warning()` for error handling
- Config in `cfg` struct at top of script

### Integration Points
- Phase 2 functions are local functions in `score_competition.m` (same pattern as `load_attempt`)
- Signal selection and alignment happen inside the session loop, after `load_attempt` and before team assignment
- The overlay plot figure is created once and reused; the dropdown callback needs access to all attempt data
- The attempt struct gains new fields: aligned time, aligned signals, start times, manual delta

</code_context>

<specifics>
## Specific Ideas

- Hardware dead time compensation: the scorer may need to left-shift the hw signal because the hardware responds with a consistent delay after the accel command is sent. The manual delta input handles this.
- Signal names vary between teams — keyword scoring must be tolerant (substring matching, case-insensitive).

</specifics>

<deferred>
## Deferred Ideas

- **Live leaderboard figure** — A second persistent figure showing a continuously updated leaderboard of best runs per metric, refreshed after each attempt. Belongs in Phase 4 (Scoring and Output) since it requires the scoring rubric.

</deferred>

---

*Phase: 02-signal-selection-alignment-and-per-file-plots*
*Context gathered: 2026-04-03*
