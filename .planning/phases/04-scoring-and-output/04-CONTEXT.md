# Phase 4: Scoring and Output - Context

**Gathered:** 2026-04-06
**Status:** Ready for planning

<domain>
## Phase Boundary

When the scorer finalizes the session, the script applies the full scoring rubric across the accumulated session state (best-per-metric selection, stepper competitive ranking, BLDC absolute scoring, participation points), produces a ranked leaderboard table, exports to CSV and Excel, prints per-team diagnostic summaries, and displays a score breakdown bar chart. No changes to the session loop, signal selection, alignment, or metric computation -- those are Phases 1-3.

</domain>

<decisions>
## Implementation Decisions

### Best-Per-Metric Scoring
- **D-01:** For each team, select the fastest swingup time from any successful attempt and the lowest SMAPE from any attempt where participation was achieved (`smape_eligible == true`). These may come from different attempts (SCOR-01).
- **D-02:** If a team has no successful swingup across all attempts, their BestSwingupTime is `Inf` (or `NaN`) and TimePoints is 0.
- **D-03:** If a team has no SMAPE-eligible attempt, their BestSMAPE is `NaN` and SMAPEPoints is 0.

### Tie-Breaking Policy
- **D-04:** Equal rank, full points. When stepper teams tie on time or SMAPE, both tied teams receive the higher rank's points. E.g., 2 teams tie for 1st -> both get 2pt, next team gets 3rd place (0.5pt). Standard sports convention.
- **D-05:** Stepper time points: 1st=2pt, 2nd=1pt, 3rd=0.5pt, 4th=0pt.
- **D-06:** Stepper SMAPE points: 1st=2pt, 2nd=1pt, 3rd-4th=0.5pt (as specified in ROADMAP SC2).

### BLDC Scoring
- **D-07:** BLDC team receives absolute SMAPE score on bands: 0-40%=4pt, 40-80%=3pt, 80-120%=2pt, 120-160%=1pt, 160+%=0pt (SCOR-04).
- **D-08:** BLDC team is NOT ranked against stepper teams. It appears in a separate diagnostic section below the stepper leaderboard.
- **D-09:** BLDC team still receives a participation point (1pt if any attempt has `participation == true`).

### Leaderboard Display
- **D-10:** Command window table via `disp(table(...))` showing stepper teams only, with columns: Team, BestSwingupTime, BestSMAPE, TimePoints, SMAPEPoints, ParticipationPoint, TotalPoints, Rank.
- **D-11:** Separate BLDC section printed below the stepper table with: team name, best SMAPE, SMAPE band, SMAPE points, participation, total points.
- **D-12:** A grouped bar chart figure showing score breakdown per team (stepper + BLDC). Stacked or grouped bars with TimePoints, SMAPEPoints, ParticipationPoint segments. Simple and readable.

### Export
- **D-13:** Auto-save to `data/` folder with timestamped filename: `data/competition_results_YYYYMMDD_HHMM.csv` and `.xlsx`. No file dialog prompt.
- **D-14:** Both stepper and BLDC teams appear in the exported table (BLDC rank column = `NaN` or empty).

### Per-Team Diagnostics
- **D-15:** Print per-team summary to command window before the leaderboard: N attempts loaded, best swingup time, best SMAPE, participation status (OUTP-04).

### Claude's Discretion
- Bar chart color scheme and layout (grouped vs stacked bars)
- Exact table formatting and column widths
- How to handle the edge case of 0 attempts for a team (skip or show zeros)
- Whether to also save the session struct to a .mat file alongside CSV/xlsx

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Requirements
- `.planning/REQUIREMENTS.md` -- SCOR-01 through SCOR-06, OUTP-01, OUTP-02, OUTP-04, OUTP-05 define acceptance criteria

### Project Context
- `.planning/PROJECT.md` -- Competition context, 5 teams (4 stepper + 1 BLDC), core value (defensible grades)
- `.planning/ROADMAP.md` -- Phase 4 success criteria and point assignments

### Prior Phase Context
- `.planning/phases/01-sdi-loading-and-session-loop/01-CONTEXT.md` -- Session state struct layout, team struct fields, finalize keyword
- `.planning/phases/03-metric-computation/03-CONTEXT.md` -- Metrics struct fields (swingup_success, swingup_time, participation, smape, smape_eligible), D-09 hybrid window policy

### Current Script
- `scripts/score_competition.m` -- Phases 1-3 implementation. Key integration points:
  - Finalization stub at line 249 (replace with scoring logic)
  - `cfg` struct at lines 14-28 (add scoring parameters here)
  - `session.teams(i).attempts{j}.metrics` holds Phase 3 output
  - `cfg.teams(i).type` distinguishes 'stepper' from 'bldc'

### Codebase
- `CLAUDE.md` -- Live Script .m format conventions, project open check, git commit rules

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `scripts/score_competition.m` -- The script already has:
  - `cfg.teams` struct array with `name` and `type` fields (stepper/bldc discrimination)
  - `session.teams(i).attempts{j}.metrics` populated by Phase 3 with: `swingup_success`, `swingup_time`, `participation`, `smape`, `smape_eligible`, `smape_window_s`
  - Finalization section (line 249) with basic team summary loop -- replace with full scoring
  - `data/` folder exists and is gitignored

### Established Patterns
- Local functions at the bottom of `score_competition.m`
- `fprintf` for session feedback
- `cfg` struct for all tunable parameters
- `create_overlay_figure` / `update_overlay_figure` pattern for persistent figures (reuse for bar chart)

### Integration Points
- Scoring logic replaces the finalization stub (lines 249-264)
- New local functions: `compute_leaderboard`, `rank_stepper_teams`, `score_bldc_team`, `print_diagnostics`, `export_results`, `plot_score_breakdown`
- New cfg fields: point values, SMAPE bands, export path template
- Output: `data/competition_results_*.csv` and `.xlsx`

</code_context>

<specifics>
## Specific Ideas

- Stepper leaderboard first, BLDC section below -- clear visual separation since they use different scoring systems.
- Bar chart adds visual impact for showing students their results. Keep it simple -- this is a one-time grading tool, not a dashboard.
- Auto-save to data/ avoids the extra step of a file dialog during a time-pressured grading session.

</specifics>

<deferred>
## Deferred Ideas

- VIZ-01 (stacked bar chart with score breakdown) is partially addressed here with the simple bar chart. Full polished version deferred to v2.
- VIZ-02 (interpolation mode selection) -- deferred to v2.

</deferred>

---

*Phase: 04-scoring-and-output*
*Context gathered: 2026-04-06*
