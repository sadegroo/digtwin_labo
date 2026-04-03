# Phase 3: Metric Computation - Context

**Gathered:** 2026-04-03
**Status:** Ready for planning

<domain>
## Phase Boundary

This phase delivers metric computation for each loaded attempt: swingup success detection (with 1s hold requirement), swingup time extraction, participation check, and SMAPE between hardware and simulation q2. All metrics use correct angle wrapping, division-by-zero guards, and a configurable SMAPE window. Metrics are computed immediately per-attempt inside the session loop, before truncation. No scoring rubric, ranking, or leaderboard is applied here -- those are Phase 4.

</domain>

<decisions>
## Implementation Decisions

### Unit Normalization
- **D-01:** Convert aligned hw_q2 and sim_q2 to radians at the start of metric computation, regardless of the per-file display unit. All thresholds, angle wrapping formulas, and comparisons operate in radians.
- **D-02:** The conversion happens inside the `compute_metrics` function as its first step. The aligned struct (`attempt.aligned`) stays in its native unit for plotting. The caller does not need to pre-convert.

### Swingup Detection
- **D-03:** Swingup success requires `|abs(q2) - pi| < 2deg` (0.0349 rad) sustained for at least 1 continuous second. A brief crossing that falls back out does not count.
- **D-04:** Swingup time is the moment q2 **enters** the +/-2deg band at the start of the successful 1-second hold -- not just any crossing. The time is backdated to the entry of the sustained hold.
- **D-05:** Both +pi and -pi count as upright. The detection uses `abs(q2)` so the sign doesn't matter (they are the same physical position).

### Metrics Timing and Integration
- **D-06:** Metrics are computed immediately after alignment, **before** truncation. The session loop order becomes: load -> team -> signals -> align -> **compute_metrics** -> truncate -> store -> overlay.
- **D-07:** Metrics use the full (pre-truncation) aligned data so the 1s hold check and SMAPE window are not cut short by the truncation margin.
- **D-08:** After metric computation, print a compact one-liner: `"Metrics: swingup=YES t=3.42s SMAPE=12.3% participation=YES"`. Quick glance, does not clutter the session.

### SMAPE Window Policy
- **D-09:** SMAPE window = `max(5s, time_to_first_90deg)` where time_to_first_90deg is the first sample where `|q2| > pi/2` (90deg) on the hardware signal.
- **D-10:** If `|q2|` never reaches 90deg (pi/2) at any point in the attempt, skip SMAPE computation entirely and mark the attempt as **ineligible** for SMAPE scoring. Do not compute SMAPE for ineligible attempts.
- **D-11:** If participation is achieved (|q2| > pi/2), SMAPE is computed over the window and the attempt is marked as eligible.

### SMAPE Computation
- **D-12:** SMAPE uses the angular difference formula `mod(hw-sim+pi, 2*pi)-pi` to handle wrapping near +/-pi (METR-07). All computation in radians per D-01.
- **D-13:** Denominator guard: exclude samples where `|hw_q2| + |sim_q2| < epsilon`. Epsilon value is Claude's discretion (reasonable choice given 2kHz data and radian range).

### Participation
- **D-14:** Participation = `|q2| > pi/2` (90deg) at any point during the attempt on the hardware signal. This is a separate flag from SMAPE eligibility (though they share the same threshold).

### Claude's Discretion
- Epsilon value for SMAPE denominator guard (reasonable default given data precision)
- Internal function signature for `compute_metrics` (inputs: aligned struct, cfg, q2_unit; output: metrics struct)
- Whether participation check uses a consecutive-sample filter for noise robustness (METR-02 mentions 10 consecutive samples) or a simple any() check
- How to handle edge case where data is shorter than 5s (use available data)

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Requirements
- `.planning/REQUIREMENTS.md` -- METR-01 through METR-07 define acceptance criteria for this phase

### Project Context
- `.planning/PROJECT.md` -- Competition context, angle convention (q2=0 down, q2=+/-pi inverted), sample rate (2kHz hw)
- `.planning/ROADMAP.md` -- Phase 3 success criteria and dependencies

### Prior Phase Context
- `.planning/phases/01-sdi-loading-and-session-loop/01-CONTEXT.md` -- Session state struct layout, attempt.metrics placeholder (D-08)
- `.planning/phases/02-signal-selection-alignment-and-per-file-plots/02-CONTEXT.md` -- Aligned struct fields (D-12), start-time detection (D-04), truncation logic

### Current Script
- `scripts/score_competition.m` -- Phase 1+2 implementation. Key integration points:
  - `align_signals` returns the aligned struct (line ~532)
  - `truncate_at_swingup` runs after alignment (line ~205)
  - `attempt.metrics = struct()` placeholder in `load_attempt` (line ~334)
  - `cfg` struct at top defines all Phase 3 parameters (lines 21-26)
  - Session loop body (lines 67-244) where metrics insertion point is

### Codebase
- `CLAUDE.md` -- Live Script .m format conventions, project open check

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `scripts/score_competition.m` -- The script already has:
  - `cfg.smape_window`, `cfg.smape_fixed_duration`, `cfg.swingup_hold_time`, `cfg.swingup_tolerance_deg`, `cfg.participation_threshold` defined in the config section
  - `attempt.metrics = struct()` placeholder ready to receive Phase 3 output
  - `truncate_at_swingup` function with `|q2| >= threshold` crossing detection (reusable logic pattern)
  - `extract_signal`, `align_signals` producing the aligned struct consumed by metrics

### Established Patterns
- Local functions at the bottom of `score_competition.m` (same pattern for `compute_metrics`)
- `fprintf` for session feedback (compact one-liner matches existing style)
- `cfg` struct for all tunable parameters
- `try/catch` with `warning()` for error handling

### Integration Points
- Metrics computation inserts between `align_signals` (line 201) and `truncate_at_swingup` (line 205)
- The aligned struct from `align_signals` has fields: `t`, `hw_q2`, `sim_q2`, `hw_cmd`, `hw_t_start`, `sim_t_start`, `delta`
- `attempt.metrics` struct receives output; consumed by Phase 4 scoring
- `attempt.signals.q2_unit` provides the unit for rad conversion

</code_context>

<specifics>
## Specific Ideas

- SMAPE window is a hybrid: fixed 5s minimum, extended to first 90deg if that takes longer. This ensures slow swing-ups are still measured fairly.
- Ineligible attempts (never reach 90deg) skip SMAPE entirely -- cleaner than computing a meaningless number.
- Swingup time is backdated to band entry of the *successful* hold, not just any crossing. This rewards clean, controlled swing-ups.

</specifics>

<deferred>
## Deferred Ideas

None -- discussion stayed within phase scope.

</deferred>

---

*Phase: 03-metric-computation*
*Context gathered: 2026-04-03*
