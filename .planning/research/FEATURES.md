# Feature Landscape

**Domain:** Competition scoring tool — MATLAB post-hoc grading script for pendulum swingup contest
**Researched:** 2026-04-02
**Confidence:** HIGH (requirements directly sourced from PROJECT.md + existing codebase analysis)

---

## Table Stakes

Features users expect. Missing = scoring is unfair, unreproducible, or breaks on any real submission.

| Feature | Why Expected | Complexity | Notes |
|---------|--------------|------------|-------|
| Load `.mldatx` via `Simulink.sdi.*` API | Only way to read the data format teams submit | Low | `Simulink.sdi.load`, `Simulink.sdi.Run` objects; must clear SDI workspace before each load to avoid run ID collisions |
| Separate hardware run from simulation run within each `.mldatx` | Each file has two runs: archived (hardware) and recent (simulation); mixing them breaks all metrics | Medium | Identify by run metadata (source tag or position); archive = hardware, recent = simulation — must be confirmed per-file because teams may have different SDI state when they export |
| Manual signal selection with smart sort | Signal names vary between teams; auto-detection is unreliable (confirmed out-of-scope in PROJECT.md) | Medium | Sort candidates: signals with substrings "accel", "torque", "cmd" float to top for accel_cmd; "q2", "theta", "pend" for pendulum angle. Present as numbered list; scorer picks by index |
| Start-time detection via first non-zero accel/torque command | Aligns hardware and simulation signals at a common "t=0 when control started" reference | Medium | Threshold must tolerate noise floor; use first sample where `abs(accel_cmd) > threshold`; threshold should be configurable (default ~1% of saturation limit) |
| Time-align hardware and simulation signals | Hardware clock and Simulink clock are independent; SMAPE on unaligned signals is meaningless | Medium | Shift simulation time so t=0 corresponds to same physical event as hardware; hardware reference is the non-zero torque event; simulation reference is the same |
| Swingup success check: q2 crosses ±pi AND holds within ±2 deg for >= 1 second | Definition of a successful swingup per PROJECT.md | Medium | Two sub-conditions: (1) crossing detector; (2) dwell timer. Both must pass. Wrap-around handling required for ±pi crossing (q2 may oscillate around pi with sign changes) |
| Participation check: |q2| > pi/2 at any point | 1-point participation score; minimum bar to show any meaningful swingup effort | Low | Simple `any(abs(q2) > pi/2)` after alignment |
| Swingup time computation | Primary metric for stepper team ranking | Medium | Time from t=0 (first non-zero accel) to first sample where |q2| >= pi - 2deg AND the dwell condition has been met; must handle case where no successful swingup occurred (return NaN or Inf) |
| SMAPE computation on q2 (hardware vs simulation) | Primary accuracy metric; measures how well the digital twin predicts hardware behavior | Medium | `SMAPE = (2/N) * sum(|hw - sim| / (|hw| + |sim|)) * 100`; guard against denominator = 0 by adding epsilon; interpolate one signal onto the other's time axis first (simulation may differ from 2 kHz hardware rate) |
| Configurable SMAPE window | PROJECT.md explicitly requires three window modes: fixed time, angle threshold (|q2| > pi/2), or until swingup completion | Medium | Enum/flag: `'fixed'` (e.g., first 5 s), `'participation'` (while |q2| > pi/2), `'swingup'` (until swingup success time). Default should be `'participation'` as it is the most forgiving and SMAPE can be scored from any attempt exceeding 90 deg |
| Best-per-metric scoring | Fastest time and best SMAPE may come from different `.mldatx` files within one team's submission | Medium | Collect `{time, smape}` from all attempts; pick min(time) ignoring NaN; pick min(smape) from attempts where participation=true; score each independently |
| Stepper team ranking scores (time: 1st=2pt, 2nd=1pt, 3rd=0.5pt, 4th=0pt; SMAPE: 1st=2pt, 2nd=1pt, 3rd-4th=0.5pt) | Defined scoring rubric per PROJECT.md | Low | Rank 4 stepper teams; handle ties (equal points to tied teams); NaN attempts get 0 pt by default |
| BLDC team absolute SMAPE scoring (0-40%=4pt, 40-80%=3pt, 80-120%=2pt, 120-160%=1pt, 160-200%=0pt) | BLDC team scored against absolute threshold, not competitively — only 1 BLDC team | Low | Simple lookup table; use best SMAPE across all BLDC attempts |
| Participation point (1pt if |q2| > pi/2) | Applied to all teams | Low | Already a prerequisite for SMAPE scoring; reuse the same flag |
| Final MATLAB table with all scores | Required output; must be machine-readable | Low | `table()` with columns: TeamName, SwingupTime, SMAPE, TimePoints, SMAPEPoints, ParticipationPoint, TotalPoints, Rank |
| CSV/Excel export | Required for sharing results with students and admins | Low | `writetable(T, 'leaderboard.csv')` and `writetable(T, 'leaderboard.xlsx')` |

---

## Differentiators

Features that set the tool apart from a minimal implementation. Not expected from the requirements, but valuable for instructor convenience and auditability.

| Feature | Value Proposition | Complexity | Notes |
|---------|-------------------|------------|-------|
| Overlay plot: hardware q2 vs simulation q2 per attempt | Instant visual sanity check; lets scorer spot misaligned runs or wrong signal picks before scores are finalized | Low | Two subplots per attempt: aligned q2 overlay, and residual (hw - sim). Use `tiledlayout` |
| Leaderboard summary figure | Single-figure ranked bar chart with score breakdown per team; useful for projecting to class at end of competition | Low | Stacked bar: TimePoints + SMAPEPoints + ParticipationPoint per team, sorted by TotalPoints |
| Signal preview before confirmation | Show a small plot of the candidate signals before scorer commits to a selection; prevents picking the wrong channel | Low | Plot selected accel_cmd and q2 candidates for ~2 s; ask "confirm? [y/n]" |
| Per-team diagnostic printout | Structured `fprintf` output per team: "Team X: N attempts loaded, best time = X s (attempt 2), best SMAPE = X% (attempt 1), participation = YES" | Low | Valuable for explaining grades to students; adds negligible complexity |
| Attempt validity flags in output table | Extra columns: `SwingupSuccess`, `ParticipationAchieved`, `SMAPEWindow` used for each team's best SMAPE | Low | Makes the scoring defensible; score cannot be disputed without a data argument |
| Graceful skip on corrupt/unloadable files | If one `.mldatx` fails to load, log the error and continue with remaining attempts; don't abort the entire run | Low | `try/catch` around per-file processing; record `Status = 'load_error'` in output |
| Interpolation mode selection | `interp1` method choice (linear vs. pchip) for aligning sim to hw time base; pchip may overshoot near sharp transitions | Low | Linear is safer default; expose as parameter `cfg.interp_method = 'linear'` |
| Config struct at top of script | Centralise all tunable thresholds (start-time threshold, dwell duration, SMAPE window mode, swingup angle tolerance) in one `cfg` struct at the top of the script | Low | Prevents magic numbers scattered through the code; easy for instructor to tune next year |

---

## Anti-Features

Features to deliberately NOT build. These are overengineering risks in a 5-team, one-day scoring context.

| Anti-Feature | Why Avoid | What to Do Instead |
|--------------|-----------|-------------------|
| Automatic signal name detection without manual confirmation | Out of scope per PROJECT.md; team naming conventions vary too much; false positives would cause wrong scores silently | Smart-sorted list + manual selection (table stakes item above) |
| Re-running simulations as part of scoring | Out of scope per PROJECT.md; takes minutes per attempt, requires matching design files, and introduces reproducibility risk | Trust logged `.mldatx` data as the authoritative record |
| GUI (App Designer) | 5 teams scored once; the overhead of building and maintaining a MATLAB App is not justified for a single event per year | Plain Live Script `.m` with printed prompts and plots; scorer interacts via command line |
| Real-time scoring during competition | Out of scope per PROJECT.md; data is inherently post-hoc (teams submit `.mldatx` files after their slot) | Post-competition batch processing |
| Database or persistent storage | 5 teams, one run of the script per year; no need for session state or cross-run comparisons | Single workspace + CSV/Excel export |
| Web dashboard or external sharing | Data is internal to the course; `.xlsx` export is sufficient for grade book upload | `writetable()` export |
| Weighted SMAPE variants (e.g., frequency-domain SMAPE) | Adds interpretive complexity with no fairness benefit over time-domain SMAPE; students cannot replicate without custom tooling | Standard time-domain SMAPE as defined in PROJECT.md |
| Video or visual verification | Out of scope per PROJECT.md; trust the sensor data | Logged q2 signals are the ground truth |
| Per-signal parametric noise filtering (Butterworth, Kalman) | Hardware data at 2 kHz is already clean enough for SMAPE; filtering would bias the comparison | Raw signal alignment is sufficient; if needed, a simple moving average is enough and should be optional |
| Unit tests or test harness | Tool runs once per competition; formal test infra is disproportionate; the live visual checks (signal preview, overlay plot) serve as the verification step | Manual sanity checks during scoring session |

---

## Feature Dependencies

```
Load .mldatx
  └── Separate hardware run / simulation run
        └── Manual signal selection (smart sort)
              ├── Start-time detection (accel/torque channel)
              │     └── Time alignment (hw + sim on same t=0)
              │           ├── Swingup success check
              │           │     └── Swingup time
              │           ├── Participation check
              │           └── SMAPE computation (configurable window)
              │                 └── Best-per-metric across attempts
              │                       ├── Stepper ranking scores
              │                       ├── BLDC absolute score
              │                       └── Participation point
              │                             └── Final MATLAB table
              │                                   └── CSV/Excel export
              └── [Differentiator] Signal preview before confirmation
                    └── [Differentiator] Overlay plots (q2 hw vs sim)
                          └── [Differentiator] Leaderboard figure
```

**Critical path:** Everything flows from correct signal selection. A wrong accel_cmd pick corrupts start-time, which corrupts alignment, which corrupts both swingup time and SMAPE. The signal selection + preview step is the highest-leverage point for catching user error.

---

## MVP Recommendation

Build in this order:

1. **Load + separate runs** — core data access; nothing works without it
2. **Manual signal selection with smart sort** — highest error-leverage point; get this right first
3. **Start-time detection + time alignment** — prerequisite for all metrics
4. **Swingup success + participation checks** — binary gates for subsequent scoring
5. **Swingup time + SMAPE computation** — the two primary metrics
6. **Best-per-metric aggregation** — needed before scoring
7. **Stepper ranking + BLDC absolute scoring** — the actual grade computation
8. **MATLAB table + CSV/Excel export** — required deliverable

Then add in order of instructor value:
9. **Overlay plots (q2 hw vs sim)** — sanity check; low cost, high benefit
10. **Signal preview before confirmation** — prevents the most likely source of scoring error
11. **Per-team diagnostic printout** — grading transparency
12. **Config struct at top** — makes the tool reusable next year without code edits

Defer permanently:
- Leaderboard bar chart (nice but marginal value over the printed table)
- Interpolation mode selection (linear default is sufficient)

---

## Sources

- `C:/Users/u0130154/MATLAB/projects/digtwin_labo/.planning/PROJECT.md` — Requirements source of truth (HIGH confidence)
- `C:/Users/u0130154/MATLAB/projects/digtwin_labo/.planning/codebase/ARCHITECTURE.md` — Signal conventions, state vector, data formats (HIGH confidence)
- `C:/Users/u0130154/MATLAB/projects/digtwin_labo/.planning/codebase/INTEGRATIONS.md` — SDI logging context, `.mldatx` data format (HIGH confidence)
- Codebase inspection of `scripts/` — existing script patterns, Live Script `.m` format conventions (HIGH confidence)
