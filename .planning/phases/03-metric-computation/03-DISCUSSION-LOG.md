# Phase 3: Metric Computation - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md -- this log preserves the alternatives considered.

**Date:** 2026-04-03
**Phase:** 03-metric-computation
**Areas discussed:** Unit normalization, Swingup detection, Metrics timing, SMAPE window policy

---

## Unit Normalization

### Q1: When should unit conversion happen?

| Option | Description | Selected |
|--------|-------------|----------|
| Convert to rad on entry | Convert at start of metric computation. All formulas in rad. Display unit stays for plots. | Yes |
| Compute in native units | Adapt thresholds/formulas per unit. More complex. | |
| Store both units | Keep rad copy alongside native copy in attempt struct. | |

**User's choice:** Convert to rad on entry (Recommended)
**Notes:** None

### Q2: Where should the rad conversion live?

| Option | Description | Selected |
|--------|-------------|----------|
| Inside compute_metrics | Function converts internally as first step. Aligned struct stays native. | Yes |
| Before calling metrics | Convert in session loop, add .hw_q2_rad fields. | |

**User's choice:** Inside compute_metrics (Recommended)
**Notes:** None

---

## Swingup Detection

### Q3: How should 'crossing +/-pi' be detected?

| Option | Description | Selected |
|--------|-------------|----------|
| abs(q2) >= pi | Treat +/-pi as same position. Simple, matches truncate_at_swingup. | |
| Track sign separately | Detect which sign was crossed, check hold around that sign. | |

**User's choice:** Other -- "use the 1% band (so if the angle is between 180 deg +/- 1.8 degree for more than 1 second, it means that when it entered the band, a swingup was completed. 1s stability is a requirement"
**Notes:** User initially proposed 1% of 180deg = 1.8deg tolerance. After follow-up, settled on +/-2deg (matching METR-01).

### Q4: Confirm 1% band (+/-1.8deg) with 1s hold?

| Option | Description | Selected |
|--------|-------------|----------|
| Yes, exactly | 1% band, 1s hold, swingup time = entry into successful hold | |
| Adjust tolerance | Different percentage or absolute tolerance | Yes |

**User's choice:** Adjust tolerance

### Q5: What tolerance around +/-pi?

| Option | Description | Selected |
|--------|-------------|----------|
| +/-2deg (as in METR-01) | Original requirement. Clear, round number. | Yes |
| +/-1% of full rotation | +/-3.6deg. More forgiving. | |
| +/-1.8deg (1% of 180deg) | Tighter than METR-01. | |

**User's choice:** +/-2deg (as in METR-01)
**Notes:** None

### Q6: Should metrics use full or truncated data?

| Option | Description | Selected |
|--------|-------------|----------|
| Metrics on full data | Compute on pre-truncation aligned data. Truncation only for plotting/SMAPE bounds. | Yes |
| Metrics on truncated data | Use truncated data. Risk: 1s hold might get cut off. | |

**User's choice:** Metrics on full data (Recommended)
**Notes:** None

---

## Metrics Timing

### Q7: When should metrics be computed?

| Option | Description | Selected |
|--------|-------------|----------|
| Immediately per attempt | Compute after alignment, before truncation. Scorer sees results before next file. | Yes |
| Deferred to finalization | Compute when scorer types 'done'. No per-file feedback. | |

**User's choice:** Immediately per attempt (Recommended)
**Notes:** None

### Q8: What should the metric printout look like?

| Option | Description | Selected |
|--------|-------------|----------|
| Compact one-liner | "Metrics: swingup=YES t=3.42s SMAPE=12.3% participation=YES" | Yes |
| Detailed block | Multi-line box with labeled fields. | |
| You decide | Claude picks the format. | |

**User's choice:** Compact one-liner (Recommended)
**Notes:** None

---

## SMAPE Window Policy

### Q9: Which SMAPE window mode for the competition?

| Option | Description | Selected |
|--------|-------------|----------|
| Fixed time | SMAPE over first 5s. Fair, transparent. | |
| Until swingup | SMAPE until hold band entry. Problem with failed swingups. | |
| Angle threshold | SMAPE until |q2| > threshold. Ignores stabilization. | |
| Let scorer choose | All modes available, scorer picks at startup. | |

**User's choice:** Other -- "fixed time 5s, but only counts towards SMAPE leaderboard if angle of more than 90 degrees was achieved within these 5s"
**Notes:** Initially proposed 5s fixed with participation gate. Evolved through follow-up.

### Q10: Confirm fixed 5s with participation gate within window?

| Option | Description | Selected |
|--------|-------------|----------|
| Yes, exactly | Compute over 5s, eligible only if |q2|>90deg within window. | |
| Participation check is global | SMAPE counts if |q2|>90deg at any point in attempt, not just within 5s. | |

**User's choice:** Other -- "if it takes longer than 5s to reach 90 degrees, SMAPE window is extended until this angle is reached. if never reached => ineligible"
**Notes:** Window = max(5s, time_to_first_90deg). If 90deg never reached, skip SMAPE entirely.

### Q11: Confirm window = max(5s, first 90deg)?

| Option | Description | Selected |
|--------|-------------|----------|
| Yes, that's it | Window = max(5s, first_90deg_time). Ineligible if never reached. | |
| No, let me clarify | Different logic. | |

**User's choice:** Other -- "If never reached, just not compute SMAPE and call the run ineligible for SMAPE scoring"
**Notes:** Confirmed: skip SMAPE entirely for ineligible attempts.

### Q12: SMAPE denominator epsilon?

| Option | Description | Selected |
|--------|-------------|----------|
| 1e-3 rad (as in METR-06) | Skip samples where |hw|+|sim| < 1e-3 rad. | |
| You decide | Claude picks reasonable epsilon. | Yes |

**User's choice:** You decide
**Notes:** None

---

## Claude's Discretion

- Epsilon value for SMAPE denominator guard
- Internal function signature for compute_metrics
- Participation consecutive-sample filter vs simple any() check
- Edge case handling when data < 5s

## Deferred Ideas

None -- discussion stayed within phase scope.
