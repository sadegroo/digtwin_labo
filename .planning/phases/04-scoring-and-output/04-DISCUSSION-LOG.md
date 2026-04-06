# Phase 4: Scoring and Output - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-04-06
**Phase:** 04-scoring-and-output
**Areas discussed:** Export path and naming, Tie-breaking detail, Leaderboard display, BLDC diagnostic depth

---

## Export path and naming

| Option | Description | Selected |
|--------|-------------|----------|
| Auto-save to data/ folder | Auto-generate: data/competition_results_YYYYMMDD_HHMM.csv and .xlsx. No prompt. | ✓ |
| Prompt scorer with uiputfile | File dialog lets scorer choose location and filename. | |
| Save next to last .mldatx file | Export to same folder as last loaded file. | |

**User's choice:** Auto-save to data/ folder
**Notes:** data/ is already gitignored. Instant export, no extra step.

---

## Tie-breaking detail

| Option | Description | Selected |
|--------|-------------|----------|
| Equal rank, full points | Both tied teams get higher rank's points. Standard sports convention. | ✓ |
| Equal rank, averaged points | Split the tied positions' points. Fractional points may confuse students. | |
| You decide | Claude picks the most defensible approach. | |

**User's choice:** Equal rank, full points
**Notes:** Standard sports convention. E.g., 2 teams tie for 1st -> both get 2pt, next team gets 3rd.

---

## Leaderboard display

| Option | Description | Selected |
|--------|-------------|----------|
| Command window table only | disp(table) in command window. Clean, matches minimal-UI style. | |
| Command window + simple bar chart | Table plus grouped bar chart showing score breakdown per team. | ✓ |
| You decide | Claude picks based on script style. | |

**User's choice:** Command window + simple bar chart
**Notes:** Bar chart adds visual impact for showing students their results.

---

## BLDC diagnostic depth

| Option | Description | Selected |
|--------|-------------|----------|
| Same table, rank = N/A | BLDC in same table, Rank = N/A, TimePoints = '-'. Single unified view. | |
| Separate BLDC section | Stepper leaderboard first, then separate BLDC section below. | ✓ |
| You decide | Claude picks clearest presentation. | |

**User's choice:** Separate BLDC section
**Notes:** Clearer separation since stepper and BLDC use different scoring systems.

## Claude's Discretion

- Bar chart color scheme and layout
- Table formatting details
- Edge case handling (0 attempts)
- Optional .mat file export alongside CSV/xlsx

## Deferred Ideas

- VIZ-01 full polished stacked bar chart -- partially addressed with simple chart, full version v2
- VIZ-02 interpolation mode selection -- v2
