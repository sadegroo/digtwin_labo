# Phase 2: Signal Selection, Alignment, and Per-File Plots - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-04-03
**Phase:** 02-signal-selection-alignment-and-per-file-plots
**Areas discussed:** Signal selection flow, Start-time detection, Alignment & resampling, Overlay plot design

---

## Signal Selection Flow

### How many signal picks per run?

| Option | Description | Selected |
|--------|-------------|----------|
| Two picks per run | Scorer picks accel/torque command AND q2 from each run (hw and sim). 4 picks total. | |
| Two picks from hw, reuse for sim | Scorer picks from hw run only. Script finds matching signal names in sim automatically. | ✓ |
| One pick per run | Only pick q2. Auto-detect accel/torque command. | |

**User's choice:** Two picks from hw, reuse for sim
**Notes:** None

### Signal mapping persistence across attempts?

| Option | Description | Selected |
|--------|-------------|----------|
| Ask every time | Fresh signal selection per file. | |
| Offer previous mapping | Pre-fill with previous signal names for same team. Scorer confirms or re-picks. | ✓ |
| Auto-reuse silently | Reuse automatically for all subsequent attempts of same team. | |

**User's choice:** Offer previous mapping (Recommended)
**Notes:** None

### Smart-sort heuristic?

| Option | Description | Selected |
|--------|-------------|----------|
| Keyword scoring | Score signal names against keyword lists. Sort by score descending. | ✓ |
| Two-group split | Split into 'likely' and 'other' groups with separator. | |
| No sorting | Alphabetical only. | |

**User's choice:** Keyword scoring (Recommended)
**Notes:** None

---

## Start-Time Detection

### Threshold basis?

| Option | Description | Selected |
|--------|-------------|----------|
| % of peak command | 5% of max(abs(accel_cmd)). Adaptive. | |
| % of saturation limit | Use hardware saturation limit from cfg. | |
| Absolute threshold | Fixed value in cfg. | |

**User's choice:** Other — "First nonzero acceleration or torque command is the start time for the run"
**Notes:** No percentage threshold or hysteresis. Simple: first sample where command != 0.

### Scorer confirmation of detected start time?

| Option | Description | Selected |
|--------|-------------|----------|
| Show on preview plot | Mark t=0 with vertical line on preview. No separate confirmation. | |
| Explicit confirm prompt | Print start time, ask scorer to confirm or override. | |
| Silent, no confirmation | Detect and align without showing. | |

**User's choice:** Other — Manual time delta for hardware dead time compensation
**Notes:** User wants to adjust alignment manually because hardware responds with consistent delay. Manual delta (seconds) to left-shift hw signal, defaults to 0. Shows on preview plot for visual verification.

### Manual delta scope?

| Option | Description | Selected |
|--------|-------------|----------|
| Per-attempt with team default | Pre-fill with last delta for same team. Scorer can change. | ✓ |
| Per-attempt, always 0 default | Fresh prompt every time, default 0. | |
| Per-team, set once | Set on first attempt, reuse for all. | |

**User's choice:** Per-attempt with team default (Recommended)
**Notes:** None

---

## Alignment & Resampling

### Crop behavior after alignment?

| Option | Description | Selected |
|--------|-------------|----------|
| Crop to common overlap | Keep only time range where both signals have data. | ✓ |
| Keep full length, NaN-pad | Preserve original lengths, fill gaps with NaN. | |
| Crop to shorter signal | Crop both to shorter signal's length. | |

**User's choice:** Crop to common overlap (Recommended)
**Notes:** None

### Resampling method?

| Option | Description | Selected |
|--------|-------------|----------|
| Resample sim onto hw grid | interp1 to match hw time vector. | ✓ |
| Resample both to common grid | Uniform grid, resample both. | |
| Resample hw onto sim grid | Downsample hw to sim rate. | |

**User's choice:** Option 1, but noted that normally both are at 1 kHz (not 2 kHz). Resampling is defensive only.
**Notes:** Hardware sample rate is 1 kHz, not 2 kHz as originally stated in project docs. Resampling via interp1 only if rates differ.

---

## Overlay Plot Design

### Plot content?

| Option | Description | Selected |
|--------|-------------|----------|
| q2 overlay + command subplot | 2 subplots: q2 overlay + command signal. | |
| q2 overlay only | Single axes: hw q2 vs sim q2. | |
| Three subplots | q2 overlay + command + q2 difference. | ✓ |

**User's choice:** Three subplots
**Notes:** None

### Figure lifecycle?

| Option | Description | Selected |
|--------|-------------|----------|
| Persist, reuse same figure | One figure, updated each attempt. | |
| Persist, new figure each time | New figure per attempt. | |
| Auto-close after confirm | Show, wait, close. | |

**User's choice:** Other — Reuse same window with dropdown to browse previous attempts
**Notes:** User wants a uicontrol dropdown on the figure to select/browse previously loaded attempts. Initially suggested a MATLAB app but accepted dropdown-on-figure as sufficient.

### Full App Designer vs dropdown?

| Option | Description | Selected |
|--------|-------------|----------|
| Dropdown on figure | uicontrol popup listing all loaded attempts. | ✓ |
| Full App Designer app | Proper MATLAB app with panels and tabs. | |
| No browsing | Always show latest only. | |

**User's choice:** Dropdown on figure
**Notes:** User also asked about a live leaderboard figure — deferred to Phase 4.

---

## Claude's Discretion

- Keyword lists and scoring weights for signal sorting
- uicontrol dropdown implementation details
- Color scheme and line styles
- Interpolation method for interp1
- Preview plot implementation (separate vs same figure)

## Deferred Ideas

- **Live leaderboard figure** — Continuously updated best-per-metric leaderboard. Belongs in Phase 4 (requires scoring rubric).
