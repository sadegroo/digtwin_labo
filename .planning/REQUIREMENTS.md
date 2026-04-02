# Requirements: Swingup Competition Scoring Script

**Defined:** 2026-04-02
**Core Value:** Correctly and fairly score every team's swingup attempts — accurate time extraction, proper signal alignment, and transparent SMAPE computation — so grades are defensible.

## v1 Requirements

Requirements for initial release. Each maps to roadmap phases.

### Data Loading

- [ ] **LOAD-01**: Script loads `.mldatx` files via `Simulink.sdi.load` with `Simulink.sdi.clear` called before each file to prevent run contamination
- [ ] **LOAD-02**: Script separates hardware run (archived) from simulation run (recent) within each `.mldatx` file
- [ ] **LOAD-03**: Script gracefully skips corrupt or unloadable `.mldatx` files with `try/catch`, logging the error and continuing with remaining attempts
- [ ] **LOAD-04**: Script operates in an incremental session loop: scorer provides one `.mldatx` file path at a time and assigns it to a named team; script processes that file immediately (signal mapping, alignment, metrics) before waiting for the next
- [ ] **LOAD-05**: Script maintains a session state struct that accumulates per-team results across all loaded files; each new file appends to the team's attempt list without clearing prior results
- [ ] **LOAD-06**: Script provides a "finalize" command that the scorer issues when all files have been submitted; finalization triggers competitive ranking (stepper ranking, BLDC absolute scoring) and produces the final leaderboard

### Signal Mapping

- [ ] **SIGM-01**: Script presents all signals from a run in a selection dialog (`listdlg`), sorted so likely candidates (accel, torque, cmd, q2, theta, pend) appear first
- [ ] **SIGM-02**: Scorer manually picks which signal is the acceleration/torque command and which is q2 for each team
- [ ] **SIGM-03**: Script shows a preview plot of the selected signals before scorer confirms the mapping

### Time Alignment

- [ ] **ALGN-01**: Script detects start time as the first sample where `abs(accel_cmd) > threshold` (configurable threshold, default ~1% of saturation limit)
- [ ] **ALGN-02**: Script time-aligns hardware and simulation signals so t=0 corresponds to the first non-zero command in each

### Metric Computation

- [ ] **METR-01**: Script checks swingup success: q2 crosses ±π AND holds within ±2° of ±π for at least 1 continuous second
- [ ] **METR-02**: Script checks participation: |q2| exceeds π/2 at any point during the attempt
- [ ] **METR-03**: Script computes swingup time: from aligned start (first non-zero command) to first ±π crossing on the hardware signal
- [ ] **METR-04**: Script computes SMAPE between hardware q2 and simulation q2 using `interp1` for resampling (not timeseries arithmetic)
- [ ] **METR-05**: SMAPE window is configurable: fixed time (e.g., 5s), angle threshold (e.g., until |q2| > π/2), or until swingup completion
- [ ] **METR-06**: SMAPE computation guards against division-by-zero when both signals are near zero (denominator check)
- [ ] **METR-07**: SMAPE computation handles angle wrapping near ±π using angular difference formula (`mod(hw-sim+pi, 2*pi)-pi`)

### Scoring

- [ ] **SCOR-01**: Script uses best-per-metric scoring: fastest swingup time from any successful attempt, lowest SMAPE from any attempt where participation achieved (|q2| > π/2)
- [ ] **SCOR-02**: Script ranks 4 stepper teams on swingup time: 1st=2pt, 2nd=1pt, 3rd=0.5pt, 4th=0pt
- [ ] **SCOR-03**: Script ranks 4 stepper teams on SMAPE: 1st=2pt, 2nd=1pt, 3rd-4th=0.5pt
- [ ] **SCOR-04**: Script scores BLDC team on absolute SMAPE: 0-40%=4pt, 40-80%=3pt, 80-120%=2pt, 120-160%=1pt, 160-200%=0pt
- [ ] **SCOR-05**: Script awards 1 participation point to any team where |q2| > π/2
- [ ] **SCOR-06**: Script computes total points per team (max 5) and produces a ranked leaderboard

### Output

- [ ] **OUTP-01**: Script produces a MATLAB table with columns: Team, BestSwingupTime, BestSMAPE, TimePoints, SMAPEPoints, ParticipationPoint, TotalPoints, Rank
- [ ] **OUTP-02**: Script exports leaderboard to CSV and Excel (`.xlsx`) at finalization (when scorer issues the finalize command)
- [ ] **OUTP-03**: Script produces an overlay plot immediately after each file is processed: hardware q2 vs simulation q2 (aligned), so the scorer can visually verify each attempt before loading the next
- [ ] **OUTP-04**: Script prints per-team diagnostic summary: N attempts loaded, best time, best SMAPE, participation status
- [ ] **OUTP-05**: All tunable parameters (thresholds, SMAPE window mode, angle tolerances) are in a `cfg` struct at the top of the script

## v2 Requirements

Deferred to future release. Tracked but not in current roadmap.

### Visualization

- **VIZ-01**: Leaderboard summary figure (stacked bar chart with score breakdown per team)
- **VIZ-02**: Interpolation mode selection exposed as parameter (linear vs pchip)

## Out of Scope

Explicitly excluded. Documented to prevent scope creep.

| Feature | Reason |
|---------|--------|
| Automatic signal name detection | Signal names vary too much between teams; false positives silently corrupt scores |
| GUI (App Designer) | 5 teams scored once per year; plain script with `listdlg` is sufficient |
| Re-running simulations | Takes minutes, requires design files, introduces reproducibility risk |
| Real-time scoring during competition | Data is inherently post-hoc (teams submit after their slot) |
| Database or persistent storage | 5 teams, one run per year — workspace + CSV is enough |
| Frequency-domain SMAPE variants | Adds interpretive complexity with no fairness benefit |
| Parametric noise filtering | Hardware data at 2 kHz is clean enough; filtering biases the comparison |
| Formal test harness | Tool runs once per competition; visual checks serve as verification |

## Traceability

Which phases cover which requirements. Updated during roadmap creation.

| Requirement | Phase | Status |
|-------------|-------|--------|
| LOAD-01 | Phase 1 | Pending |
| LOAD-02 | Phase 1 | Pending |
| LOAD-03 | Phase 1 | Pending |
| LOAD-04 | Phase 1 | Pending |
| LOAD-05 | Phase 1 | Pending |
| LOAD-06 | Phase 1 | Pending |
| SIGM-01 | Phase 2 | Pending |
| SIGM-02 | Phase 2 | Pending |
| SIGM-03 | Phase 2 | Pending |
| ALGN-01 | Phase 2 | Pending |
| ALGN-02 | Phase 2 | Pending |
| OUTP-03 | Phase 2 | Pending |
| METR-01 | Phase 3 | Pending |
| METR-02 | Phase 3 | Pending |
| METR-03 | Phase 3 | Pending |
| METR-04 | Phase 3 | Pending |
| METR-05 | Phase 3 | Pending |
| METR-06 | Phase 3 | Pending |
| METR-07 | Phase 3 | Pending |
| SCOR-01 | Phase 4 | Pending |
| SCOR-02 | Phase 4 | Pending |
| SCOR-03 | Phase 4 | Pending |
| SCOR-04 | Phase 4 | Pending |
| SCOR-05 | Phase 4 | Pending |
| SCOR-06 | Phase 4 | Pending |
| OUTP-01 | Phase 4 | Pending |
| OUTP-02 | Phase 4 | Pending |
| OUTP-04 | Phase 4 | Pending |
| OUTP-05 | Phase 4 | Pending |

**Coverage:**
- v1 requirements: 29 total
- Mapped to phases: 29
- Unmapped: 0

---
*Requirements defined: 2026-04-02*
*Last updated: 2026-04-02 revised for incremental interactive workflow*
