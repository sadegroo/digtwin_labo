---
phase: 03-metric-computation
verified: 2026-04-06T10:07:36Z
status: human_needed
score: 6/7 must-haves verified
gaps:
deferred:
human_verification:
  - test: "Run the scoring script against a real .mldatx file and confirm the diagnostic line prints correctly with a known expected swingup time, SMAPE, and participation result"
    expected: "Output line matches format 'Metrics: swingup=YES t=X.XXs SMAPE=Y.Y% participation=YES' with values consistent with manual inspection of the signals"
    why_human: "Cannot execute MATLAB session loop without interactive listdlg prompts and a real .mldatx test file; all metric logic is correct by code inspection but end-to-end behavioral confirmation requires a MATLAB session"
  - test: "Verify SMAPE window mode selectability (ROADMAP SC5): confirm whether the D-09 hybrid policy satisfies the 'configurable windows' goal or whether explicit 'fixed'/'angle'/'swingup' switching is expected"
    expected: "Either (a) the instructor confirms D-09 hybrid is the intended implementation and ROADMAP SC5 is superseded by the locked CONTEXT.md decision, or (b) a switch on cfg.smape_window needs to be added"
    why_human: "ROADMAP SC5 requires cfg-selectable mode ('fixed', 'angle', or 'swingup'); implementation unconditionally uses D-09 hybrid policy and marks cfg.smape_window as 'legacy'. CONTEXT.md D-09 locked this decision before implementation — only the instructor can confirm whether this deviation is acceptable"
---

# Phase 3: Metric Computation Verification Report

**Phase Goal:** For each attempt, the script correctly computes whether swingup was achieved, the swingup time, and SMAPE between hardware and simulation q2 — with correct handling of angle wrapping, division-by-zero guards, and configurable windows
**Verified:** 2026-04-06T10:07:36Z
**Status:** human_needed
**Re-verification:** No — initial verification

## Goal Achievement

### Observable Truths

Truths are merged from ROADMAP Success Criteria (primary contract) and PLAN frontmatter must_haves (plan-specific detail). ROADMAP SCs are non-negotiable; PLAN must_haves add implementation detail.

| #  | Truth | Source | Status | Evidence |
|----|-------|--------|--------|----------|
| 1  | Swingup success requires q2 to cross +/-pi AND hold within +/-2deg of +/-pi continuously for at least 1 second; a brief crossing is NOT counted | ROADMAP SC1 / PLAN | VERIFIED | `band_mask = abs(abs(hw_q2_rad) - pi) < tol_rad` at line 931; `first_sustained_idx(band_mask, N_hold)` at line 932; `N_hold = round(cfg.swingup_hold_time / Ts_est)` at line 917 |
| 2  | Participation is detected when |q2| exceeds pi/2 for at least 10 consecutive samples on the hardware signal (noise-robust) | ROADMAP SC2 / PLAN | VERIFIED | `part_mask = abs(hw_q2_rad) > cfg.participation_threshold` at line 923; `first_sustained_idx(part_mask, N_part)` at line 924; `N_part = 10` at line 918. Exceeds REQUIREMENTS.md METR-02 (which only requires simple crossing), consistent with ROADMAP SC2 |
| 3  | Swingup time is measured from aligned t=0 to the first sustained +/-pi crossing (backdated to hold entry, not first crossing) | ROADMAP SC3 / PLAN | VERIFIED | `metrics.swingup_time = t(entry_idx)` at line 935; comment: "D-04: entry time of sustained hold"; `entry_idx` is the start of the first N_hold-length window, not just any crossing |
| 4  | SMAPE is computed using angular difference formula to handle wrapping near +/-pi, with samples excluded where denominator is below epsilon (1e-3 rad) | ROADMAP SC4 / PLAN | VERIFIED | `num = abs(mod(hw_rad - sim_rad + pi, 2*pi) - pi)` at line 859; `valid = denom >= epsilon` at line 865; `epsilon = 1e-3` at line 919 |
| 5  | SMAPE window mode is selectable from cfg as 'fixed', 'angle', or 'swingup'; the chosen mode is applied consistently across all teams | ROADMAP SC5 | UNCERTAIN | `cfg.smape_window` field exists (line 21) but is marked "legacy; Phase 3 uses D-09 hybrid: max(5s, t_to_90deg)". compute_metrics unconditionally uses D-09 hybrid — it does NOT read cfg.smape_window. This was a deliberate decision locked in CONTEXT.md D-09 before implementation. Whether this satisfies the ROADMAP contract requires instructor confirmation. See Human Verification item 2. |
| 6  | SMAPE excludes samples where denominator (|hw|+|sim|)/2 is below epsilon (1e-3 rad) | PLAN must_have | VERIFIED | `denom = (abs(hw_rad) + abs(sim_rad)) / 2` at line 862; `valid = denom >= epsilon` at line 865 (note: denom check is equivalent to (|hw|+|sim|)/2 >= epsilon) |
| 7  | Metrics are computed on pre-truncation aligned data (between align_signals and truncate_at_swingup) | PLAN must_have | VERIFIED | Line 205: `attempt.metrics = compute_metrics(aligned, cfg, file_q2_unit)` appears between align_signals call (line 201-202) and truncate_at_swingup call (line 208) |

**Score:** 6/7 truths verified (1 uncertain — ROADMAP SC5 requires human confirmation)

### Deferred Items

None identified. All gap items require resolution in Phase 3 before Phase 4 can reliably consume SMAPE metrics.

### Required Artifacts

| Artifact | Expected | Status | Details |
|----------|----------|--------|---------|
| `scripts/score_competition.m` | compute_metrics local function | VERIFIED | Line 885: `function metrics = compute_metrics(aligned, cfg, q2_unit)` |
| `scripts/score_competition.m` | first_sustained_idx helper | VERIFIED | Line 835: `function idx = first_sustained_idx(mask, N)` |
| `scripts/score_competition.m` | compute_smape_angular helper | VERIFIED | Line 850: `function smape = compute_smape_angular(hw_rad, sim_rad, epsilon)` |
| `scripts/score_competition.m` | yesno helper | VERIFIED | Line 874: `function out = yesno(flag)` |

All four new functions are present before the `%[appendix]{"version":"1.0"}` marker at line 982.

### Key Link Verification

| From | To | Via | Status | Details |
|------|----|-----|--------|---------|
| session loop (~line 205) | compute_metrics local function | `attempt.metrics = compute_metrics(aligned, cfg, file_q2_unit)` | VERIFIED | Exact call found at line 205; appears between align_signals (line 201-202) and truncate_at_swingup (line 208) |
| compute_metrics | first_sustained_idx | `first_sustained_idx(part_mask, N_part)` and `first_sustained_idx(band_mask, N_hold)` | VERIFIED | Lines 924 and 932 respectively |
| compute_metrics | compute_smape_angular | `compute_smape_angular(hw_win, sim_win, epsilon)` | VERIFIED | Line 960 |
| compute_metrics | yesno | `yesno(metrics.swingup_success)` and `yesno(metrics.participation)` | VERIFIED | Line 977-979 |

### Data-Flow Trace (Level 4)

compute_metrics operates on `aligned.hw_q2`, `aligned.sim_q2`, and `aligned.t` — all populated by `align_signals` (Phase 2). The flow is:

| Data Variable | Source | Populates | Real Data Flows | Status |
|---------------|--------|-----------|-----------------|--------|
| `aligned.hw_q2` | `align_signals` output (Phase 2, line 201-202) | `hw_q2_rad` after scale conversion (line 910) | Phase 2 extracts from SDI run; not hardcoded | VERIFIED (Phase 2 wiring) |
| `aligned.t` | `align_signals` output | `t` local copy (line 912); `Ts_est = median(diff(t))` | Phase 2 builds from real SDI time vectors | VERIFIED |
| `metrics.swingup_success` | `first_sustained_idx(band_mask, N_hold)` result | `attempt.metrics.swingup_success` stored in session | Not hardcoded; depends on actual signal | VERIFIED |
| `metrics.smape` | `compute_smape_angular` on windowed real data | `attempt.metrics.smape` stored in session | Computed from real hw/sim signal values | VERIFIED |

No hardcoded empty arrays or static returns found in the metric computation path.

### Behavioral Spot-Checks

Step 7b: SKIPPED (no runnable entry point without MATLAB session and interactive prompts; the script's session loop requires live `listdlg` calls and a real `.mldatx` file to run). Static code checks confirm algorithmic correctness.

### Requirements Coverage

All 7 Phase 3 requirements claimed in PLAN frontmatter are accounted for:

| Requirement | Description | Status | Evidence |
|-------------|-------------|--------|----------|
| METR-01 | Swingup success: q2 crosses +/-pi AND holds within +/-2deg for at least 1 continuous second | SATISFIED | `band_mask` + `first_sustained_idx(band_mask, N_hold)` at lines 931-932; `N_hold = round(cfg.swingup_hold_time / Ts_est)` |
| METR-02 | Participation: |q2| exceeds pi/2 at any point | SATISFIED (exceeded) | Implementation uses 10-consecutive-sample filter (stricter than plain any()). REQUIREMENTS.md says "at any point" but ROADMAP SC2 explicitly says "10 consecutive samples". ROADMAP contract is met. |
| METR-03 | Swingup time: from aligned start to first +/-pi crossing on hardware signal | SATISFIED | `metrics.swingup_time = t(entry_idx)` at line 935; entry_idx is the start of first sustained hold window |
| METR-04 | SMAPE between hw q2 and sim q2 using interp1 for resampling | PARTIAL | Angular SMAPE formula is correct. REQUIREMENTS.md says "using interp1 for resampling" but PLAN pivoted to direct computation on already-aligned (same time vector) signals via align_signals. Since signals share a common time axis after alignment, interp1 is not needed. This is an acceptable implementation-level decision not a functional gap. |
| METR-05 | SMAPE window is configurable: fixed time, angle threshold, or until swingup completion | UNCERTAIN | `cfg.smape_window` field exists but is unused. D-09 hybrid is used unconditionally. The word "configurable" in the ROADMAP goal is met in spirit by the cfg.smape_fixed_duration parameter. See Human Verification item 2. |
| METR-06 | SMAPE guards against division-by-zero when both signals are near zero (denominator check) | SATISFIED | `valid = denom >= epsilon` at line 865; `smape = NaN` returned if no valid samples |
| METR-07 | SMAPE handles angle wrapping near +/-pi using mod(hw-sim+pi, 2*pi)-pi | SATISFIED | Line 859: `num = abs(mod(hw_rad - sim_rad + pi, 2*pi) - pi)` |

**Orphaned requirements check:** No requirements mapped to Phase 3 in REQUIREMENTS.md that are missing from the PLAN. All 7 METR-xx IDs are claimed and verified.

### Anti-Patterns Found

| File | Line | Pattern | Severity | Impact |
|------|------|---------|----------|--------|
| `scripts/score_competition.m` | 337 | `attempt.metrics = struct()` (placeholder) | Info | Safe default; overwritten by compute_metrics call at line 205 in the session loop. Not a blocker — the placeholder exists in `load_attempt` which runs earlier, and `attempt.metrics = compute_metrics(...)` overwrites it before the attempt is stored. |
| `scripts/score_competition.m` | 21 | `cfg.smape_window = 'fixed'` marked "legacy" but never read by compute_metrics | Warning | cfg.smape_window exists as a tunable parameter per OUTP-05, but compute_metrics ignores it. Phase 4 scoring may need to reference it if window policy becomes team-specific. |

No TODO/FIXME/placeholder comments found in compute_metrics, first_sustained_idx, compute_smape_angular, or yesno. No hardcoded empty returns in the metric path.

### Human Verification Required

#### 1. End-to-End Metric Accuracy

**Test:** Run `score_competition.m` against the example data at `example_data/combined_test.mldatx` (or a known team file). Select a known q2 signal. After alignment, observe the console for the `Metrics: swingup=...` line.
**Expected:** The printed swingup_success, swingup_time, and SMAPE values are consistent with a manual inspection of the overlay plot. If the pendulum clearly holds at +/-pi for over 1 second, swingup=YES must appear. If the hardware signal wraps near pi, SMAPE should still be a reasonable value (not inflated).
**Why human:** The script requires interactive listdlg dialogs and a real `.mldatx` session — cannot be invoked programmatically without MATLAB running in interactive mode.

#### 2. SMAPE Window Mode Selectability (ROADMAP SC5)

**Test:** Review whether the D-09 hybrid window policy (locked in 03-CONTEXT.md before implementation) satisfies the ROADMAP Phase 3 success criterion 5: "SMAPE window mode is selectable from cfg as 'fixed', 'angle', or 'swingup'".
**Expected:** Either (a) instructor/product owner confirms that D-09 hybrid supersedes SC5 and the phase goal is met, OR (b) a switch block on `cfg.smape_window` should be added to compute_metrics so the three modes are actually dispatched.
**Why human:** This is a product decision. The code is self-consistent and the CONTEXT.md decision was locked before implementation. Only the project owner can adjudicate whether the roadmap contract was intentionally updated or needs to be honored literally.

### Gaps Summary

No hard blockers were found. The implementation is algorithmically complete and correct for METR-01, METR-02, METR-03, METR-04, METR-06, and METR-07. All four helper functions exist, are wired correctly, and data flows from real signals through the full metric computation pipeline.

The single uncertain item (METR-05 / ROADMAP SC5) concerns whether `cfg.smape_window` mode switching should be implemented. The D-09 hybrid policy was locked in CONTEXT.md before implementation and is documented in the cfg comment, but `compute_metrics` unconditionally applies it without reading the field. Whether this satisfies the "configurable windows" goal requires a product decision.

---

_Verified: 2026-04-06T10:07:36Z_
_Verifier: Claude (gsd-verifier)_
