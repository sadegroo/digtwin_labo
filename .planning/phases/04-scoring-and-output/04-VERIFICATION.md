---
phase: 04-scoring-and-output
verified: 2026-04-06T00:00:00Z
status: passed
score: 6/6 must-haves verified
re_verification: false
---

# Phase 4: Scoring and Output Verification Report

**Phase Goal:** When the scorer finalizes the session, the script applies the full scoring rubric across the accumulated session state, produces a ranked leaderboard table, exports to CSV and Excel, and prints per-team diagnostic summaries that make grades defensible
**Verified:** 2026-04-06
**Status:** passed
**Re-verification:** No — initial verification

## Goal Achievement

### Observable Truths

| # | Truth | Status | Evidence |
|---|-------|--------|----------|
| 1 | Best-per-metric scoring selects fastest swingup time and lowest SMAPE independently across all attempts per team | VERIFIED | `aggregate_best` uses `min(times, [], 'omitnan')` and `min(smapes, [], 'omitnan')` at lines 1044-1045; empty/all-NaN guard at lines 1048-1049 |
| 2 | Four stepper teams are ranked on time (2/1/0.5/0 pt) and on SMAPE (2/1/0.5/0.5 pt) with ties handled by equal rank assignment | VERIFIED | `cfg.time_points = [2, 1, 0.5, 0]` at line 31; `cfg.smape_points = [2, 1, 0.5, 0.5]` at line 32; `assign_points_dense` at line 1054 uses `ranks(k) = ranks(k-1)` for ties (line 1078) |
| 3 | BLDC team receives absolute SMAPE score (4/3/2/1/0 pt on 0-40/40-80/80-120/120-160/160+ % bands) and is not ranked against stepper teams | VERIFIED | `score_bldc_smape` at line 1101 uses `smape_pct < bands(k)` (upper-exclusive); `bands = [40,80,120,160]`, `band_pts = [4,3,2,1,0]`; BLDC `ranks` left as NaN in `compute_leaderboard` (line 1168) |
| 4 | Output MATLAB table has 8 required columns and is exported to both CSV and xlsx at finalization | VERIFIED | `compute_leaderboard` builds table with `VariableNames` `{'Team','BestSwingupTime','BestSMAPE','TimePoints','SMAPEPoints','ParticipationPoint','TotalPoints','Rank'}` at lines 1184-1188; `export_results` writes CSV (line 1303) and xlsx in try/catch (lines 1307-1313); both stepper and BLDC teams included |
| 5 | Per-team diagnostic summary prints N attempts loaded, best time, best SMAPE, and participation status to the command window | VERIFIED | `print_diagnostics` at line 1195 prints `n_att`, formatted `bt`, `bs`, and `yesno(has_p)` for each team; reuses `aggregate_best` |
| 6 | All tunable parameters (thresholds, SMAPE window mode, angle tolerances, team names, team types, scoring points) are defined in a cfg struct at the top of the script | VERIFIED | cfg struct at lines 14-36 contains team config, all Phase 3 thresholds, and Phase 4 scoring fields (`time_points`, `smape_points`, `bldc_smape_bands`, `bldc_smape_pts`, `export_dir`, `export_stem`) |

**Score:** 6/6 truths verified

### Required Artifacts

| Artifact | Expected | Status | Details |
|----------|----------|--------|---------|
| `scripts/score_competition.m` | 8 local scoring functions + finalization pipeline | VERIFIED | 1371 lines; all 8 functions present before `%[appendix]{"version":"1.0"}` at line 1371 |
| `aggregate_best` | Best-per-metric aggregation (SCOR-01) | VERIFIED | Line 1018; `min(..., 'omitnan')` pattern; NaN guard; returns 3 values |
| `assign_points_dense` | Dense ranking with sports-convention tie-breaking (SCOR-02, SCOR-03) | VERIFIED | Line 1054; forward-scan rank build; `ranks(k) = ranks(k-1)` for ties at line 1078 |
| `score_bldc_smape` | Absolute band scoring for BLDC (SCOR-04) | VERIFIED | Line 1101; upper-exclusive `smape_pct < bands(k)`; NaN returns 0; 160+% fallback |
| `compute_leaderboard` | Full scoring table (SCOR-01..06, OUTP-01) | VERIFIED | Line 1123; calls all scoring sub-functions; returns 8-column MATLAB table |
| `print_diagnostics` | Per-team summary printout (OUTP-04) | VERIFIED | Line 1195; prints N attempts, best time, best SMAPE, participation |
| `disp_leaderboard` | Stepper table + BLDC section display (D-10, D-11) | VERIFIED | Line 1232; `disp(T(stepper_mask, :))` at line 1243; "BLDC TEAM" header at line 1247 |
| `export_results` | CSV + xlsx + .mat export with timestamps (OUTP-02) | VERIFIED | Line 1285; `datetime('now', 'Format', 'yyyyMMdd_HHmm')` at line 1299; two `writetable` calls; `warning('scorer:xlsxfail', ...)` try/catch; `save(mat_file, 'session', 'T')` at line 1318 |
| `plot_score_breakdown` | Grouped bar chart of score components (D-12) | VERIFIED | Line 1326; `bar(score_matrix, 'grouped')` at line 1340; legend with 'Time Points', 'SMAPE Points', 'Participation' at line 1355; total annotations per team |

### Key Link Verification

| From | To | Via | Status | Details |
|------|----|-----|--------|---------|
| Finalization block | `print_diagnostics`, `compute_leaderboard`, `disp_leaderboard`, `export_results`, `plot_score_breakdown` | Sequential calls at lines 266-290 | WIRED | Steps 1-5 in correct order; Steps 1, 4, 5 wrapped in try/catch per SUMMARY deviation note |
| `compute_leaderboard` | `aggregate_best` | `aggregate_best(session.teams(i))` at line 1146 | WIRED | Loops over all N teams |
| `compute_leaderboard` | `assign_points_dense` | `assign_points_dense(best_times(stepper_mask), cfg.time_points)` at line 1152 | WIRED | Called for both time and SMAPE |
| `compute_leaderboard` | `score_bldc_smape` | `score_bldc_smape(best_smapes(i), cfg.bldc_smape_bands, cfg.bldc_smape_pts)` at line 1160 | WIRED | Called for each BLDC team |
| `export_results` | `data/competition_results_*.csv` and `.xlsx` | `writetable(T, [base '.csv'])` at 1303; `writetable(T, [base '.xlsx'])` at 1308 | WIRED | Timestamp via `datetime('now', 'Format', 'yyyyMMdd_HHmm')` |
| `print_diagnostics` | `aggregate_best` | `[bt, bs, has_p] = aggregate_best(t)` at line 1205 | WIRED | Reuses aggregation logic correctly |

### Data-Flow Trace (Level 4)

| Artifact | Data Variable | Source | Produces Real Data | Status |
|----------|--------------|--------|-------------------|--------|
| `compute_leaderboard` | `best_times`, `best_smapes` | `aggregate_best` iterates `session.teams(i).attempts{j}.metrics` (populated by Phase 3 `compute_metrics` at line 213) | Yes — real attempt data from SDI-loaded signals | FLOWING |
| `export_results` | Table `T` | Passed as argument from `compute_leaderboard` return value | Yes — full 8-column table with all teams | FLOWING |
| `plot_score_breakdown` | `score_matrix` | `[T.TimePoints, T.SMAPEPoints, T.ParticipationPoint]` from table T | Yes — computed from real scoring data | FLOWING |

### Behavioral Spot-Checks

Step 7b: SKIPPED — script requires interactive MATLAB session (`listdlg`, `uigetfile`, `input`); no headless entry point. Human verification was completed as Task 2 of 04-02-PLAN.md (blocking gate, confirmed done per SUMMARY `<done>` tag).

### Requirements Coverage

| Requirement | Source Plan | Description | Status | Evidence |
|-------------|------------|-------------|--------|----------|
| SCOR-01 | 04-01-PLAN.md | Best-per-metric scoring across all attempts | SATISFIED | `aggregate_best` uses `min(..., 'omitnan')` on times and smapes arrays |
| SCOR-02 | 04-01-PLAN.md | Stepper time ranking 2/1/0.5/0 pt | SATISFIED | `cfg.time_points = [2, 1, 0.5, 0]`; `assign_points_dense` called with stepper time subset |
| SCOR-03 | 04-01-PLAN.md | Stepper SMAPE ranking 2/1/0.5/0.5 pt | SATISFIED | `cfg.smape_points = [2, 1, 0.5, 0.5]`; `assign_points_dense` called with stepper SMAPE subset |
| SCOR-04 | 04-01-PLAN.md | BLDC absolute SMAPE band scoring 4/3/2/1/0 pt | SATISFIED | `score_bldc_smape` with `bands=[40,80,120,160]`, upper-exclusive boundaries; 160+%=0pt fallback (aligned with ROADMAP SC3 and CONTEXT D-07) |
| SCOR-05 | 04-01-PLAN.md | 1 participation point per team with |q2|>pi/2 | SATISFIED | `part_pts(i) = double(part_flags(i))` in `compute_leaderboard`; `part_flag` set when `m.participation == true` in `aggregate_best` |
| SCOR-06 | 04-01-PLAN.md | Total points = time + SMAPE + participation | SATISFIED | `total_pts = time_pts + smape_pts + part_pts` at line 1165 |
| OUTP-01 | 04-01-PLAN.md | MATLAB table with 8 columns | SATISFIED | `compute_leaderboard` returns table with `VariableNames` matching all 8 required columns exactly |
| OUTP-02 | 04-02-PLAN.md | Export to CSV and xlsx at finalization | SATISFIED | `export_results` writes both formats; xlsx in try/catch; `export_results` called at Step 4 in finalization pipeline |
| OUTP-04 | 04-01-PLAN.md | Per-team diagnostic summary | SATISFIED | `print_diagnostics` prints N attempts, best time, best SMAPE, participation for each team |
| OUTP-05 | 04-01-PLAN.md | All tunable parameters in cfg struct | SATISFIED | cfg at lines 14-36 includes team config, all thresholds, and all Phase 4 scoring parameters |

**Orphaned Phase 4 requirements check:** REQUIREMENTS.md traceability table maps SCOR-01 through SCOR-06, OUTP-01, OUTP-02, OUTP-04, OUTP-05 to Phase 4 — all 10 are claimed by the plans and verified above. No orphaned requirements.

### Anti-Patterns Found

| File | Line | Pattern | Severity | Impact |
|------|------|---------|----------|--------|
| `scripts/score_competition.m` | 366 | `attempt.metrics = struct()` — placeholder comment | Info | Not a blocker — this initialization value is unconditionally overwritten by `compute_metrics` at line 213 in the session loop; SUMMARY explicitly notes this is intentional |

No blockers or warnings found. The single info-level pattern is benign initialization.

### Human Verification Required

Human verification was completed as a blocking checkpoint (Task 2 of 04-02-PLAN.md). The `<done>` tag in the plan records the user confirmed: "diagnostics print correctly, stepper leaderboard table has 8 columns, BLDC section appears separately, CSV/xlsx exported to data/, bar chart shows score breakdown for all teams." No additional human verification is needed.

### Gaps Summary

No gaps found. All 6 roadmap success criteria are verified by direct code inspection. All 10 requirement IDs claimed by the phase plans (SCOR-01 through SCOR-06, OUTP-01, OUTP-02, OUTP-04, OUTP-05) are satisfied with implementation evidence. All 8 local functions are substantive, wired into the finalization pipeline, and consume real data from Phase 3's `compute_metrics` output. The `%[appendix]{"version":"1.0"}` marker remains the last line of the file (line 1371).

---

_Verified: 2026-04-06_
_Verifier: Claude (gsd-verifier)_
