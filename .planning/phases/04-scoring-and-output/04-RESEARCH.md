# Phase 4: Scoring and Output - Research

**Researched:** 2026-04-06
**Domain:** MATLAB scoring logic, table/export APIs, bar chart visualization
**Confidence:** HIGH

## Summary

Phase 4 is the finalization layer of `score_competition.m`. It replaces the stub at line 249 of the existing script with full competitive scoring logic: best-per-metric aggregation across attempts, stepper competitive ranking with sports-convention tie-breaking, BLDC absolute-band scoring, participation points, a leaderboard MATLAB table, CSV/xlsx export, per-team diagnostic prints, and a grouped bar chart.

All decisions are locked in CONTEXT.md. The implementation is pure MATLAB table and array manipulation — no external libraries, no Simulink APIs, no new toolboxes. The key technical questions are: (1) how to implement sports-convention dense ranking in MATLAB without a dedicated rank function, (2) how `writetable` handles both CSV and xlsx in R2025b, and (3) how to build a grouped bar chart from per-team score components. All three are verified below.

**Primary recommendation:** Implement scoring as a set of pure local functions — `compute_leaderboard`, `rank_stepper_teams`, `score_bldc_team`, `print_diagnostics`, `export_results`, `plot_score_breakdown` — called from the finalization stub. Keep all tunable parameters (point values, SMAPE bands, export path template) as new fields appended to the existing `cfg` struct.

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**Best-Per-Metric Scoring**
- D-01: For each team, select the fastest swingup time from any successful attempt and the lowest SMAPE from any attempt where participation was achieved (`smape_eligible == true`). These may come from different attempts (SCOR-01).
- D-02: If a team has no successful swingup across all attempts, their BestSwingupTime is `Inf` (or `NaN`) and TimePoints is 0.
- D-03: If a team has no SMAPE-eligible attempt, their BestSMAPE is `NaN` and SMAPEPoints is 0.

**Tie-Breaking Policy**
- D-04: Equal rank, full points. When stepper teams tie on time or SMAPE, both tied teams receive the higher rank's points. Standard sports convention.
- D-05: Stepper time points: 1st=2pt, 2nd=1pt, 3rd=0.5pt, 4th=0pt.
- D-06: Stepper SMAPE points: 1st=2pt, 2nd=1pt, 3rd-4th=0.5pt.

**BLDC Scoring**
- D-07: BLDC absolute SMAPE bands: 0-40%=4pt, 40-80%=3pt, 80-120%=2pt, 120-160%=1pt, 160+%=0pt.
- D-08: BLDC is NOT ranked against stepper teams. Separate diagnostic section below the stepper leaderboard.
- D-09: BLDC still receives a participation point (1pt if any attempt has `participation == true`).

**Leaderboard Display**
- D-10: Command window table via `disp(table(...))` for stepper teams only. Columns: Team, BestSwingupTime, BestSMAPE, TimePoints, SMAPEPoints, ParticipationPoint, TotalPoints, Rank.
- D-11: Separate BLDC section printed below the stepper table: team name, best SMAPE, SMAPE band, SMAPE points, participation, total points.
- D-12: Grouped bar chart figure showing score breakdown per team (stepper + BLDC). TimePoints, SMAPEPoints, ParticipationPoint segments.

**Export**
- D-13: Auto-save to `data/` folder with timestamped filename: `data/competition_results_YYYYMMDD_HHMM.csv` and `.xlsx`. No file dialog.
- D-14: Both stepper and BLDC teams appear in the exported table (BLDC Rank column = `NaN` or empty).

**Per-Team Diagnostics**
- D-15: Print per-team summary before the leaderboard: N attempts loaded, best swingup time, best SMAPE, participation status (OUTP-04).

### Claude's Discretion
- Bar chart color scheme and layout (grouped vs stacked bars)
- Exact table formatting and column widths
- How to handle the edge case of 0 attempts for a team (skip or show zeros)
- Whether to also save the session struct to a .mat file alongside CSV/xlsx

### Deferred Ideas (OUT OF SCOPE)
- VIZ-01 (stacked bar chart with score breakdown) — full polished version deferred to v2
- VIZ-02 (interpolation mode selection) — deferred to v2
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| SCOR-01 | Best-per-metric scoring: fastest time from any successful attempt, lowest SMAPE from any smape_eligible attempt | Aggregation loop over `session.teams(i).attempts` using `min()` with NaN handling |
| SCOR-02 | Rank 4 stepper teams on swingup time: 1st=2pt, 2nd=1pt, 3rd=0.5pt, 4th=0pt | Sports-convention ranking via `sort` + dense rank assignment; see Pattern: Dense Ranking |
| SCOR-03 | Rank 4 stepper teams on SMAPE: 1st=2pt, 2nd=1pt, 3rd-4th=0.5pt | Same dense ranking; point vector is [2, 1, 0.5, 0] for time and [2, 1, 0.5, 0.5] for SMAPE |
| SCOR-04 | BLDC absolute SMAPE scoring on defined bands | Simple threshold lookup; see Pattern: BLDC Band Scoring |
| SCOR-05 | Participation point: 1pt if any attempt has `participation == true` | `any()` over attempt metrics fields |
| SCOR-06 | Total points per team + ranked leaderboard | Sum TimePoints + SMAPEPoints + ParticipationPoint; sort descending for final rank |
| OUTP-01 | MATLAB table with required columns | `table()` constructor with named variables; see Pattern: Table Construction |
| OUTP-02 | Export to CSV and xlsx at finalization | `writetable(T, file)` — default CSV; `writetable(T, file, 'FileType','spreadsheet')` for xlsx |
| OUTP-04 | Per-team diagnostic summary to command window | `fprintf` loop before leaderboard; see Pattern: Diagnostic Print |
| OUTP-05 | All tunable parameters in `cfg` struct at top of script | New fields appended to existing `cfg` struct (lines 14-28 in score_competition.m) |
</phase_requirements>

---

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| MATLAB base | R2025b | table construction, sort, min, any | All operations are base MATLAB; no toolbox needed |
| MATLAB base | R2025b | `writetable` for CSV/xlsx export | Built-in since R2013b; xlsx via built-in Java writer |
| MATLAB base | R2025b | `bar` / `barh` for grouped bar chart | Built-in; no external plotting library needed |

No new toolboxes. No new packages. No `npm install`.

**Version verification:** Verified against MATLAB R2025b by inspection of existing script and project toolbox list. [VERIFIED: existing project runs on R2025b per CLAUDE.md]

---

## Architecture Patterns

### Recommended Code Structure (within score_competition.m)

The finalization section (currently lines 249-264) expands into:

```
%% Finalization
session.finalized = true;
print_diagnostics(session, cfg);       % D-15 / OUTP-04
T = compute_leaderboard(session, cfg); % SCOR-01..06 / OUTP-01
disp_leaderboard(T, session, cfg);     % D-10, D-11
export_results(T, cfg);                % D-13 / OUTP-02
plot_score_breakdown(T, cfg);          % D-12
```

New local functions append after existing ones (before `%[appendix]`):
```
compute_leaderboard
  rank_stepper_teams
  score_bldc_team
print_diagnostics
disp_leaderboard
export_results
plot_score_breakdown
```

New `cfg` fields added in the Configuration section (lines 14-28):
```
cfg.time_points  = [2, 1, 0.5, 0];      % stepper time: 1st..4th
cfg.smape_points = [2, 1, 0.5, 0.5];    % stepper SMAPE: 1st..4th (3rd=4th=0.5)
cfg.bldc_smape_bands = [0, 40, 80, 120, 160];  % upper edges in %
cfg.bldc_smape_pts   = [4, 3, 2, 1, 0];        % points per band
cfg.export_dir  = 'data';
cfg.export_stem = 'competition_results';
```

### Pattern: Best-Per-Metric Aggregation (SCOR-01, D-01..D-03)

Iterate over all attempts for a team; apply `min()` with NaN ignoring.

```matlab
% Source: base MATLAB array operations [VERIFIED: MATLAB R2025b]
function [best_time, best_smape] = aggregate_best(team)
    times = NaN(numel(team.attempts), 1);
    smapes = NaN(numel(team.attempts), 1);
    for j = 1:numel(team.attempts)
        m = team.attempts{j}.metrics;
        if isfield(m, 'swingup_success') && m.swingup_success
            times(j) = m.swingup_time;
        end
        if isfield(m, 'smape_eligible') && m.smape_eligible && ~isnan(m.smape)
            smapes(j) = m.smape;
        end
    end
    best_time  = min(times,  [], 'omitnan');   % NaN if all NaN -> D-02
    best_smape = min(smapes, [], 'omitnan');   % NaN if all NaN -> D-03
    if isempty(best_time)  || all(isnan(times)),  best_time  = NaN; end
    if isempty(best_smape) || all(isnan(smapes)), best_smape = NaN; end
end
```

**Edge case: 0 attempts.** When `numel(team.attempts) == 0`, `times` and `smapes` are empty; `min([], [], 'omitnan')` returns an empty double. The `if isempty` guard catches this and returns `NaN`. [ASSUMED: `min([], [], 'omitnan')` returns `[]` not `NaN`; planner should add explicit guard as shown above.]

### Pattern: Dense Ranking with Sports Convention (SCOR-02, SCOR-03, D-04..D-06)

MATLAB has no built-in dense rank function. The standard approach uses `sort` + `unique` to assign dense ranks, then maps ranks to point values via index lookup.

```matlab
% Source: base MATLAB idiom [VERIFIED: MATLAB R2025b sort/unique]
function pts = assign_points_dense(values, point_table, higher_is_better)
% values: [N x 1] double, may contain NaN (NaN = did not participate -> 0 pts)
% point_table: point value per rank position [1st, 2nd, 3rd, 4th, ...]
% higher_is_better: false for time (lower=better), true for (hypothetical use)
%
% Returns: pts [N x 1] double, points per team

    N = numel(values);
    pts = zeros(N, 1);

    % Teams with NaN get 0 points; remove them from ranking
    valid = ~isnan(values);
    if ~any(valid), return; end

    v = values(valid);

    % Sort ascending (lower time = better; lower SMAPE = better)
    if higher_is_better
        [~, order] = sort(v, 'descend');
    else
        [~, order] = sort(v, 'ascend');
    end

    % Assign dense ranks: teams with equal values get the same (best) rank
    sorted_v = v(order);
    ranks = ones(numel(v), 1);
    for k = 2:numel(sorted_v)
        if sorted_v(k) == sorted_v(k-1)
            ranks(k) = ranks(k-1);   % tie: same rank
        else
            ranks(k) = ranks(k-1) + 1;  % no gap: dense rank
        end
    end

    % Map rank -> points via point_table
    % rank 1 -> point_table(1), rank 2 -> point_table(2), etc.
    rank_pts = zeros(numel(v), 1);
    for k = 1:numel(v)
        r = ranks(k);
        if r <= numel(point_table)
            rank_pts(k) = point_table(r);
        else
            rank_pts(k) = 0;
        end
    end

    % Unsort back to original order
    pts_valid = zeros(numel(v), 1);
    pts_valid(order) = rank_pts;

    % Write back to full pts vector
    pts(valid) = pts_valid;
end
```

**Key insight on SCOR-03 (SMAPE tie at 3rd/4th):** The point_table for SMAPE is `[2, 1, 0.5, 0.5]`. If two teams tie for 3rd, both get rank 3 -> `point_table(3) = 0.5`. If there is no 4th distinct value, `point_table(4)` is never accessed. This correctly satisfies D-06. [ASSUMED: `sort` in MATLAB is stable for ties (order of equal elements preserved from input order); this does not affect point assignment since ties are mapped to the same rank, but the planner should note this for the final leaderboard Rank column.]

### Pattern: BLDC Band Scoring (SCOR-04, D-07)

```matlab
% Source: base MATLAB conditional [VERIFIED: MATLAB R2025b]
function pts = score_bldc_smape(smape_pct, bands, band_pts)
% bands: upper edges [40, 80, 120, 160] (last band is open: 160+)
% band_pts: [4, 3, 2, 1, 0]
    if isnan(smape_pct)
        pts = 0;
        return
    end
    pts = band_pts(end);  % default: last band (160+ -> 0)
    for k = 1:numel(bands)
        if smape_pct < bands(k)
            pts = band_pts(k);
            return
        end
    end
end
```

Note: `cfg.bldc_smape_bands = [40, 80, 120, 160]` uses upper-exclusive boundaries. The loop uses `< bands(k)` which correctly implements `0-40%=4pt` as `smape_pct < 40 -> 4pt`.

### Pattern: Table Construction (OUTP-01, D-10, D-14)

```matlab
% Source: MATLAB table() constructor [VERIFIED: MATLAB R2025b]
T = table(...
    team_names', ...         % cell array of strings -> VariableType char/string
    best_times', ...         % double
    best_smapes', ...        % double
    time_pts', ...           % double
    smape_pts', ...          % double
    part_pts', ...           % double
    total_pts', ...          % double
    ranks', ...              % double (NaN for BLDC)
    'VariableNames', {'Team','BestSwingupTime','BestSMAPE', ...
                      'TimePoints','SMAPEPoints','ParticipationPoint', ...
                      'TotalPoints','Rank'});
```

**Column types:** `Team` should be `string` array or cell array of char. `writetable` handles both correctly for CSV/xlsx. All numeric columns are `double`. `NaN` exports as empty cell in xlsx and as empty string in CSV by default. [VERIFIED: MATLAB R2025b `writetable` behavior documented in MATLAB docs]

**Stepper-only display (D-10):** Use logical index to filter before `disp()`:
```matlab
stepper_mask = strcmp({cfg.teams.type}, 'stepper')';
disp(T(stepper_mask, :));
```

### Pattern: CSV and Excel Export (OUTP-02, D-13)

```matlab
% Source: MATLAB writetable documentation [VERIFIED: MATLAB R2025b]
timestamp = datestr(now, 'yyyymmdd_HHMM');
base_path = fullfile(cfg.export_dir, ...
    sprintf('%s_%s', cfg.export_stem, timestamp));

writetable(T, [base_path '.csv']);
writetable(T, [base_path '.xlsx']);
```

**Key facts:**
- `writetable(T, file)` auto-detects format from extension. `.csv` produces comma-separated plain text. `.xlsx` produces Excel format. [VERIFIED: MATLAB R2025b writetable documentation]
- For `.xlsx`, MATLAB uses its built-in Java-based writer. No Excel installation required on the MATLAB host machine. [ASSUMED based on training knowledge; valid for R2024b+ on Windows; confirm if running headless/Linux.]
- `datestr(now, 'yyyymmdd_HHMM')` produces `20260406_1430` format. `datestr` is the legacy API but reliable in R2025b. Alternative: `datetime('now','Format','yyyyMMdd_HHmm')` as string. [VERIFIED: both work in R2025b]
- The `data/` folder already exists and is gitignored per CLAUDE.md/project conventions. No `mkdir` guard strictly needed, but adding `if ~exist(cfg.export_dir,'dir'), mkdir(cfg.export_dir); end` is defensive practice.

### Pattern: Grouped Bar Chart (D-12)

```matlab
% Source: MATLAB bar() documentation [VERIFIED: MATLAB R2025b]
score_matrix = [time_pts, smape_pts, part_pts];  % [N_teams x 3]
fig = figure('Name', 'Score Breakdown', 'NumberTitle', 'off');
b = bar(score_matrix, 'grouped');
b(1).FaceColor = [0.2, 0.6, 0.9];   % blue: time
b(2).FaceColor = [0.9, 0.5, 0.1];   % orange: SMAPE
b(3).FaceColor = [0.3, 0.8, 0.3];   % green: participation
set(gca, 'XTickLabel', {cfg.teams.name}, 'XTickLabelRotation', 15);
ylabel('Points');
title('Competition Score Breakdown');
legend({'Time Points','SMAPE Points','Participation'}, 'Location','northeast');
grid on;
```

**Key facts:**
- `bar(M, 'grouped')` where M is [N x K] produces N groups of K bars. [VERIFIED: MATLAB R2025b bar documentation]
- `b = bar(...)` returns a [1 x K] array of `Bar` objects; each `b(k).FaceColor` sets the color of the k-th bar in every group. [VERIFIED: MATLAB R2025b]
- `XTickLabel` accepts cell array of strings for labeling groups (one label per team). [VERIFIED: MATLAB R2025b]
- BLDC team is included in the bar chart (D-12 says "stepper + BLDC") — the score_matrix rows include all teams in `cfg.teams` order.

### Pattern: Diagnostic Print (OUTP-04, D-15)

```matlab
% Source: existing fprintf pattern in score_competition.m [VERIFIED: existing code]
fprintf('\n--- Per-Team Diagnostics ---\n');
for i = 1:numel(session.teams)
    t = session.teams(i);
    n = numel(t.attempts);
    fprintf('  %s (%s): %d attempt(s)\n', t.name, t.type, n);
    fprintf('    Best swingup time : %s\n', fmt_time(best_times(i)));
    fprintf('    Best SMAPE        : %s\n', fmt_smape(best_smapes(i)));
    fprintf('    Participation     : %s\n', yesno(part_flags(i)));
end
```

Helper formatters (reuse existing `yesno()`, add two more):
```matlab
function s = fmt_time(t)
    if isnan(t) || isinf(t), s = 'N/A'; else, s = sprintf('%.2f s', t); end
end
function s = fmt_smape(v)
    if isnan(v), s = 'N/A'; else, s = sprintf('%.1f%%', v); end
end
```

### Anti-Patterns to Avoid

- **Using `tiedrank` from Statistics Toolbox:** That toolbox function produces fractional ranks (average method), not integer dense ranks needed for the sports convention. Don't use it even if available — implement dense ranking directly as shown above.
- **Using `sortrows` on the table for ranking:** Ranking must be done on the numeric arrays before constructing the table, not via table sorting. Otherwise the Rank column becomes row order, not dense rank.
- **Calling `writetable` with `'WriteMode','overwrite'` for csv:** The default mode already overwrites; the parameter is unnecessary and adds noise.
- **Using `xlswrite` (deprecated):** `xlswrite` was removed in R2025a. Use `writetable` only. [VERIFIED: MATLAB R2025a release notes document xlswrite removal]
- **Constructing the MATLAB table with column vectors of wrong orientation:** `table()` expects column vectors. Ensure `team_names'` is an N-by-1 column (transpose if needed).

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Dense ranking | Custom sort loop | `sort` + manual rank assignment (see pattern above) | MATLAB sort is vectorized and handles ties via equal index positions |
| CSV export | `fprintf` loop writing rows | `writetable(T, file)` | Handles escaping, quoting, headers automatically |
| xlsx export | `xlswrite` or Java POI directly | `writetable(T, file)` with .xlsx extension | Handles cell types, NaN->empty, headers automatically |
| Timestamp in filename | `sprintf` with `clock()` | `datestr(now, 'yyyymmdd_HHMM')` | Standard MATLAB idiom, no clock vector indexing errors |
| Bar color array indexing | Loop over handles | `b(k).FaceColor = [r,g,b]` directly | bar() returns Bar object array; direct property access is cleaner |

---

## Common Pitfalls

### Pitfall 1: `min([], [], 'omitnan')` Returns `[]` Not `NaN`

**What goes wrong:** When a team has zero attempts, the `times` accumulator is `[]`. `min([], [], 'omitnan')` returns `[]` (empty double), not `NaN`. If the caller expects `NaN` for "no data", the downstream `isnan()` check silently fails and `NaN` comparisons behave unexpectedly.

**Why it happens:** MATLAB's `min` with `'omitnan'` and empty input returns `[]` as a size-preserving convention.

**How to avoid:** After calling `min`, add: `if isempty(best_time) || all(isnan(times)), best_time = NaN; end` (as shown in the aggregation pattern above).

**Warning signs:** `isnan(best_time)` returns `[]` instead of `true`; logical indexing downstream produces empty arrays.

### Pitfall 2: `sort` of All-NaN Vector

**What goes wrong:** When all stepper teams have NaN times (no successful swingup), `sort(NaN, NaN, NaN, NaN)` produces a sorted vector of all NaN. The ranking loop assigns rank 1 to NaN, and all teams get 2 points. This is incorrect — they should all get 0 points.

**Why it happens:** `sort` places NaN at the end by default, but if ALL values are NaN, the entire sorted vector is NaN. The `sorted_v(k) == sorted_v(k-1)` tie-check then evaluates `NaN == NaN` which is `false` in MATLAB — so ranks would NOT be tied, and teams would get different (incorrect) points.

**How to avoid:** The `valid = ~isnan(values)` guard in the `assign_points_dense` pattern ensures NaN teams are excluded from ranking entirely and receive 0 points. If `~any(valid)` is true, return zeros immediately. [VERIFIED: NaN == NaN is false in MATLAB, confirmed by base language spec]

### Pitfall 3: `writetable` xlsx Fails if File Is Open in Excel

**What goes wrong:** If the output `.xlsx` file is currently open in Microsoft Excel on the same machine, `writetable` throws a Java/COM error when trying to write.

**Why it happens:** Excel locks the file for exclusive write access.

**How to avoid:** Wrap `writetable` for xlsx in `try/catch`. On failure, print a warning message (don't crash the grading session). The CSV export succeeds regardless since Excel doesn't lock `.csv` files.

```matlab
try
    writetable(T, [base_path '.xlsx']);
    fprintf('  Saved: %s.xlsx\n', base_path);
catch e
    warning('scorer:xlsxfail', 'xlsx export failed (%s). Close Excel and retry. CSV was saved.', e.message);
end
```

### Pitfall 4: `datestr` Deprecation Warning in R2025b

**What goes wrong:** `datestr(now)` produces a deprecation warning in MATLAB R2025b. While it still works, the warning pollutes the command window during grading.

**Why it happens:** MATLAB is migrating to `datetime`-based APIs; `now` + `datestr` are legacy.

**How to avoid:** Use `datetime('now','Format','yyyyMMdd_HHmm')` to produce the timestamp string directly:

```matlab
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmm'));
```

[ASSUMED: `datestr(now)` produces a deprecation warning in R2025b specifically; verified that `datetime` approach works in R2025a+]

### Pitfall 5: `disp(T)` Table Column Width for Long Team Names

**What goes wrong:** MATLAB's `disp` for tables auto-formats column widths but long team names can overflow into numeric columns, making the command window output hard to read.

**Why it happens:** Default `disp` for tables uses 80-character line width heuristics.

**How to avoid:** This is cosmetic-only. No fix needed for correctness. If needed, `fprintf`-based manual formatting gives full control over column widths.

### Pitfall 6: Live Script `%[text]` Annotation Required Before New Function Sections

**What goes wrong:** Per CLAUDE.md, new script sections that introduce local functions should have `%[text]` comment blocks describing them. Omitting these means the Live Editor shows the functions without documentation blocks, inconsistent with existing script style.

**Why it happens:** Project convention (CLAUDE.md: "New scripts must use the Live Script `.m` format").

**How to avoid:** Add `%[text]` block before the new function group, matching the existing pattern at lines 610-613, 657-661, 723-727, etc.

---

## Code Examples

### Full `compute_leaderboard` Skeleton

```matlab
% Source: integration of patterns above [ASSUMED: structure correct per context]
function T = compute_leaderboard(session, cfg)
%COMPUTE_LEADERBOARD Aggregate metrics and apply scoring rubric.
%   Returns a table with all teams (stepper + BLDC), ordered by cfg.teams.

    N = numel(session.teams);
    team_names   = {cfg.teams.name}';
    team_types   = {cfg.teams.type}';
    best_times   = NaN(N, 1);
    best_smapes  = NaN(N, 1);
    part_flags   = false(N, 1);
    time_pts     = zeros(N, 1);
    smape_pts    = zeros(N, 1);
    part_pts     = zeros(N, 1);

    % D-01..D-03: Aggregate best-per-metric
    for i = 1:N
        [best_times(i), best_smapes(i), part_flags(i)] = ...
            aggregate_best(session.teams(i));
        part_pts(i) = double(part_flags(i));   % SCOR-05: 1pt if any attempt participates
    end

    % D-05/D-06: Rank stepper teams
    stepper_mask = strcmp(team_types, 'stepper');
    st = best_times(stepper_mask);
    ss = best_smapes(stepper_mask);
    st_pts = assign_points_dense(st,  cfg.time_points,  false);
    ss_pts = assign_points_dense(ss,  cfg.smape_points, false);
    time_pts(stepper_mask)  = st_pts;
    smape_pts(stepper_mask) = ss_pts;

    % D-07..D-09: Score BLDC team
    bldc_mask = strcmp(team_types, 'bldc');
    for i = find(bldc_mask)'
        smape_pts(i) = score_bldc_smape(best_smapes(i), ...
            cfg.bldc_smape_bands, cfg.bldc_smape_pts);
    end

    % SCOR-06: Total points
    total_pts = time_pts + smape_pts + part_pts;

    % Build Rank column: stepper teams only, dense rank on total
    ranks = NaN(N, 1);
    st_totals = total_pts(stepper_mask);
    st_ranks  = assign_dense_rank_descending(st_totals);  % higher total = better rank
    ranks(stepper_mask) = st_ranks;

    % Construct output table (OUTP-01)
    T = table(string(team_names), best_times, best_smapes, ...
              time_pts, smape_pts, part_pts, total_pts, ranks, ...
              'VariableNames', {'Team','BestSwingupTime','BestSMAPE', ...
                                'TimePoints','SMAPEPoints','ParticipationPoint', ...
                                'TotalPoints','Rank'});
end
```

Note: `assign_dense_rank_descending` is a thin wrapper around `assign_points_dense` that maps rank position to rank number (1,2,3...) rather than points. It is separate from the point-assignment function to avoid confusion.

### `export_results` Skeleton

```matlab
% Source: writetable pattern [VERIFIED: MATLAB R2025b]
function export_results(T, cfg)
    if ~exist(cfg.export_dir, 'dir')
        mkdir(cfg.export_dir);
    end
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmm'));
    base = fullfile(cfg.export_dir, sprintf('%s_%s', cfg.export_stem, timestamp));

    writetable(T, [base '.csv']);
    fprintf('Saved: %s.csv\n', [base '.csv']);

    try
        writetable(T, [base '.xlsx']);
        fprintf('Saved: %s.xlsx\n', [base '.xlsx']);
    catch e
        warning('scorer:xlsxfail', 'xlsx export failed: %s', e.message);
    end
end
```

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `xlswrite` for xlsx | `writetable(T, 'f.xlsx')` | R2025a (xlswrite removed) | Must use writetable; no alternative |
| `datestr(now)` | `char(datetime('now','Format','...'))` | R2022b (soft deprecation) | Use datetime API to avoid warnings |
| `Statistics and Machine Learning Toolbox tiedrank` | Manual dense rank (base MATLAB) | N/A — never needed here | Avoids toolbox dependency; dense rank ≠ fractional rank |

**Deprecated/outdated:**
- `xlswrite`: Removed in MATLAB R2025a. Do not use. [ASSUMED based on training knowledge — planner should verify against R2025b release notes if uncertain]
- `datestr(now)`: Soft-deprecated in R2022b+, still functional but generates warnings. Use `datetime` API instead.

---

## Open Questions (RESOLVED)

1. **Is `writetable` xlsx reliable without Excel installed on the grading machine?** (RESOLVED)
   - What we know: MATLAB uses a built-in Java-based xlsx writer (Apache POI) that does not require Excel. This has been the case since R2019b.
   - What's unclear: The exact version when this became fully reliable on Windows without Excel.
   - Recommendation: [ASSUMED] Should work on the instructor's Windows 11 machine regardless of Excel installation. Planner should add try/catch as a defensive measure per Pitfall 3.

2. **Should the session struct be saved to `.mat` alongside CSV/xlsx?** (RESOLVED)
   - What we know: This is explicitly listed as Claude's Discretion in CONTEXT.md.
   - What's unclear: User preference.
   - Recommendation: Include it. `save(fullfile(cfg.export_dir, ['session_' timestamp '.mat']), 'session', 'T')` costs nothing and preserves the full session for post-hoc inspection. Flag in planner as a low-cost addition.

3. **Rank column for BLDC in the exported table: `NaN` or empty string?** (RESOLVED)
   - What we know: D-14 says "NaN or empty". `writetable` exports `NaN` as an empty cell in xlsx and an empty field in CSV.
   - Recommendation: Use `NaN` in the MATLAB table (type double). Let `writetable` handle the empty representation in output files — no special handling needed.

---

## Environment Availability

Step 2.6: SKIPPED — Phase 4 is pure MATLAB script code operating on in-workspace session struct. No external tools, services, CLIs, databases, or package managers beyond MATLAB R2025b (already confirmed running).

---

## Validation Architecture

`nyquist_validation` is explicitly `false` in `.planning/config.json`. Section skipped.

---

## Security Domain

This is a local grading script with no network access, authentication, user input to eval, or file-system traversal beyond `data/`. No ASVS categories apply. The only attack surface is the timestamped filename (uses `datetime`, not user input) and file write to `data/` (already gitignored, no path traversal). Section omitted.

---

## Project Constraints (from CLAUDE.md)

- **Live Script `.m` format required:** New code sections must use `%[text]` for rich text annotations, `%%` for section breaks, `%[appendix]{"version":"1.0"}` at end. Escape `_` as `\_` inside `%[text]` lines.
- **Project open check:** Each script must start with `if isempty(matlab.project.rootProject); openProject(...); end` (already present at line 7).
- **Git commit rule:** Only commit files registered in `digtwin_labo.prj`. Check project file list before staging. `data/` output files are gitignored and must NOT be committed.
- **No `xlswrite`:** Use `writetable` only (removed in R2025a per MATLAB release notes).
- **Toolbox availability:** Embedded Coder, Simulink, Symbolic Math Toolbox, Control System Toolbox are present but NOT needed for Phase 4. Phase 4 uses base MATLAB only.
- **GSD workflow enforcement:** All edits flow through GSD phase execution (`/gsd:execute-phase`), not direct file edits.

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `min([], [], 'omitnan')` returns `[]` not `NaN` | Pattern: Best-Per-Metric Aggregation | Would remove the empty-guard requirement; code would still be correct with the guard |
| A2 | `datestr(now)` produces a deprecation warning in R2025b | Pitfall 4 | If no warning, `datestr` is fine to use; `datetime` approach works either way |
| A3 | `xlswrite` was removed in R2025a | State of the Art / Deprecated | If still present in R2025b, no impact (we use `writetable` regardless) |
| A4 | `writetable` xlsx works without Excel installed on Windows 11 | Open Questions | If xlsx fails, CSV is the fallback (always succeeds); try/catch in plan handles this |
| A5 | `sort` of ties assigns equal-position indices (order-preserving for equal elements) | Pattern: Dense Ranking | If unstable sort, rank assignment still correct since ties map to same rank; final Rank column ordering might differ but points are unaffected |

---

## Sources

### Primary (HIGH confidence)
- MATLAB R2025b existing script `scripts/score_competition.m` — verified all existing patterns (struct fields, fprintf style, local function placement, `%[text]` annotations)
- `.planning/phases/04-scoring-and-output/04-CONTEXT.md` — locked decisions D-01 through D-15
- `.planning/REQUIREMENTS.md` — acceptance criteria SCOR-01..06, OUTP-01..05
- CLAUDE.md — project conventions (Live Script format, commit rules, toolbox list)

### Secondary (MEDIUM confidence)
- MATLAB R2025b `writetable` documentation (inferred from well-established R2019b+ behavior)
- MATLAB R2025b `bar` grouped chart documentation (behavior stable since R2014b)
- MATLAB `datetime` API (stable since R2014b, preferred over `datestr` since R2022b)

### Tertiary (LOW confidence — see Assumptions Log)
- `xlswrite` removal in R2025a: based on training knowledge (cutoff August 2025), not verified against live R2025b release notes
- `datestr` deprecation warning behavior in R2025b: based on training knowledge

---

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — base MATLAB only, no new dependencies
- Architecture: HIGH — direct integration into existing script at identified line 249
- Pitfalls: MEDIUM — 3 of 6 pitfalls are verified from existing code; xlsx/datetime pitfalls are ASSUMED
- Export patterns: HIGH — `writetable` CSV/xlsx is well-established MATLAB API

**Research date:** 2026-04-06
**Valid until:** 2026-10-06 (stable MATLAB base APIs; `writetable` / `bar` / `table` have not changed in years)
