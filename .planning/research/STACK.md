# Technology Stack

**Project:** Swingup Competition Scoring Script
**Researched:** 2026-04-02
**MATLAB Version:** R2025b (project constraint)

---

## Recommended Stack

### Core SDI API — Loading and Signal Extraction

| Function / Property | Purpose | Confidence |
|--------------------|---------|------------|
| `Simulink.sdi.clear('Export', false)` | Wipe SDI repository before each load to prevent run ID collisions between team files | HIGH |
| `Simulink.sdi.load(filepath)` | Load `.mldatx` session file into SDI repository | HIGH |
| `Simulink.sdi.getAllRunIDs()` | Returns numeric array of all run IDs after load, most-recent last | HIGH |
| `Simulink.sdi.getRun(runID)` | Returns `Simulink.sdi.Run` object for a given ID | HIGH |
| `run.Name` | String property — default follows SDI naming rule (e.g. `"Run 1: ModelName"`) | HIGH |
| `run.Description` | String property — empty unless team set it; useful secondary discriminator | HIGH |
| `run.SignalCount` | Integer — sanity check that signal count matches expectations | HIGH |
| `getSignalByIndex(runObj, idx)` | Returns `Simulink.sdi.Signal` object by 1-based index | HIGH |
| `sig.Name` | Signal name string — used for manual matching with smart sort | HIGH |
| `sig.Values` | Returns `timeseries` object (`.time` = double seconds, `.data` = double array) | HIGH |

**Why this path:** `Simulink.sdi.load` + `getAllRunIDs` is the documented programmatic workflow for `.mldatx` files. The SDI is the only supported mechanism for reading `.mldatx` — there is no file-level parser alternative.

**Confidence source:** MathWorks official documentation pages for `Simulink.sdi.load`, `Simulink.sdi.Run`, `Simulink.sdi.Signal`.

---

### Separating Archive Run from Simulation Run

The `.mldatx` file contains both a hardware run (manually archived in the SDI before saving) and a simulation run (most recent run). The SDI API does **not** expose an `isArchived` boolean property on `Simulink.sdi.Run` objects — archive status is a display concept only.

**Recommended discrimination strategy (in priority order):**

1. **Name pattern matching** — hardware runs typically carry names like `"Run N: digtwin_labo_lib / External"` or contain `"External"` / `"PIL"` / `"hardware"`. Simulation runs contain `"Normal"` or the model name without hardware suffix.
2. **Run count expectation** — after `Simulink.sdi.load` on a well-formed team file, exactly 2 runs are expected. If count != 2, raise a warning and fall back to manual selection.
3. **`Simulink.sdi.Run.getLatest()`** — returns the most-recently created run, which is always the simulation run (SDI archives the prior run when a new sim finishes). Use this to identify simulation run; the other run is hardware.
4. **Signal count heuristic** — hardware run may have more signals (raw sensor channels) than the simulation run; check `run.SignalCount`.
5. **Fallback** — present both run names to the scorer and request manual assignment.

**Confidence:** MEDIUM. The `getLatest` / "other-is-archive" pattern follows from documented SDI auto-archive behavior, but archive status is not a first-class property. The name-matching approach depends on team run naming conventions, which can vary.

---

### Time-Series Interpolation and Signal Alignment

| Function | Purpose | When to Use | Confidence |
|----------|---------|-------------|------------|
| `timetable` | Modern container for time-stamped data; supports `retime`/`synchronize` | Convert `timeseries` outputs to timetable for resampling | HIGH |
| `retime(tt, newTimeVec, 'linear')` | Resample timetable to arbitrary time vector using linear interpolation | Align simulation signal to hardware time grid | HIGH |
| `synchronize(tt1, tt2, 'union', 'linear')` | Merge two timetables onto union time grid with linear fill | When a common time axis is needed | HIGH |
| `interp1(t_old, y_old, t_new, 'linear')` | Direct numeric interpolation without timetable overhead | Fastest path when signal is already a `double` array | HIGH |
| `timeseries.resample(newTime)` | Resample timeseries object in-place | Avoid — legacy API, less composable than timetable | MEDIUM |

**Recommended workflow:**

```matlab
% Extract numeric arrays from SDI Signal
ts_hw  = sig_hw.Values;          % timeseries
ts_sim = sig_sim.Values;         % timeseries
t_hw   = ts_hw.Time;             % double column vector, seconds
y_hw   = ts_hw.Data;
t_sim  = ts_sim.Time;
y_sim  = ts_sim.Data;

% Interpolate simulation onto hardware time grid
% (hardware at 2 kHz is the reference grid)
y_sim_aligned = interp1(t_sim, y_sim, t_hw, 'linear');
```

**Why `interp1` over `retime`:** For this use case — fixed-rate hardware at 2 kHz as reference, simulation at possibly different rate — `interp1` is the most direct and transparent. The timetable overhead adds complexity without benefit when the output is just two aligned `double` vectors for SMAPE computation. Use `retime`/`synchronize` only if building a timetable-based pipeline from the start.

**Why NOT `resample` (Signal Processing Toolbox):** `resample` applies an anti-aliasing FIR filter appropriate for downsampling audio/comms signals. For pendulum kinematics (smooth, low-frequency), this filter introduces unnecessary phase delay and endpoint artifacts. Linear `interp1` is correct here.

---

### SMAPE Computation

No dedicated MATLAB toolbox function exists for SMAPE. Implement directly from definition.

```matlab
% Symmetric Mean Absolute Percentage Error
% Applied over a windowed segment [t_start, t_end]
function smape = compute_smape(y_hw, y_sim)
    % y_hw and y_sim must be on the same time grid (same length)
    % Handles near-zero denominator with epsilon guard
    epsilon = 1e-6;
    denom   = (abs(y_hw) + abs(y_sim)) / 2 + epsilon;
    smape   = 100 * mean(abs(y_hw - y_sim) ./ denom);
end
```

**Windowing options (all configurable):**

| Mode | Index extraction |
|------|-----------------|
| Fixed time window (e.g. 5 s after start) | `idx = t_hw >= t_start & t_hw <= t_start + window_s` |
| Until angle threshold (|q2| > pi/2) | `idx = t_hw >= t_start & t_hw <= t_cross_90` |
| Until swingup completion (|q2| crosses pi) | `idx = t_hw >= t_start & t_hw <= t_swingup` |

**Confidence:** HIGH — SMAPE formula is mathematical, not library-dependent.

---

### MATLAB Table Operations for Leaderboard

| Function | Purpose | Confidence |
|----------|---------|------------|
| `table(...)` | Construct leaderboard table from column variables | HIGH |
| `T.Properties.VariableNames` | Set human-readable column headers | HIGH |
| `sortrows(T, 'SwingupTime_s')` | Sort by metric for ranking | HIGH |
| `T.Rank = (1:height(T))'` | Add rank column after sort | HIGH |
| `T(idx, :)` | Filter rows (e.g. stepper teams only) | HIGH |
| `T.Points = zeros(height(T), 1)` | Preallocate scoring column | HIGH |

**Recommended table schema:**

```matlab
T = table( ...
    teamNames,        ...  % string array
    attemptNums,      ...  % double
    successFlags,     ...  % logical
    participFlags,    ...  % logical (|q2| > pi/2)
    swingupTimes_s,   ...  % double (NaN if unsuccessful)
    smapeVals_pct,    ...  % double (NaN if not scoreable)
    timePoints,       ...  % double
    smapePoints,      ...  % double
    participPoints,   ...  % double
    totalPoints       ...  % double
);
T.Properties.VariableNames = { ...
    'Team', 'Attempt', 'SwingupOK', 'Participated', ...
    'SwingupTime_s', 'SMAPE_pct', ...
    'Pts_Time', 'Pts_SMAPE', 'Pts_Partic', 'Pts_Total' ...
};
```

**Why table over struct array:** Tables support `sortrows`, display cleanly in the Live Editor output pane, and feed directly into `writetable` for export without conversion.

---

### CSV / Excel Export

| Function | Purpose | Confidence |
|----------|---------|------------|
| `writetable(T, 'scores.csv')` | CSV export — no Excel dependency, cross-platform | HIGH |
| `writetable(T, 'scores.xlsx', 'Sheet', 'Leaderboard')` | Excel export with named sheet | HIGH |
| `writetable(T, file, 'WriteVariableNames', true)` | Default — always keep headers on | HIGH |

**Recommended pattern:**

```matlab
out_csv  = fullfile(out_dir, 'leaderboard.csv');
out_xlsx = fullfile(out_dir, 'leaderboard.xlsx');
writetable(T_leaderboard, out_csv);
writetable(T_leaderboard, out_xlsx, 'Sheet', 'Leaderboard');
```

**Why `writetable` over `xlswrite`:** `xlswrite` is deprecated since R2019a. `writetable` is cross-platform (does not require a Windows Excel COM instance when writing `.xlsx` with `UseExcel=false`, which is the default on non-Windows or when Excel is absent).

**Confidence:** HIGH — `writetable` is the current standard with no viable alternative.

---

### Plotting — Signal Comparison

| Function | Purpose | Confidence |
|----------|---------|------------|
| `tiledlayout(m, n, 'TileSpacing', 'compact', 'Padding', 'tight')` | Multi-panel figure layout; preferred over `subplot` since R2019b | HIGH |
| `nexttile` | Advance to next panel in tiled layout | HIGH |
| `plot(t, y)` | Line plot for time series | HIGH |
| `xline(t_event)` | Vertical event markers (start time, swingup time) | HIGH |
| `yline(pi, '--')` | Horizontal threshold lines (q2 = ±pi) | HIGH |
| `legend('HW', 'Sim')` | Signal identification | HIGH |
| `title(sprintf('Team %s — Attempt %d', name, k))` | Per-panel title with team/attempt info | HIGH |
| `xlabel('Time (s)')` / `ylabel('q_2 (rad)')` | Axis labels | HIGH |
| `exportgraphics(fig, 'plot.png', 'Resolution', 150)` | Save figure to file without display dependency | HIGH |

**Recommended figure layout per attempt:**

```
tiledlayout(2,1)
  Tile 1: q2 hardware vs simulation (with ±pi lines, xline at start and swingup)
  Tile 2: accel/torque command (hardware only, for start detection verification)
```

**Why `tiledlayout` over `subplot`:** `tiledlayout` introduced in R2019b handles tight spacing, shared axis labels, and per-tile title placement without the positioning hacks `subplot` requires. It reflowsautomatically when figure is resized. Standard for new MATLAB code from R2020a onwards.

**Why `exportgraphics` over `saveas` or `print`:** `exportgraphics` produces publication-quality output, respects figure background, and handles resolution correctly. `saveas` is legacy and `print` has verbose syntax.

---

## Alternatives Considered

| Category | Recommended | Alternative | Why Not |
|----------|-------------|-------------|---------|
| Signal container | `timeseries.Values` -> `double` arrays | `timetable` pipeline end-to-end | Timetable adds overhead for what is fundamentally numeric array math; `interp1` + `double` is faster and clearer |
| Resampling | `interp1(..., 'linear')` | `resample` (Signal Processing TB) | `resample` applies anti-aliasing FIR filter — inappropriate for smooth low-frequency kinematic signals; introduces endpoint artifacts |
| Resampling | `interp1` | `timeseries.resample` | Legacy API; less composable; no advantage over direct numeric interpolation |
| Figure layout | `tiledlayout` | `subplot` | `subplot` has fragile positioning; `tiledlayout` is the current standard |
| Excel export | `writetable` | `xlswrite` | `xlswrite` is deprecated since R2019a |
| Run ID access | `getAllRunIDs` + loop | `getRunIDByIndex` in loop | `getAllRunIDs` returns the full array at once, eliminating repeated function calls |
| Signal name filtering | `contains(sig.Name, candidates, 'IgnoreCase', true)` | Exact string match | Case-insensitive substring match is robust to teams using `Q2`, `q2`, `Theta2`, etc. |

---

## Session Isolation Pattern (Critical)

**Problem:** `Simulink.sdi.load` *appends* runs to the existing SDI repository. If a prior team's file is still loaded, run IDs from different teams coexist and `getAllRunIDs` returns all of them.

**Solution:** Always clear before loading:

```matlab
Simulink.sdi.clear('Export', false);   % wipe without workspace export (faster)
Simulink.sdi.load(team_file_path);
runIDs = Simulink.sdi.getAllRunIDs();   % now guaranteed to be only this team's runs
```

**Confidence:** HIGH — documented behavior of `Simulink.sdi.load` (appends, not replaces).

---

## Complete SDI Workflow Summary

```matlab
% 1. Isolate session
Simulink.sdi.clear('Export', false);
Simulink.sdi.load(mldatx_path);

% 2. Get run objects
runIDs = Simulink.sdi.getAllRunIDs();
assert(numel(runIDs) == 2, 'Expected exactly 2 runs (HW + SIM)');

% 3. Identify simulation run (most recently created = lowest creation order = first ID)
%    The archived (hardware) run was created first; getLatest returns the sim run.
simRun = Simulink.sdi.Run.getLatest();
hwRunID = setdiff(runIDs, simRun.ID);
hwRun   = Simulink.sdi.getRun(hwRunID);

% 4. List signal names (for manual mapping UI)
for k = 1:hwRun.SignalCount
    sig = getSignalByIndex(hwRun, k);
    fprintf('  [%2d] %s\n', k, sig.Name);
end

% 5. Retrieve specific signals (after user selects indices)
sig_q2_hw  = getSignalByIndex(hwRun,  idx_q2_hw);
sig_q2_sim = getSignalByIndex(simRun, idx_q2_sim);

% 6. Extract numeric arrays
t_hw  = sig_q2_hw.Values.Time;
y_hw  = sig_q2_hw.Values.Data;
t_sim = sig_q2_sim.Values.Time;
y_sim = sig_q2_sim.Values.Data;

% 7. Align simulation to hardware time grid
y_sim_aligned = interp1(t_sim, y_sim, t_hw, 'linear', NaN);
```

---

## Installation / Dependencies

All APIs are part of Simulink (base) and core MATLAB. No additional toolboxes required for the scoring script.

| API | Toolbox | Notes |
|-----|---------|-------|
| `Simulink.sdi.*` | Simulink | Included in project toolbox set |
| `timetable`, `retime`, `synchronize` | MATLAB (core) | Available since R2016b |
| `interp1` | MATLAB (core) | Always available |
| `table`, `sortrows`, `writetable` | MATLAB (core) | Always available |
| `tiledlayout`, `nexttile` | MATLAB (core) | Available since R2019b |
| `exportgraphics` | MATLAB (core) | Available since R2020a |

---

## Sources

- [Simulink.sdi.load — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.load.html)
- [Simulink.sdi.Run — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.run.html)
- [Simulink.sdi.Signal — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.signal.html)
- [Inspect and Compare Data Programmatically — MathWorks](https://www.mathworks.com/help/simulink/ug/record-and-inspect-signal-data-programmatically.html)
- [Simulink.sdi.getAllRunIDs — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.getallrunids.html)
- [Simulink.sdi.Run.getLatest — MathWorks](https://www.mathworks.com/help/simulink/slref/simulink.sdi.run.getlatest.html)
- [retime — MathWorks](https://www.mathworks.com/help/matlab/ref/timetable.retime.html)
- [synchronize — MathWorks](https://www.mathworks.com/help/matlab/ref/timetable.synchronize.html)
- [writetable — MathWorks](https://www.mathworks.com/help/matlab/ref/writetable.html)
- [tiledlayout — MathWorks](https://www.mathworks.com/help/matlab/ref/tiledlayout.html)
- [Save and Share SDI Data and Views — MathWorks](https://www.mathworks.com/help/simulink/ug/save-and-share-simulation-data-inspector-data-and-views.html)
