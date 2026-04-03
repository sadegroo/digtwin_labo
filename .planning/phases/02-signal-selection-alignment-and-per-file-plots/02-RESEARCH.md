%% Phase 2: Signal Selection, Alignment, and Per-File Plots - Research
%[text] 
%[text] **Researched:** 2026-04-03
%[text] **Domain:** MATLAB interactive UI (listdlg, uicontrol), Simulink.sdi.Signal data extraction, time-alignment via interp1, multi-subplot figure management
%[text] **Confidence:** HIGH
%[text] 
%[text] ---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01:** Scorer picks 2 signals from the **hardware run only**: one accel/torque command signal and one q2 signal. The script then finds matching signal names in the simulation run automatically.
- **D-02:** Signal list presented via `listdlg` with keyword-based scoring. Each signal name is scored against keyword lists (`'accel','torque','cmd'` for command; `'q2','theta','pend'` for angle). Sorted by score descending, then alphabetical. Likely candidates float to top.
- **D-03:** If a team already has a previous attempt with signal mappings, **offer the previous signal names as default** (pre-select in listdlg). Scorer confirms or re-picks. Fresh selection for first attempt of each team.
- **D-04:** Start time = first sample where accel/torque command is **nonzero** (no percentage threshold, no consecutive-sample hysteresis). Simple and deterministic. Applied independently to each run (hw and sim).
- **D-05:** After auto-detection, scorer is offered a **manual time delta** (seconds) to left-shift the hardware signal. Compensates for hardware dead time between accel command and q2 response. Default = 0, press Enter to accept.
- **D-06:** Manual delta is **per-attempt with team default**: pre-filled with the last delta used for that team, scorer can accept or change.
- **D-07:** After alignment (t=0 + manual delta), **crop both signals to their common time overlap**. No extrapolation, no NaN-padding.
- **D-08:** Normally both hw and sim are at **1 kHz** — no resampling needed. If rates differ, resample sim onto hw time grid via `interp1`. Defensive code, not the normal path.
- **D-09:** Per-attempt plot has **3 subplots** sharing a common aligned time axis: (1) top: hw q2 vs sim q2 overlay, (2) middle: accel/torque command signal, (3) bottom: q2 difference (hw - sim).
- **D-10:** Single persistent figure window, **reused** for each new attempt. A **uicontrol dropdown** on the figure lists all loaded attempts (team name + filename). Selecting a previous attempt re-draws the 3 subplots.
- **D-11:** The plot shows the detected start time (t=0) and any manual delta as annotations/markers.
- **D-12:** After signal selection and alignment, store the aligned time vector, aligned q2 signals (hw and sim), aligned command signal, detected start times, and manual delta in the attempt struct. This data feeds Phase 3 metric computation.

### Claude's Discretion

- Exact keyword lists and scoring weights for signal sorting — researcher should check actual signal names in example `.mldatx` files
- uicontrol dropdown implementation details (position, callback style)
- Color scheme and line styles for the overlay plot
- Interpolation method for `interp1` when resampling is needed (linear is fine)
- Whether the SIGM-03 preview plot (before confirming signal selection) is a separate figure or the same figure as the overlay plot

### Deferred Ideas (OUT OF SCOPE)

- **Live leaderboard figure** — A second persistent figure showing a continuously updated leaderboard of best runs per metric, refreshed after each attempt. Belongs in Phase 4 (Scoring and Output) since it requires the scoring rubric.

</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| SIGM-01 | Script presents all signals from a run in a `listdlg`, sorted so likely candidates (accel, torque, cmd, q2, theta, pend) appear first | `listdlg` with `InitialValue` for pre-selection verified. Keyword scoring pattern documented in Architecture Patterns. |
| SIGM-02 | Scorer manually picks accel/torque command signal and q2 signal for each team | Two separate `listdlg` calls with `SelectionMode='single'`. Pattern verified against Phase 1 established usage. |
| SIGM-03 | Script shows a preview plot of the selected signals before scorer confirms the mapping | Separate `figure()` (temporary) with 2 subplots before `input()` confirmation. Pattern documented in Architecture Patterns. |
| ALGN-01 | Script detects start time as first sample where `abs(accel_cmd) > threshold` (configurable, default ~1% of sat limit) | D-04 overrides: first nonzero sample (`abs(cmd) > 0`). `find(abs(cmd_data) > eps, 1, 'first')` pattern documented. |
| ALGN-02 | Script time-aligns hardware and simulation signals so t=0 corresponds to first non-zero command in each | `t_aligned = t_raw - t_start` pattern. Manual delta applied after. Crop to overlap documented. |
| OUTP-03 | Script produces an overlay plot immediately after each file is processed: hw q2 vs sim q2 (aligned) | Persistent figure with `clf`/redraw pattern. `uicontrol('Style','popupmenu')` dropdown for attempt browsing. Full pattern documented. |

</phase_requirements>

---

## Summary

Phase 2 extends `score_competition.m` with four new capabilities added as local functions inside the existing script: signal selection (`select_signals`), time alignment (`align_signals`), preview plot (`plot_preview`), and the persistent overlay figure with attempt dropdown (`update_overlay_figure`). All needed MATLAB API building blocks are verified in R2025b from Phase 1 research.

The two new interactive interactions are (1) two `listdlg` dialogs for command and q2 signal selection from the hardware run, and (2) an `input()` prompt for the optional manual time delta. The simulation run is never shown to the scorer — the script finds signals by matching the names chosen from the hardware run.

Signal data extraction uses `sig.Values` (returns a MATLAB `timeseries` object) with `.Time` and `.Data` fields — verified in Phase 1. Time alignment is a simple subtraction (`t_aligned = t_raw - t_start`), optionally shifted by the scorer-supplied delta. Crop to common overlap uses `max(t0, t0_ref)` / `min(tEnd, tEnd_ref)` index logic. Resampling (defensive path only) uses `interp1(t_sim, data_sim, t_hw, 'linear')`.

The persistent overlay figure uses a standard MATLAB `figure()` (not `uifigure`) with `uicontrol('Style','popupmenu')` for the attempt dropdown. The callback reads `get(src,'Value')` to determine which attempt to redraw, and the attempt list is stored in the figure's `UserData`. The preview plot for SIGM-03 uses a temporary `figure()` closed after confirmation.

**Primary recommendation:** Use `sig.Values.Time` and `sig.Values.Data` for all data extraction. Use `find(abs(cmd) > 0, 1, 'first')` for start detection. Use `uicontrol` popupmenu (not `uidropdown`) because the figure is a standard `figure()`, not a `uifigure`.

---

## Standard Stack

### Core

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `Simulink.sdi` | R2025b built-in | Access signal data from loaded runs | Only API for `.mldatx` format; verified Phase 1 |
| `listdlg` | R2025b built-in | Interactive signal selection from list | Established pattern in Phase 1; supports `InitialValue` |
| `input()` | R2025b built-in | Manual delta prompt | Consistent with existing session loop |
| `figure` / `subplot` | R2025b built-in | Preview and overlay plots | Standard MATLAB graphics |
| `uicontrol` | R2025b built-in | Attempt dropdown in overlay figure | Works with standard `figure()`; `uidropdown` requires `uifigure` |
| `interp1` | R2025b built-in | Resample sim onto hw time grid (defensive) | Standard; `'linear'` method sufficient |

### Supporting

| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `find` | MATLAB core | Locate first nonzero sample index | Start-time detection (D-04) |
| `xline` | R2025b built-in | Vertical line annotation on axes | Mark t=0 and delta on plots (D-11) |
| `linkaxes` | R2025b built-in | Synchronize x-axis zoom across subplots | Overlay figure (D-09, 3 subplots share time axis) |

**Installation:** No external packages. All built into MATLAB R2025b with Simulink.

**Version verification:** All packages are MATLAB built-ins in R2025b. No npm/pip installation needed.

---

## Architecture Patterns

### Recommended Script Extension Structure

```
score_competition.m (existing Phase 1 script)
├── %% Configuration          — add cfg.cmd_keywords, cfg.q2_keywords (D-02)
├── %% Session Loop           — existing while loop
│   ├── load_attempt()        — existing (Phase 1)
│   ├── select_signals()      — NEW Phase 2: two listdlg calls
│   ├── align_signals()       — NEW Phase 2: t=0 detection + crop + interp
│   ├── plot_preview()        — NEW Phase 2: SIGM-03 preview before confirm
│   ├── update_overlay_figure()  — NEW Phase 2: persistent 3-subplot figure
│   └── append attempt to session
├── %% Finalization           — existing
└── Local functions (before %[appendix]):
    ├── load_attempt          — existing Phase 1
    ├── select_signals        — NEW
    ├── align_signals         — NEW
    ├── plot_preview          — NEW (SIGM-03)
    └── update_overlay_figure — NEW (OUTP-03)
```

### Pattern 1: Keyword Scoring and listdlg Pre-Selection (SIGM-01, SIGM-02)

**What:** Score signal names against keyword lists. Sort by score descending. Pre-select the top candidate. If team has a prior mapping, pre-select that signal name's index.
**When to use:** Both command-signal and q2 signal selection dialogs.

```matlab
% Source: verified R2025b — listdlg InitialValue pre-selects by index
function [cmd_name, q2_name] = select_signals(hw_run, team)
    % Build signal name list from hardware run
    n = hw_run.SignalCount;
    names = cell(n, 1);
    for i = 1:n
        names{i} = hw_run.getSignalByIndex(i).Name;
    end

    % Keyword scoring: command signal
    cmd_kw = {'accel', 'torque', 'cmd', 'tau', 'input', 'ref'};
    q2_kw  = {'q2', 'theta', 'pend', 'angle', 'joint2', 'phi'};

    cmd_scores = score_names(names, cmd_kw);
    q2_scores  = score_names(names, q2_kw);

    % Sort for listdlg (stable sort: score desc, then original order)
    [~, cmd_order] = sort(cmd_scores, 'descend', 'stable');
    [~, q2_order]  = sort(q2_scores,  'descend', 'stable');

    cmd_list = names(cmd_order);
    q2_list  = names(q2_order);

    % Default pre-selection: top scored, or last used if team has prior mapping
    cmd_init = 1;  % index into sorted list
    q2_init  = 1;

    if isfield(team, 'last_cmd_signal') && ~isempty(team.last_cmd_signal)
        idx = find(strcmp(cmd_list, team.last_cmd_signal), 1);
        if ~isempty(idx), cmd_init = idx; end
    end
    if isfield(team, 'last_q2_signal') && ~isempty(team.last_q2_signal)
        idx = find(strcmp(q2_list, team.last_q2_signal), 1);
        if ~isempty(idx), q2_init = idx; end
    end

    % First dialog: command/accel signal
    sel = listdlg('ListString', cmd_list, ...
                  'SelectionMode', 'single', ...
                  'Name', 'Select Command Signal', ...
                  'PromptString', 'Pick the accel/torque command signal:', ...
                  'InitialValue', cmd_init, ...
                  'ListSize', [350, 250]);
    if isempty(sel)
        cmd_name = [];  % caller handles cancel
        q2_name  = [];
        return
    end
    cmd_name = cmd_list{sel};

    % Second dialog: q2 pendulum angle signal
    sel = listdlg('ListString', q2_list, ...
                  'SelectionMode', 'single', ...
                  'Name', 'Select q2 Signal', ...
                  'PromptString', 'Pick the pendulum angle (q2) signal:', ...
                  'InitialValue', q2_init, ...
                  'ListSize', [350, 250]);
    if isempty(sel)
        cmd_name = [];
        q2_name  = [];
        return
    end
    q2_name = q2_list{sel};
end

function scores = score_names(names, keywords)
    % Substring match, case-insensitive. Score = number of matching keywords.
    scores = zeros(numel(names), 1);
    for i = 1:numel(names)
        n_lower = lower(names{i});
        for k = 1:numel(keywords)
            if contains(n_lower, keywords{k})
                scores(i) = scores(i) + 1;
            end
        end
    end
end
```

**Key API detail (HIGH confidence, verified Phase 1):**
- `hw_run.getSignalByIndex(i).Name` — returns signal name as char
- `listdlg(..., 'InitialValue', N)` — pre-selects the N-th item; N is a scalar for `SelectionMode='single'`
- `listdlg` returns `[]` on cancel — must guard with `isempty(sel)`

### Pattern 2: Signal Data Extraction from SDI (HIGH confidence, verified Phase 1)

**What:** Extract time vector and data array from a `Simulink.sdi.Signal` object.
**When to use:** For every signal after selection — both hw and sim runs.

```matlab
% Source: verified experimentally in R2025b (Phase 1 research)
sig  = hw_run.getSignalsByName(cmd_name);
% getSignalsByName may return array if multiple matches — take first
if iscell(sig) || numel(sig) > 1
    sig = sig(1);
end
ts   = sig.Values;   % MATLAB timeseries object
t    = ts.Time;      % double column vector [s]
data = ts.Data;      % double column vector (same length as t)
```

**Sim run signal lookup by name:**
```matlab
% Source: verified R2025b API (Phase 1)
sim_run = Simulink.sdi.getRun(attempt.sim_run_id);
sim_q2_sigs = sim_run.getSignalsByName(q2_name);
if isempty(sim_q2_sigs)
    warning('scorer:sigsnotfound', ...
        'Signal "%s" not found in sim run. Signal names may differ.', q2_name);
    % Fallback: show listdlg for sim run if name lookup fails
end
```

**Note on getSignalsByName return type:** Returns a `Simulink.sdi.Signal` array (not cell array). Use `numel()` to check count. Index with `(1)` for first match.

### Pattern 3: Start-Time Detection and Alignment (ALGN-01, ALGN-02)

**What:** Find first nonzero command sample index, compute aligned time vector, apply manual delta, crop both signals to common overlap.
**When to use:** `align_signals` local function.

```matlab
% Source: standard MATLAB find() pattern
function aligned = align_signals(hw_t, hw_cmd, hw_q2, sim_t, sim_q2)
    % --- Step 1: Detect start time in each run independently ---
    % D-04: first nonzero command sample
    hw_start_idx = find(abs(hw_cmd) > 0, 1, 'first');
    if isempty(hw_start_idx)
        warning('scorer:nostarttime', ...
            'No nonzero command found in hw run. Using t=0.');
        hw_start_idx = 1;
    end
    hw_t_start = hw_t(hw_start_idx);

    % For sim: detect first nonzero command independently (if sim has cmd signal)
    % If sim cmd not available, use first sample of sim run
    sim_t_start = sim_t(1);   % sim starts at t=0 by construction in most runs

    % --- Step 2: Prompt for manual delta (D-05, D-06) ---
    % (caller passes team.last_delta as default; this function returns delta)
    % delta prompt is in the calling function, passed in as argument

    % --- Step 3: Align time vectors ---
    % hw: subtract hw start, add manual delta (left-shift = negative delta)
    hw_t_aligned = hw_t - hw_t_start;   % + delta applied by caller after crop
    sim_t_aligned = sim_t - sim_t_start;

    % --- Step 4: Crop to common overlap (D-07) ---
    t_start_common = max(hw_t_aligned(1), sim_t_aligned(1));
    t_end_common   = min(hw_t_aligned(end), sim_t_aligned(end));

    hw_mask  = hw_t_aligned  >= t_start_common & hw_t_aligned  <= t_end_common;
    sim_mask = sim_t_aligned >= t_start_common & sim_t_aligned <= t_end_common;

    hw_t_crop  = hw_t_aligned(hw_mask);
    hw_q2_crop = hw_q2(hw_mask);
    hw_cmd_crop = hw_cmd(hw_mask);

    sim_t_crop  = sim_t_aligned(sim_mask);
    sim_q2_crop = sim_q2(sim_mask);

    % --- Step 5: Resample sim onto hw time grid if rates differ (D-08) ---
    % Defensive: check if time grids match closely enough
    if numel(hw_t_crop) ~= numel(sim_t_crop) || ...
       max(abs(hw_t_crop - sim_t_crop)) > 1e-6
        % Resample sim onto hw grid via linear interpolation
        sim_q2_crop = interp1(sim_t_crop, sim_q2_crop, hw_t_crop, 'linear');
        sim_t_crop  = hw_t_crop;
    end

    % Return aligned struct
    aligned.t       = hw_t_crop;
    aligned.hw_q2   = hw_q2_crop;
    aligned.sim_q2  = sim_q2_crop;
    aligned.hw_cmd  = hw_cmd_crop;
    aligned.t_start = hw_t_start;   % absolute time in original hw run [s]
end
```

**Key detail on `find`:** `find(condition, 1, 'first')` returns a scalar index — the first `true` position. Returns `[]` if no match exists. Must guard against empty return.

**Key detail on `interp1`:** Signature is `interp1(x, v, xq, method)`. `x` must be monotonically increasing — verified because SDI time vectors are monotone. Default `'linear'` is sufficient; no extrapolation needed because we already cropped to overlap.

### Pattern 4: Preview Plot (SIGM-03)

**What:** Temporary figure showing the raw (unaligned) selected signals before scorer confirms.
**When to use:** Immediately after signal selection, before alignment begins.

```matlab
% Source: standard MATLAB subplot pattern
function plot_preview(hw_t, hw_cmd, hw_q2, cmd_name, q2_name)
    fig = figure('Name', 'Signal Preview — confirm or cancel', ...
                 'NumberTitle', 'off');
    ax1 = subplot(2, 1, 1);
    plot(hw_t, hw_cmd, 'b-');
    title(sprintf('Command signal: %s', cmd_name));
    ylabel('Command');
    xlabel('Time [s]');
    grid on;

    ax2 = subplot(2, 1, 2);
    plot(hw_t, hw_q2 * 180/pi, 'r-');
    title(sprintf('Pendulum angle: %s', q2_name));
    ylabel('q2 [deg]');
    xlabel('Time [s]');
    grid on;

    linkaxes([ax1, ax2], 'x');
    drawnow;

    % Scorer confirms or re-picks
    resp = input('Preview shown. Press Enter to confirm or type "repick" to re-select signals: ', 's');
    close(fig);
    confirmed = ~strcmpi(strtrim(resp), 'repick');
    % Return confirmed flag (or use a while loop in caller)
end
```

**Note on SIGM-03 scope (Claude's Discretion):** The CONTEXT.md leaves open whether the preview is a separate figure or the same as the overlay figure. A **separate temporary `figure()`** is simpler: no need to coordinate with the persistent overlay figure's dropdown state. Close it after confirmation. This is the recommended approach.

### Pattern 5: Persistent Overlay Figure with Attempt Dropdown (OUTP-03, D-09, D-10)

**What:** One `figure()` that persists across all attempts. 3 subplots. A `uicontrol` popup dropdown lists attempts by name. Selecting from dropdown redraws.
**When to use:** Created on first attempt; reused on subsequent calls via figure handle stored in session.

```matlab
% Source: verified R2025b — uicontrol popupmenu with UserData and callback
function fig = create_overlay_figure()
    fig = figure('Name', 'Attempt Overlay', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 650]);

    % Store attempt data in UserData (cell array of attempt structs)
    fig.UserData = struct('attempts', {{}}, 'labels', {{}});

    % Dropdown at top of figure (normalized units)
    uicontrol(fig, 'Style', 'popupmenu', ...
              'Units', 'normalized', ...
              'Position', [0.1, 0.95, 0.8, 0.04], ...
              'String', {'(no attempts yet)'}, ...
              'Tag', 'attempt_dropdown', ...
              'Callback', @overlay_dropdown_callback);
end

function update_overlay_figure(fig, attempt, label)
    % Add new attempt to UserData
    ud = fig.UserData;
    ud.attempts{end+1} = attempt;
    ud.labels{end+1}   = label;
    fig.UserData = ud;

    % Update dropdown string list
    dd = findobj(fig, 'Tag', 'attempt_dropdown');
    set(dd, 'String', ud.labels, 'Value', numel(ud.labels));

    % Draw the most recent attempt
    draw_attempt_subplots(fig, attempt);
end

function overlay_dropdown_callback(src, ~)
    fig = ancestor(src, 'figure');
    ud  = fig.UserData;
    idx = get(src, 'Value');
    if idx <= numel(ud.attempts)
        draw_attempt_subplots(fig, ud.attempts{idx});
    end
end

function draw_attempt_subplots(fig, attempt)
    % D-09: 3 subplots sharing a common aligned time axis
    figure(fig);   % bring to front

    % Clear previous subplots (keep the dropdown uicontrol)
    ax_list = findobj(fig, 'Type', 'axes');
    delete(ax_list);

    t   = attempt.aligned.t;
    hw  = attempt.aligned.hw_q2  * 180/pi;   % display in degrees
    sim = attempt.aligned.sim_q2 * 180/pi;
    cmd = attempt.aligned.hw_cmd;

    ax1 = subplot(3, 1, 1, 'Parent', fig);
    plot(ax1, t, hw,  'b-', 'DisplayName', 'Hardware q2');
    hold(ax1, 'on');
    plot(ax1, t, sim, 'r--', 'DisplayName', 'Simulation q2');
    ylabel(ax1, 'q2 [deg]');
    title(ax1, 'Hardware q2 vs Simulation q2 (aligned)');
    legend(ax1, 'show');
    grid(ax1, 'on');
    xline(ax1, 0, 'k--', 'DisplayName', 't=0 (start)');  % D-11

    ax2 = subplot(3, 1, 2, 'Parent', fig);
    plot(ax2, t, cmd, 'm-');
    ylabel(ax2, 'Command');
    title(ax2, 'Accel / Torque Command');
    grid(ax2, 'on');
    xline(ax2, 0, 'k--');   % D-11

    ax3 = subplot(3, 1, 3, 'Parent', fig);
    plot(ax3, t, hw - sim, 'g-');
    ylabel(ax3, 'hw-sim [deg]');
    xlabel(ax3, 'Aligned time [s]');
    title(ax3, 'q2 Difference (hw - sim)');
    grid(ax3, 'on');
    xline(ax3, 0, 'k--');   % D-11

    linkaxes([ax1, ax2, ax3], 'x');
    drawnow;
end
```

**Critical: `uicontrol` vs `uidropdown` (verified via WebSearch 2025):**
- `uicontrol('Style','popupmenu')` works with standard `figure()` — this is the correct choice here
- `uidropdown` (newer API) requires `uifigure` — incompatible with the standard `figure()` used in this script
- `get(src, 'Value')` returns the 1-based selected index for `popupmenu`
- `set(dd, 'String', cellarray)` updates the dropdown items

**Critical: `ancestor(src, 'figure')` (verified pattern):**
- In a `uicontrol` callback, `src` is the control handle
- `ancestor(src, 'figure')` returns the parent figure handle — safe way to access `UserData` without global variables

### Pattern 6: Attempt Struct Extension for Phase 2

**What:** Fields added to the attempt struct populated by Phase 2 functions. These feed Phase 3.

```matlab
% Phase 2 extends the attempt struct (Phase 1 fields: hw_run_id, sim_run_id,
% hw_file, sim_file, metrics)
attempt.signals.cmd_name    = cmd_name;   % char — hw run signal name
attempt.signals.q2_name     = q2_name;   % char — hw run signal name
attempt.signals.delta_s     = delta;     % double — manual delta [s] applied
attempt.signals.hw_t_start  = hw_t_start; % double — raw time of first nonzero cmd [s]
attempt.aligned.t           = t;         % double vector — common time axis [s], t(1)=0
attempt.aligned.hw_q2       = hw_q2;     % double vector — aligned hw pendulum angle [rad]
attempt.aligned.sim_q2      = sim_q2;    % double vector — aligned sim pendulum angle [rad]
attempt.aligned.hw_cmd      = hw_cmd;    % double vector — aligned command signal
```

### Anti-Patterns to Avoid

- **`uidropdown` with standard `figure()`:** Requires `uifigure`; throws "Parent must be a UIFigure or UIAxes" error. Use `uicontrol('Style','popupmenu')` instead.
- **Calling `delete(fig)` then recreating:** Destroys the dropdown state and attempt history. Use `clf` with selective deletion (keep uicontrol handles).
- **`clf(fig)` to reset the figure:** Deletes all children including the dropdown. Delete only axes objects via `findobj(fig,'Type','axes')`.
- **`sim_run.getSignalsByName` crashing when name not found:** Returns empty array (not error) in R2025b. Always check `isempty()` before indexing.
- **`interp1` with non-monotone time vector:** Throws error if `t` has repeated values (can happen with SDI buffer artifacts). Verify with `issorted(t)` and de-duplicate if needed.
- **`ts.Data` having a 3rd dimension (bus signals):** For multi-dimensional logged signals, `Data` may be N×1×1. Use `squeeze(ts.Data)` to ensure a column vector.
- **`find(abs(cmd) > 0, 1)` with floating-point zero:** Hardware ADC noise means `abs(cmd) > 0` fires on near-zero noise. If D-04's "nonzero" causes false positives, use `abs(cmd) > eps` or a tiny threshold (e.g., `1e-9`). Document in cfg.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Signal browsing dialog | Custom text-based signal picker | `listdlg` with `InitialValue` | Built-in modal dialog, handles scroll, keyboard, multi-OS |
| Attempt dropdown | Manual figure + axes + text objects | `uicontrol('Style','popupmenu')` | Single call; callback system already integrated |
| Time-series resampling | Manual nearest-neighbor loop | `interp1(t_sim, data_sim, t_hw, 'linear')` | Handles edge cases, vectorized, error-checked |
| Keyword fuzzy match | Edit distance or regex | `contains(lower(name), keyword)` | Sufficient for this domain; simple, debuggable |
| Start-time detection | Rolling window energy detector | `find(abs(cmd) > eps, 1, 'first')` | D-04 specifies simple first-nonzero; no complexity needed |

**Key insight:** The Phase 2 UX is intentionally minimal (one scorer, run once per year). Robust solutions beat clever ones — `listdlg`, `find`, and `interp1` cover all requirements without hidden complexity.

---

## Known Signal Names (from project codebase, MEDIUM confidence)

The Simulink models log signals whose names appear in the SDI and in `.mldatx` files. Based on codebase inspection:

| Signal | MATLAB Variable | Likely SDI Name Pattern | Role |
|--------|----------------|------------------------|------|
| Pendulum angle | `q2` | `q2`, `q2_rev`, `pendulum_angle`, `theta2` | q2 selection |
| Arm torque command | `tau_1`, `tau1` | `tau_1`, `tau1`, `torque_cmd`, `tau_ref` | cmd selection |
| Angular acceleration (stepper) | `alpha1` | `accel_cmd`, `alpha1`, `accel_ref` | cmd selection (stepper) |
| Arm angle | `q1` | `q1`, `q1_rev` | not selected (but present) |

**Phase 1 research confirmed:** `r.getSignalsByName('q2_rev')` was used in Phase 1 testing, confirming `q2_rev` is a real signal name in the project's `.mldatx` files. The `_rev` suffix likely indicates encoder revolutions or the raw counter before conversion to radians.

**Implication for keyword heuristic:** Include `'q2'`, `'rev'` (lower priority) in q2 keywords. Include `'tau'` in command keywords (covers `tau_1` and `tau1`). The BLDC model uses `'accel'` for the command type.

**Recommended keyword lists (Claude's Discretion items):**
```matlab
cfg.cmd_keywords = {'accel', 'torque', 'cmd', 'tau', 'ref', 'input'};
cfg.q2_keywords  = {'q2', 'theta', 'pend', 'angle', 'phi', 'joint2'};
```

The keyword `'rev'` is intentionally excluded from the q2 list because `q1_rev` would also score, causing ambiguity. The `q2` literal is a stronger match.

---

## Common Pitfalls

### Pitfall 1: `clf` Deletes the Dropdown
**What goes wrong:** Calling `clf(fig)` to clear the overlay figure before redrawing also deletes the `uicontrol` dropdown and its `UserData`.
**Why it happens:** `clf` removes all children of the figure, including `uicontrol` objects.
**How to avoid:** Delete only `axes` objects: `delete(findobj(fig, 'Type', 'axes'))`. Leave the `uicontrol` intact.
**Warning signs:** Dropdown disappears after second attempt is loaded.

### Pitfall 2: `getSignalsByName` Returns Empty (Sim Name Mismatch)
**What goes wrong:** Scorer picks `q2_rev` from hardware run; the sim run has `q2_sim` or `q2` — name doesn't match, `getSignalsByName` returns empty.
**Why it happens:** Teams may use different signal naming in hardware vs simulation models.
**How to avoid:** After calling `getSignalsByName` on the sim run, check for empty. If empty, fall back to showing a `listdlg` for the sim run with the same q2 keyword sorting. Display a clear message: "Signal 'X' not found in sim run — please pick manually."
**Warning signs:** `isempty(sim_q2_sigs)` is true on first team.

### Pitfall 3: `ts.Data` Is 3D Array
**What goes wrong:** Bus signals or multi-output blocks log data as an N×1×1 array. `ts.Data` has a third singleton dimension. `plot(t, data)` fails ("data must be a vector").
**Why it happens:** Simulink scalar signals logged through bus signals carry extra dimensions in the timeseries object.
**How to avoid:** Always apply `squeeze()` after reading `.Data`: `data = squeeze(ts.Data);`
**Warning signs:** `size(ts.Data)` is `[N, 1, 1]` or `[N, 1]` where the signal should be `[N, 1]`.

### Pitfall 4: Manual Delta Direction Convention
**What goes wrong:** Scorer enters a positive delta intending to "move the hardware earlier" (to account for dead time), but the code interprets positive delta as right-shift (later).
**Why it happens:** "Compensate for dead time" means the hardware response is delayed relative to the command — subtract delta from hardware time to shift it left.
**How to avoid:** Document the convention explicitly in the `input()` prompt: "Enter delta in seconds to LEFT-SHIFT hardware signal (positive = earlier start, default=0)". Apply as `hw_t_aligned = hw_t_aligned - delta`.
**Warning signs:** SMAPE increases with delta instead of decreasing.

### Pitfall 5: `interp1` Fails on Duplicate Time Values
**What goes wrong:** SDI occasionally logs duplicate time stamps in the same buffer flush. `interp1` requires strictly monotone `x` and throws if duplicates exist.
**Why it happens:** SDI buffer overflow or logging rate mismatch.
**How to avoid:** After extracting `t = ts.Time`, apply uniqueness: `[t, ia] = unique(t); data = data(ia);` before any alignment or interpolation.
**Warning signs:** Error: "The sample points X must be unique."

### Pitfall 6: `listdlg` InitialValue Index Out of Range
**What goes wrong:** Team has `last_q2_signal = 'q2_old'` from a prior session, but that signal doesn't exist in the new team's file. `find()` returns empty, and `InitialValue = []` causes `listdlg` to error.
**Why it happens:** Stale team defaults from a different team's signal set.
**How to avoid:** After computing `idx = find(...)`, check `if isempty(idx); idx = 1; end` before passing to `listdlg`.
**Warning signs:** `listdlg` throws "MATLAB:listdlg:InitialValueOutOfRange".

---

## Code Examples

Verified patterns from official sources and Phase 1 testing:

### Extract Signal Data from SDI Run
```matlab
% Source: verified experimentally in R2025b (Phase 1 research)
sig  = run_obj.getSignalsByName('q2_rev');  % returns Signal array
if isempty(sig)
    error('Signal not found');
end
sig  = sig(1);                              % first match if multiple
ts   = sig.Values;                          % timeseries object
t    = ts.Time;                             % [s], double column vector
data = squeeze(ts.Data);                    % double column vector (squeeze for safety)
```

### listdlg with Pre-Selection
```matlab
% Source: verified MATLAB R2025b docs — InitialValue pre-selects item N
[sel, ok] = listdlg('ListString', {'Signal_A', 'Signal_B', 'Signal_C'}, ...
                    'SelectionMode', 'single', ...
                    'Name', 'Pick Command Signal', ...
                    'PromptString', 'Select the accel/torque command signal:', ...
                    'InitialValue', 2, ...
                    'ListSize', [350, 250]);
% sel = 2 if user accepts default; ok = 1 if OK pressed, 0 if cancelled
if ~ok || isempty(sel)
    % user cancelled
end
chosen_name = signal_names{sel};
```

### uicontrol Popupmenu in Standard Figure
```matlab
% Source: verified MATLAB docs — works with standard figure(), NOT uifigure
fig = figure('Name', 'Overlay', 'Position', [100 100 900 650]);
fig.UserData = struct('attempts', {{}}, 'labels', {{}});

dd = uicontrol(fig, 'Style', 'popupmenu', ...
               'Units', 'normalized', ...
               'Position', [0.1, 0.95, 0.8, 0.04], ...
               'String', {'(none)'}, ...
               'Tag', 'attempt_dropdown', ...
               'Callback', @my_callback);

% In callback: get selected index
function my_callback(src, ~)
    idx = get(src, 'Value');
    fig = ancestor(src, 'figure');
    ud  = fig.UserData;
    % redraw using ud.attempts{idx}
end
```

### Time Alignment with Crop
```matlab
% Source: standard MATLAB array indexing (no external library)
t_hw  = ts_hw.Time;
t_sim = ts_sim.Time;
t_hw_aligned  = t_hw  - t_hw(find(abs(cmd_hw)  > 0, 1, 'first'));
t_sim_aligned = t_sim - t_sim(find(abs(cmd_sim) > 0, 1, 'first'));

t0 = max(t_hw_aligned(1),   t_sim_aligned(1));
t1 = min(t_hw_aligned(end), t_sim_aligned(end));

hw_mask  = t_hw_aligned  >= t0 & t_hw_aligned  <= t1;
sim_mask = t_sim_aligned >= t0 & t_sim_aligned <= t1;
t_common = t_hw_aligned(hw_mask);
```

### Resampling Sim onto HW Grid (Defensive)
```matlab
% Source: MATLAB interp1 documentation
% x = known time, v = known values, xq = query time
q2_sim_resampled = interp1(t_sim_crop, q2_sim_crop, t_hw_crop, 'linear');
% Requirement: t_sim_crop must be monotonically increasing (verified by unique() step)
```

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `guidata` for figure state sharing | `figure.UserData` struct | R2014b+ | `UserData` is simpler and avoids GUIDE dependency |
| `uicontrol 'popupmenu'` for dropdowns | `uidropdown` (App Designer) | R2019b | `uidropdown` requires `uifigure`; `uicontrol` popupmenu still works for plain `figure()` |
| `timeseries` arithmetic for alignment | Manual index arithmetic + `interp1` | — | Requirements spec forbids timeseries arithmetic (METR-04 explicitly says use `interp1`, not timeseries arithmetic) |
| `gca` / `gcf` for axes access | Explicit axes handles | Always preferred | `gca`/`gcf` is fragile when multiple figures exist; use explicit handles stored in struct or `findobj` |

**Deprecated/outdated:**
- `timeseries/synchronize` for alignment: Forbidden by METR-04. Do not use MATLAB's `synchronize` on timeseries objects — use `interp1` explicitly.
- `GUIDE` (`guide()` function): Deprecated in R2021a. Do not start App Designer or GUIDE workflows. All UI is inline with `uicontrol`.

---

## Open Questions

1. **Sim run signal name mismatch**
   - What we know: D-01 says script finds matching names in sim run automatically by name. Phase 1 research shows signals are accessed by name via `getSignalsByName`.
   - What's unclear: How often will sim run signal names differ from hw run signal names? The project uses the same Simulink model for both, so names are likely identical. But teams may have modified their model.
   - Recommendation: Implement the fallback `listdlg` for sim run when `getSignalsByName` returns empty. Flag it with a `warning()` so the scorer is aware.

2. **Preview plot figure placement**
   - What we know: D-03 says a preview plot is shown before scorer confirms signal mapping (SIGM-03). CONTEXT.md leaves it to Claude's Discretion whether it's the same figure as the overlay or a separate one.
   - What's unclear: If the overlay figure is not yet created (first attempt), using the overlay figure for preview avoids extra window. But it complicates the figure lifecycle.
   - Recommendation: Use a separate temporary `figure()` for the preview. Close it after the confirm/repick prompt. Simpler lifecycle, no side effects on the overlay figure.

3. **`q2_rev` vs `q2` signal name ambiguity**
   - What we know: Phase 1 research confirmed `q2_rev` exists as a signal name in the project's `.mldatx` files. It is unclear whether `q2_rev` is the raw encoder count in revolutions or a renamed q2 signal.
   - What's unclear: Whether teams have signal names like `q2_rad` (converted), `q2_rev` (raw encoder), or just `q2`.
   - Recommendation: Both `q2` and `rev` appear in the q2 keyword list. The top-scored match will appear first in `listdlg`. Scorer confirmation (D-03 preview) provides the final safety check.

---

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| MATLAB R2025b | Runtime | Yes | R2025b (confirmed via Phase 1) | — |
| Simulink (SDI) | Signal extraction | Yes | R2025b built-in | No fallback |
| `uicontrol` | Dropdown (D-10) | Yes | Base MATLAB | — |
| `listdlg` | Signal selection (D-02) | Yes | Base MATLAB | — |
| `interp1` | Resampling (D-08) | Yes | Base MATLAB | — |
| `xline` | t=0 annotation (D-11) | Yes | R2018b+ in R2025b | `line([0 0], ylim)` fallback if needed |
| `linkaxes` | Shared x-axis (D-09) | Yes | Base MATLAB | — |
| Display / GUI | All UI dialogs | Yes | Windows 11 interactive | — |
| `example_data/combined_test.mldatx` | Testing | Yes | Confirmed in project repo | — |

**Missing dependencies with no fallback:** None.

**Missing dependencies with fallback:** None.

---

## Project Constraints (from CLAUDE.md)

These directives apply to all code written in Phase 2:

1. **Project open check:** Every script must begin with `if isempty(matlab.project.rootProject); openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj'); end`
2. **Live Script .m format:** Use `%[text]` for rich text, `%%` for section breaks, `%[appendix]{"version":"1.0"}` at end. Escape `\_` inside `%[text]` blocks.
3. **Git commit rule:** Only commit files registered in `digtwin_labo.prj`. `score_competition.m` must already be registered from Phase 1.
4. **File location:** `scripts/score_competition.m` — same file as Phase 1, extended with new local functions.
5. **Config struct pattern:** New Phase 2 parameters (keyword lists, etc.) go in the existing `cfg` struct at the top of the script.
6. **No App Designer / no `uifigure`:** Plain script with `uicontrol` on standard `figure()` — consistent with CLAUDE.md's "no GUI" constraint and the `uidropdown`/`uifigure` incompatibility finding.
7. **Local functions in script:** MATLAB R2025b supports local functions in scripts. All Phase 2 functions are local (not separate files), placed after `%% Finalization` and before `%[appendix]`.

---

## Sources

### Primary (HIGH confidence)

- Direct MATLAB R2025b execution (Phase 1 research) — all SDI API calls verified:
  - `run.getSignalByIndex(i).Name` — confirmed working
  - `run.getSignalsByName(name)` — confirmed working, returns empty array if not found
  - `sig.Values` returns `timeseries`; `.Time` and `.Data` fields confirmed
  - `listdlg` with `InitialValue` — verified from official docs (pre-selects by index)
- `example_data/combined_test.mldatx` — 2-run test file confirmed in project repo
- Phase 1 research (`01-RESEARCH.md`) — complete SDI API surface map with verified behavior

### Secondary (MEDIUM confidence)

- [MATLAB listdlg documentation](https://www.mathworks.com/help/matlab/ref/listdlg.html) — `InitialValue` behavior confirmed via WebSearch cross-reference
- [MATLAB uicontrol documentation](https://www.mathworks.com/help/matlab/ref/uicontrol.html) — `popupmenu` style and `Value` property confirmed
- [MathWorks blog: From uicontrol to UI Components](https://blogs.mathworks.com/graphics-and-apps/2025/08/19/__from-uicontrol-to-ui-components/) — confirms `uidropdown` requires `uifigure`; `uicontrol popupmenu` remains valid for standard `figure()`
- Project codebase signal names (`q2_rev` from Phase 1, `tau_1`/`tau1` from scripts) — MEDIUM confidence (actual SDI names depend on Simulink logging configuration in each team's model)

### Tertiary (LOW confidence)

- Signal name patterns for team models — inferred from project scripts. Actual names in student-submitted `.mldatx` files may differ. Keyword heuristic designed to be tolerant.

---

## Metadata

**Confidence breakdown:**
- SDI signal data extraction (`sig.Values.Time/.Data`): HIGH — verified Phase 1
- `listdlg` with `InitialValue`: HIGH — verified via official docs + WebSearch
- `uicontrol popupmenu` on standard `figure()`: HIGH — confirmed by MathWorks blog (2025) and docs
- `interp1` for resampling: HIGH — stable MATLAB core function
- `xline` availability: HIGH — introduced R2018b, confirmed in R2025b environment
- Signal names in student `.mldatx` files: MEDIUM — keyword heuristic provides tolerance; actual names unknown until competition day

**Research date:** 2026-04-03
**Valid until:** 2026-05-03 (MATLAB built-ins are stable; SDI API unchanged since R2025b; 30-day window is safe)

%[appendix]{"version":"1.0"}
