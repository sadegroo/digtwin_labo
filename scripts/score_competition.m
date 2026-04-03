%% Swingup Competition Scoring
%[text] Scores team swingup attempts from .mldatx files. Session-based: load .mldatx
%[text] files one at a time, assign to teams, finalize to produce leaderboard.
clear
clc

if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end

%% Configuration
%[text] All tunable parameters. Edit this section before running.

cfg = struct();
cfg.teams(1) = struct('name', 'Team Alpha', 'type', 'stepper');
cfg.teams(2) = struct('name', 'Team Beta',  'type', 'stepper');
cfg.teams(3) = struct('name', 'Team Gamma', 'type', 'stepper');
cfg.teams(4) = struct('name', 'Team Delta', 'type', 'stepper');
cfg.teams(5) = struct('name', 'BLDC Team',  'type', 'bldc');

cfg.smape_window           = 'fixed';   % 'fixed', 'angle', or 'swingup' (Phase 3)
cfg.smape_fixed_duration   = 5;         % seconds (Phase 3)
cfg.swingup_hold_time      = 1.0;       % seconds pendulum must hold at +/-pi (Phase 3)
cfg.swingup_tolerance_deg  = 2;         % degrees tolerance around +/-pi (Phase 3)
cfg.participation_threshold = pi/2;     % rad -- |q2| must exceed this (Phase 3)
cfg.q2_unit = 'rev';  % 'rev' (default), 'deg', or 'rad' — display unit for q2 plots
cfg.cmd_keywords = {'accel', 'torque', 'cmd', 'tau', 'ref', 'input'};
cfg.q2_keywords  = {'q2', 'theta', 'pend', 'angle', 'phi', 'joint2'};

%% Session Initialization
%[text] Initialize the session state struct that accumulates results across all loaded files.

session = struct();
session.teams = cfg.teams;
for i = 1:numel(session.teams)
    session.teams(i).attempts        = {};
    session.teams(i).last_cmd_signal = '';
    session.teams(i).last_q2_signal  = '';
    session.teams(i).last_delta      = 0;
end
session.finalized = false;

fprintf('Session started. %d teams configured.\n', numel(cfg.teams));

% Create the persistent overlay figure (Phase 2 -- OUTP-03)
overlay_fig = create_overlay_figure(cfg.q2_unit);

%% Session Loop
%[text] Interactive loop: load files, assign to teams. Type **done** to finalize.

team_names = {cfg.teams.name};

while true
    %% Prompt for attempt file
    %[text] Select the **.mldatx** file for this attempt (contains both hw and sim runs).
    [fname, fdir] = uigetfile('*.mldatx', 'Select .mldatx file for this attempt');
    if isequal(fname, 0)
        % User cancelled file dialog
        cmd = input('File selection cancelled. Type ''done'' to finalize or press Enter to retry: ', 's');
        if strcmpi(strtrim(cmd), 'done')
            break
        end
        continue
    end

    %% Load and validate attempt
    attempt = load_attempt(fullfile(fdir, fname));
    if isempty(attempt)
        % load_attempt already printed a warning
        fprintf('Skipping this file. Try again.\n');
        continue
    end

    %% Assign to team FIRST (team needed for signal defaults -- D-03, D-06)
    sel = listdlg('ListString', team_names, ...
                  'SelectionMode', 'single', ...
                  'Name', 'Assign to Team', ...
                  'PromptString', sprintf('Assign "%s" to which team?', fname), ...
                  'ListSize', [300, 200]);
    if isempty(sel)
        fprintf('Team selection cancelled. Skipping this file.\n');
        continue
    end
    team_idx = sel;
    team     = session.teams(team_idx);

    %% Signal selection with repick loop (SIGM-01, SIGM-02, SIGM-03, D-01, D-02, D-03)
    hw_run = Simulink.sdi.getRun(attempt.hw_run_id);

    signal_ok = false;
    while true
        [cmd_name, q2_name] = select_signals(hw_run, team, cfg.cmd_keywords, cfg.q2_keywords);
        if isempty(cmd_name)
            fprintf('Signal selection cancelled.\n');
            break   % exit inner loop; signal_ok remains false
        end

        % Extract hw signals for preview (SIGM-03)
        [hw_t_cmd, hw_cmd_data] = extract_signal(hw_run, cmd_name);
        [hw_t_q2,  hw_q2_data]  = extract_signal(hw_run, q2_name);
        if isempty(hw_t_cmd) || isempty(hw_t_q2)
            warning('scorer:extractfail', 'Failed to extract selected signals. Re-pick.');
            continue
        end

        % Defensive resample: cmd and q2 may have different time bases
        % Resample q2 onto cmd's time grid so we have one consistent hw_t.
        if ~isequal(hw_t_cmd, hw_t_q2)
            hw_q2_data = interp1(hw_t_q2, hw_q2_data, hw_t_cmd, 'linear');
        end

        % Preview plot -- scorer confirms or repicks (SIGM-03)
        confirmed = plot_preview(hw_t_cmd, hw_cmd_data, hw_q2_data, cmd_name, q2_name, cfg.q2_unit);
        if confirmed
            signal_ok = true;
            break
        end
        fprintf('Re-picking signals...\n');
    end

    if ~signal_ok
        fprintf('Skipping file.\n');
        continue
    end

    %% Find matching signals in sim run (D-01)
    sim_run = Simulink.sdi.getRun(attempt.sim_run_id);

    [sim_t_cmd, sim_cmd_data] = extract_signal(sim_run, cmd_name);
    [sim_t_q2,  sim_q2_data]  = extract_signal(sim_run, q2_name);

    % Fallback: if sim command signal not found, pick manually (Research Pitfall 2)
    if isempty(sim_t_cmd)
        fprintf('Signal "%s" not found in sim run. Pick manually.\n', cmd_name);
        [sim_cmd_name, ~] = select_signals(sim_run, struct(), cfg.cmd_keywords, cfg.q2_keywords);
        if ~isempty(sim_cmd_name)
            [sim_t_cmd, sim_cmd_data] = extract_signal(sim_run, sim_cmd_name);
        end
    end

    % Fallback: if sim q2 signal not found, pick manually
    if isempty(sim_t_q2)
        fprintf('Signal "%s" not found in sim run. Pick manually.\n', q2_name);
        [~, sim_q2_name] = select_signals(sim_run, struct(), cfg.cmd_keywords, cfg.q2_keywords);
        if ~isempty(sim_q2_name)
            [sim_t_q2, sim_q2_data] = extract_signal(sim_run, sim_q2_name);
        end
    end

    % Defensive resample for sim signals: q2 onto cmd's time grid
    if ~isequal(sim_t_cmd, sim_t_q2)
        sim_q2_data = interp1(sim_t_q2, sim_q2_data, sim_t_cmd, 'linear');
    end

    %% Manual delta prompt -- LEFT-SHIFT hardware by delta seconds (D-05, D-06)
    default_delta = session.teams(team_idx).last_delta;
    delta_str = input( ...
        sprintf('Manual time delta to LEFT-SHIFT hardware [s] (default=%.3f, Enter to accept): ', ...
                default_delta), 's');
    if isempty(strtrim(delta_str))
        delta = default_delta;
    else
        delta = str2double(delta_str);
        if isnan(delta)
            fprintf('Invalid delta, using default %.3f s.\n', default_delta);
            delta = default_delta;
        end
    end

    %% Align signals (ALGN-01, ALGN-02)
    % hw_t_cmd is the single hw time vector (hw_q2 was resampled onto it above).
    % sim_t_cmd is the single sim time vector (sim_q2 was resampled onto it above).
    aligned = align_signals(hw_t_cmd, hw_cmd_data, hw_q2_data, ...
                            sim_t_cmd, sim_cmd_data, sim_q2_data, delta);

    %% Store aligned data and signal metadata in attempt struct (D-12)
    attempt.signals.cmd_name = cmd_name;
    attempt.signals.q2_name  = q2_name;
    attempt.signals.delta_s  = delta;
    attempt.aligned          = aligned;

    %% Update team defaults (D-03, D-06)
    session.teams(team_idx).last_cmd_signal = cmd_name;
    session.teams(team_idx).last_q2_signal  = q2_name;
    session.teams(team_idx).last_delta      = delta;

    %% Append attempt to team and update overlay figure (OUTP-03)
    session.teams(team_idx).attempts{end+1} = attempt;
    label = sprintf('%s — %s', session.teams(team_idx).name, fname);
    update_overlay_figure(overlay_fig, attempt, label);

    % Print session progress
    fprintf('\nAdded attempt %d for team "%s" (%s).\n', ...
        numel(session.teams(team_idx).attempts), ...
        session.teams(team_idx).name, ...
        session.teams(team_idx).type);
    fprintf('\n--- Session Progress ---\n');
    for i = 1:numel(session.teams)
        fprintf('  %s: %d attempt(s)\n', session.teams(i).name, numel(session.teams(i).attempts));
    end
    fprintf('\n');

    %% Continue or finalize (per D-03, D-04)
    cmd = input('Load another file? [Enter] to continue, ''done'' to finalize: ', 's');
    if strcmpi(strtrim(cmd), 'done')
        break
    end
end

%% Finalization
%[text] Session complete. Mark as finalized and display summary.

session.finalized = true;
fprintf('\n========================================\n');
fprintf('SESSION FINALIZED\n');
fprintf('========================================\n');
fprintf('Teams: %d\n', numel(session.teams));
for i = 1:numel(session.teams)
    fprintf('  %s (%s): %d attempt(s)\n', ...
        session.teams(i).name, ...
        session.teams(i).type, ...
        numel(session.teams(i).attempts));
end
fprintf('\nSession state is in workspace variable "session".\n');
fprintf('Proceed to Phase 3 (metric computation).\n');

function attempt = load_attempt(file)
%LOAD_ATTEMPT Load a single .mldatx file containing both hw and sim runs.
%   attempt = LOAD_ATTEMPT(file) clears SDI, loads one .mldatx file,
%   asserts exactly 2 runs, discriminates hw/sim by run ordering
%   (ids(1) = archived = hardware, ids(end) = most recent = simulation),
%   validates with SimMode, and returns an attempt struct. Returns [] on failure.

    attempt = [];

    % Clear SDI to prevent run contamination (LOAD-01, per D-02)
    Simulink.sdi.clear;

    % Load the single file (contains both hw and sim runs)
    try
        Simulink.sdi.load(file);
    catch e
        warning('scorer:loadfail', 'Failed to load file: %s', e.message);
        return
    end

    % Assert exactly 2 runs (per D-02)
    ids = Simulink.sdi.getAllRunIDs();
    if numel(ids) ~= 2
        warning('scorer:badruncount', ...
            'Expected 2 runs after loading file, got %d. Skipping.', numel(ids));
        return
    end

    % Primary: run ordering -- first ID = archived (hw), last ID = most recent (sim)
    hw_id  = ids(1);
    sim_id = ids(end);

    % Access both runs
    hw_run  = Simulink.sdi.getRun(hw_id);
    sim_run = Simulink.sdi.getRun(sim_id);

    % Display run summary (per D-01)
    fprintf('\n--- Run Summary ---\n');
    fprintf('Run 1 (hw):  "%s" | SimMode: %s | Signals: %d | Time: [%.3f, %.3f] s\n', ...
        hw_run.Name, hw_run.SimMode, hw_run.SignalCount, hw_run.StartTime, hw_run.StopTime);
    fprintf('Run 2 (sim): "%s" | SimMode: %s | Signals: %d | Time: [%.3f, %.3f] s\n', ...
        sim_run.Name, sim_run.SimMode, sim_run.SignalCount, sim_run.StartTime, sim_run.StopTime);

    % Validate with SimMode (warn if inconsistent, but do not override ordering)
    if ~isempty(hw_run.SimMode) && ~strcmp(hw_run.SimMode, 'external')
        fprintf('  Note: hardware run SimMode is ''%s'' (expected ''external'')\n', hw_run.SimMode);
    end
    if ~isempty(sim_run.SimMode) && ~strcmp(sim_run.SimMode, 'normal')
        fprintf('  Note: simulation run SimMode is ''%s'' (expected ''normal'')\n', sim_run.SimMode);
    end

    % Display assignment
    fprintf('  -> Hardware run: "%s" (SimMode: %s)\n', hw_run.Name, hw_run.SimMode);
    fprintf('  -> Simulation run: "%s" (SimMode: %s)\n', sim_run.Name, sim_run.SimMode);

    % Ask scorer to confirm or swap (per D-01)
    fprintf('  [Press Enter to confirm, or type "swap" to swap assignment]\n');
    resp = input('  Confirm? ', 's');
    if strcmpi(strtrim(resp), 'swap')
        temp   = hw_id;
        hw_id  = sim_id;
        sim_id = temp;
        fprintf('  -> Swapped. Hardware: "%s", Simulation: "%s"\n', ...
            Simulink.sdi.getRun(hw_id).Name, Simulink.sdi.getRun(sim_id).Name);
    end

    % Build attempt struct (per D-08)
    attempt             = struct();
    attempt.hw_run_id   = hw_id;
    attempt.sim_run_id  = sim_id;
    attempt.file        = file;
    attempt.metrics     = struct();   % placeholder for Phase 3
end

%[text]
%[text] ---
%[text] **Local Functions: Signal Selection, Extraction, Preview, and Alignment**
%[text] These functions are the computational building blocks for Phase 2.

function [cmd_name, q2_name] = select_signals(hw_run, team, cmd_keywords, q2_keywords)
%SELECT_SIGNALS Present sorted signal lists and let scorer pick command and q2 signals.
%   [cmd_name, q2_name] = SELECT_SIGNALS(hw_run, team, cmd_keywords, q2_keywords)
%   builds a list of all signal names from the hardware run, scores each name
%   against cmd\_keywords / q2\_keywords, sorts highest-scoring names to the top,
%   and presents two listdlg dialogs.  Returns [] for both names on cancel.
%
%   If team.last\_cmd\_signal / team.last\_q2\_signal are set, those names are
%   pre-selected (D-03 team default).

    % Build signal name list from hardware run
    n = hw_run.SignalCount;
    names = cell(n, 1);
    for i = 1:n
        names{i} = hw_run.getSignalByIndex(i).Name;
    end

    % Score and sort for command signal
    cmd_scores = score_names(names, cmd_keywords);
    [~, cmd_order] = sort(cmd_scores, 'descend');
    cmd_list = names(cmd_order);

    % Score and sort for q2 signal
    q2_scores = score_names(names, q2_keywords);
    [~, q2_order] = sort(q2_scores, 'descend');
    q2_list = names(q2_order);

    % Default pre-selection: top scored (index 1)
    cmd_init = 1;
    q2_init  = 1;

    % D-03: use previous signal names as default if available
    if isfield(team, 'last_cmd_signal') && ~isempty(team.last_cmd_signal)
        idx = find(strcmp(cmd_list, team.last_cmd_signal), 1);
        if ~isempty(idx), cmd_init = idx; end
    end
    if isfield(team, 'last_q2_signal') && ~isempty(team.last_q2_signal)
        idx = find(strcmp(q2_list, team.last_q2_signal), 1);
        if ~isempty(idx), q2_init = idx; end
    end

    % First dialog: command/accel signal (SIGM-01, SIGM-02)
    sel = listdlg('ListString', cmd_list, ...
                  'SelectionMode', 'single', ...
                  'Name', 'Select Command Signal', ...
                  'PromptString', 'Pick the accel/torque command signal:', ...
                  'InitialValue', cmd_init, ...
                  'ListSize', [350, 250]);
    if isempty(sel)
        cmd_name = [];
        q2_name  = [];
        return
    end
    cmd_name = cmd_list{sel};

    % Second dialog: q2 pendulum angle signal (SIGM-01, SIGM-02)
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
%SCORE_NAMES Score each signal name by how many keywords it contains.
%   scores = SCORE_NAMES(names, keywords) returns a numeric vector with one
%   score per element of names.  Score = number of keyword substrings found
%   in the lowercase signal name (case-insensitive substring matching).

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

%[text] **convert\_q2** — Converts q2 from radians to the configured display unit and returns the axis label.
function [data_out, label] = convert_q2(data_rad, unit)
    switch unit
        case 'rev'
            data_out = data_rad / (2*pi);
            label = 'q2 [rev]';
        case 'deg'
            data_out = data_rad * 180/pi;
            label = 'q2 [deg]';
        case 'rad'
            data_out = data_rad;
            label = 'q2 [rad]';
        otherwise
            data_out = data_rad / (2*pi);
            label = 'q2 [rev]';
    end
end

function [t, data] = extract_signal(run_obj, signal_name)
%EXTRACT_SIGNAL Extract time vector and data from an SDI run by signal name.
%   [t, data] = EXTRACT_SIGNAL(run\_obj, signal\_name) looks up signal\_name in
%   run\_obj, extracts the timeseries, and returns time t and data as double
%   column vectors.  Applies unique(t) to guard against duplicate timestamps
%   (Pitfall 5 -- prevents interp1 failure).  Returns t=[] and data=[] if the
%   signal is not found.

    sigs = run_obj.getSignalsByName(signal_name);
    if isempty(sigs)
        t    = [];
        data = [];
        return
    end
    sig  = sigs(1);
    ts   = sig.Values;
    t    = ts.Time;
    data = squeeze(ts.Data);

    % Guard against duplicate timestamps (interp1 requires strictly monotonic t)
    [t, ia] = unique(t);
    data = data(ia);
end

function confirmed = plot_preview(hw_t, hw_cmd, hw_q2, cmd_name, q2_name, q2_unit)
%PLOT_PREVIEW Show a 2-subplot preview of selected signals before confirming.
%   confirmed = PLOT_PREVIEW(hw\_t, hw\_cmd, hw\_q2, cmd\_name, q2\_name, q2\_unit)
%   creates a temporary figure with two subplots: command signal (top) and
%   pendulum angle (bottom).  Prompts the scorer to confirm or type "repick".
%   Returns true if scorer confirms, false if scorer requests re-selection.
%   (SIGM-03)

    fig = figure('Name', 'Signal Preview - confirm or cancel', 'NumberTitle', 'off');

    ax1 = subplot(2, 1, 1);
    plot(ax1, hw_t, hw_cmd, 'b');
    title(ax1, sprintf('Command signal: %s', cmd_name));
    ylabel(ax1, 'Command');
    xlabel(ax1, 'Time [s]');
    grid(ax1, 'on');

    ax2 = subplot(2, 1, 2);
    [hw_q2_disp, q2_label] = convert_q2(hw_q2, q2_unit);
    plot(ax2, hw_t, hw_q2_disp, 'r');
    title(ax2, sprintf('Pendulum angle: %s', q2_name));
    ylabel(ax2, q2_label);
    xlabel(ax2, 'Time [s]');
    grid(ax2, 'on');

    linkaxes([ax1, ax2], 'x');
    drawnow;

    resp = input('Preview shown. Press Enter to confirm or type "repick" to re-select: ', 's');
    close(fig);

    confirmed = ~strcmpi(strtrim(resp), 'repick');
end

function aligned = align_signals(hw_t, hw_cmd, hw_q2, sim_t, sim_cmd, sim_q2, delta)
%ALIGN_SIGNALS Align hardware and simulation signals to t=0 at first nonzero command.
%   aligned = ALIGN_SIGNALS(hw\_t, hw\_cmd, hw\_q2, sim\_t, sim\_cmd, sim\_q2, delta)
%   detects the start time independently in each run as the first sample where
%   abs(cmd) > 0, shifts both time vectors to t=0, applies a manual delta to the
%   hardware time axis (delta > 0 left-shifts hw -- compensates for dead time),
%   crops both signals to their common time overlap, and defensively resamples the
%   simulation onto the hardware time grid if sample rates differ.
%
%   **Delta convention:** hw\_t\_aligned = hw\_t - hw\_t\_start - delta.
%   A positive delta means "the hardware response is delayed by this much,
%   shift it earlier" (subtract delta shifts the hw time axis to the left).
%
%   Returns a struct with fields:
%     aligned.t           -- common aligned time axis [s] (starts near 0)
%     aligned.hw\_q2       -- hardware pendulum angle [rad]
%     aligned.sim\_q2      -- simulation pendulum angle [rad]
%     aligned.hw\_cmd      -- hardware command signal
%     aligned.hw\_t\_start  -- raw time of first nonzero cmd in hw run [s]
%     aligned.sim\_t\_start -- raw time of first nonzero cmd in sim run [s]
%     aligned.delta       -- manual delta applied [s]

    % Step 1: Detect start time in hw run (D-04: first nonzero sample)
    hw_start_idx = find(abs(hw_cmd) > 0, 1, 'first');
    if isempty(hw_start_idx)
        warning('scorer:nostart', 'No nonzero command in hw run. Using t(1).');
        hw_start_idx = 1;
    end
    hw_t_start = hw_t(hw_start_idx);

    % Step 2: Detect start time in sim run independently (D-04)
    sim_start_idx = find(abs(sim_cmd) > 0, 1, 'first');
    if isempty(sim_start_idx)
        warning('scorer:nostart', 'No nonzero command in sim run. Using t(1).');
        sim_start_idx = 1;
    end
    sim_t_start = sim_t(sim_start_idx);

    % Step 3: Align time vectors to t=0 at start; apply manual delta to hw (D-05)
    % Positive delta means hw response is delayed -- subtract to left-shift hw
    hw_t_aligned  = hw_t  - hw_t_start  - delta;
    sim_t_aligned = sim_t - sim_t_start;

    % Step 4: Crop to common time overlap (D-07 -- no NaN padding, no extrapolation)
    t_start_common = max(hw_t_aligned(1),   sim_t_aligned(1));
    t_end_common   = min(hw_t_aligned(end), sim_t_aligned(end));

    hw_mask  = hw_t_aligned  >= t_start_common & hw_t_aligned  <= t_end_common;
    sim_mask = sim_t_aligned >= t_start_common & sim_t_aligned <= t_end_common;

    hw_t_crop   = hw_t_aligned(hw_mask);
    hw_q2_crop  = hw_q2(hw_mask);
    hw_cmd_crop = hw_cmd(hw_mask);

    sim_t_crop  = sim_t_aligned(sim_mask);
    sim_q2_crop = sim_q2(sim_mask);

    % Step 5: Defensive resample sim onto hw grid if sample rates differ (D-08)
    if numel(hw_t_crop) ~= numel(sim_t_crop) || ...
       max(abs(hw_t_crop - sim_t_crop)) > 1e-6
        sim_q2_crop = interp1(sim_t_crop, sim_q2_crop, hw_t_crop, 'linear');
        sim_t_crop  = hw_t_crop;  %#ok<NASGU>
    end

    % Step 6: Return aligned struct (D-12)
    aligned            = struct();
    aligned.t          = hw_t_crop;    % common aligned time axis [s]
    aligned.hw_q2      = hw_q2_crop;   % hw pendulum angle [rad]
    aligned.sim_q2     = sim_q2_crop;  % sim pendulum angle [rad]
    aligned.hw_cmd     = hw_cmd_crop;  % command signal
    aligned.hw_t_start = hw_t_start;   % raw time of first nonzero cmd in hw [s]
    aligned.sim_t_start = sim_t_start; % raw time of first nonzero cmd in sim [s]
    aligned.delta      = delta;         % manual delta applied [s]
end

%[text]
%[text] ---
%[text] **Local Functions: Overlay Figure for Attempt Browsing**
%[text] These functions create and manage the persistent overlay figure (OUTP-03).

%[text] **create\_overlay\_figure** — Creates a persistent figure window with a popupmenu
%[text] dropdown for browsing loaded attempts. Uses standard `figure()` (not `uifigure`)
%[text] so that `uicontrol` popupmenu works (D-10).

function fig = create_overlay_figure(q2_unit)
%CREATE_OVERLAY_FIGURE Create the persistent overlay figure with attempt dropdown.
%   fig = CREATE_OVERLAY_FIGURE(q2\_unit) creates a standard MATLAB figure window
%   with a uicontrol popupmenu at the top for browsing loaded attempts.  Attempt
%   data is stored in fig.UserData with 'attempts', 'labels', and 'q2\_unit'.
%   (OUTP-03, D-10)

    fig = figure('Name', 'Attempt Overlay', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 650]);
    fig.UserData = struct('attempts', {{}}, 'labels', {{}}, 'q2_unit', q2_unit);
    uicontrol(fig, 'Style', 'popupmenu', ...
              'Units', 'normalized', ...
              'Position', [0.1, 0.95, 0.8, 0.04], ...
              'String', {'(no attempts yet)'}, ...
              'Tag', 'attempt_dropdown', ...
              'Callback', @overlay_dropdown_callback);
end

%[text] **update\_overlay\_figure** — Appends a new attempt to the figure's UserData,
%[text] updates the dropdown list to include all loaded attempts, selects the newest
%[text] entry, and redraws the 3-subplot overlay for the new attempt.

function update_overlay_figure(fig, attempt, label)
%UPDATE_OVERLAY_FIGURE Append an attempt to the overlay figure and redraw subplots.
%   UPDATE_OVERLAY_FIGURE(fig, attempt, label) appends attempt to fig.UserData,
%   updates the dropdown list, selects the newest entry, and calls
%   draw\_attempt\_subplots to render the 3-subplot overlay for this attempt.
%   (OUTP-03, D-10)

    ud = fig.UserData;
    ud.attempts{end+1} = attempt;
    ud.labels{end+1}   = label;
    fig.UserData = ud;
    dd = findobj(fig, 'Tag', 'attempt_dropdown');
    set(dd, 'String', ud.labels, 'Value', numel(ud.labels));
    draw_attempt_subplots(fig, attempt);
end

%[text] **draw\_attempt\_subplots** — Draws the 3-subplot overlay for a given attempt:
%[text] (1) hw q2 vs sim q2 aligned, (2) command signal, (3) q2 difference.
%[text] Uses `delete(findobj(...,'Type','axes'))` instead of `clf` to preserve
%[text] the uicontrol dropdown (anti-pattern from Research: clf destroys uicontrol).

function draw_attempt_subplots(fig, attempt)
%DRAW_ATTEMPT_SUBPLOTS Render 3-subplot overlay for one attempt on the overlay figure.
%   DRAW_ATTEMPT_SUBPLOTS(fig, attempt) brings the figure to front, deletes only
%   axes (NOT the uicontrol dropdown), then draws three linked subplots:
%     Subplot 1 (top)    -- hw q2 vs sim q2 overlay with t=0 annotation
%     Subplot 2 (middle) -- accel/torque command signal with t=0 annotation
%     Subplot 3 (bottom) -- q2 difference (hw - sim) with t=0 annotation
%   q2 display unit is read from fig.UserData.q2_unit ('rev', 'deg', or 'rad').
%   X-axes are linked for synchronized zoom.  (D-09, D-11)

    figure(fig);

    % Delete only axes -- never use clf (would destroy the uicontrol dropdown)
    delete(findobj(fig, 'Type', 'axes'));

    % Extract aligned data and convert q2 to display unit
    t   = attempt.aligned.t;
    q2_unit = fig.UserData.q2_unit;
    [hw, q2_label]  = convert_q2(attempt.aligned.hw_q2, q2_unit);
    [sim, ~]        = convert_q2(attempt.aligned.sim_q2, q2_unit);
    cmd = attempt.aligned.hw_cmd;

    % Subplot 1 (top): hw q2 vs sim q2 overlay (D-09)
    ax1 = subplot(3, 1, 1, 'Parent', fig);
    plot(ax1, t, hw,  'b-',  'DisplayName', 'Hardware q2');
    hold(ax1, 'on');
    plot(ax1, t, sim, 'r--', 'DisplayName', 'Simulation q2');
    hold(ax1, 'off');
    ylabel(ax1, q2_label);
    title(ax1, 'Hardware q2 vs Simulation q2 (aligned)');
    legend(ax1, 'show');
    grid(ax1, 'on');
    % t=0 annotation (D-11): include delta if nonzero
    if attempt.aligned.delta ~= 0
        xline(ax1, 0, 'k--', sprintf('t=0 (delta=%.3fs)', attempt.aligned.delta));
    else
        xline(ax1, 0, 'k--', 't=0');
    end

    % Subplot 2 (middle): command signal (D-09)
    ax2 = subplot(3, 1, 2, 'Parent', fig);
    plot(ax2, t, cmd, 'm');
    ylabel(ax2, 'Command');
    title(ax2, 'Accel / Torque Command');
    grid(ax2, 'on');
    xline(ax2, 0, 'k--');

    % Subplot 3 (bottom): q2 difference hw - sim (D-09)
    ax3 = subplot(3, 1, 3, 'Parent', fig);
    plot(ax3, t, hw - sim, 'g');
    diff_label = strrep(q2_label, 'q2', 'hw-sim');
    ylabel(ax3, diff_label);
    xlabel(ax3, 'Aligned time [s]');
    title(ax3, 'q2 Difference (hw - sim)');
    grid(ax3, 'on');
    xline(ax3, 0, 'k--');

    % Link x-axes for synchronized zoom (D-09)
    linkaxes([ax1, ax2, ax3], 'x');
    drawnow;
end

%[text] **overlay\_dropdown\_callback** — Callback for the attempt dropdown.  When
%[text] the scorer selects a different attempt from the dropdown, this redraws the
%[text] 3-subplot overlay for the selected attempt.  Uses `ancestor` to get the
%[text] figure handle (not `gcf`), and `get(src,'Value')` for the 1-based index.

function overlay_dropdown_callback(src, ~)
%OVERLAY_DROPDOWN_CALLBACK Redraw overlay subplots for the selected attempt.
%   OVERLAY_DROPDOWN_CALLBACK(src, ~) reads the dropdown selection index,
%   retrieves the corresponding attempt from fig.UserData, and redraws the
%   3-subplot overlay via draw\_attempt\_subplots.  (D-10)

    fig = ancestor(src, 'figure');
    ud  = fig.UserData;
    idx = get(src, 'Value');
    if idx <= numel(ud.attempts)
        draw_attempt_subplots(fig, ud.attempts{idx});
    end
end

%[appendix]{"version":"1.0"}
