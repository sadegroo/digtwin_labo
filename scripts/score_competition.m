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
cfg.teams(1) = struct('name', 'Team Lambda', 'type', 'bldc');
cfg.teams(2) = struct('name', 'Team Rho',    'type', 'stepper');
cfg.teams(3) = struct('name', 'Team Theta',  'type', 'stepper');
cfg.teams(4) = struct('name', 'Team Pi',     'type', 'stepper');
cfg.teams(5) = struct('name', 'Team Omega',  'type', 'stepper');

cfg.smape_fixed_duration   = 5;         % seconds — D-09 hybrid window: max(this, t_to_first_90deg)
cfg.swingup_hold_time      = 1.0;       % seconds pendulum must hold at +/-pi (Phase 3)
cfg.swingup_tolerance_deg  = 2;         % degrees tolerance around +/-pi (Phase 3)
cfg.participation_threshold = pi/2;     % rad -- |q2| must exceed this (Phase 3)
cfg.truncation_margin = 1;  % seconds after q2 reaches upright to truncate signals
cfg.cmd_keywords = {'accel', 'torque', 'cmd', 'tau', 'ref', 'input'};
cfg.q2_keywords  = {'q2', 'theta', 'pend', 'angle', 'phi', 'joint2'};

%[text] **Scoring parameters** (Phase 4)
cfg.time_points       = [2, 1, 0.5, 0];        % stepper time rank: 1st..4th (D-05)
cfg.smape_points      = [2, 1, 0.5, 0.5];      % stepper SMAPE rank: 1st..4th (D-06)
cfg.bldc_smape_bands  = [40, 80, 120, 160];     % upper-exclusive SMAPE% band edges (D-07)
cfg.bldc_smape_pts    = [4, 3, 2, 1, 0];        % points per band (D-07)
cfg.export_dir        = 'data';                  % output folder (D-13)
cfg.export_stem       = 'competition_results';   % filename base (D-13)

% Prompt for q2 display unit
q2_units = {'rev', 'deg', 'rad'};
[sel, ok] = listdlg('ListString', q2_units, ...
    'SelectionMode', 'single', ...
    'Name', 'q2 Unit', ...
    'PromptString', 'Select q2 display unit:', ...
    'InitialValue', 1, ...
    'ListSize', [200, 100]);
if ok
    cfg.q2_unit = q2_units{sel};
else
    cfg.q2_unit = 'rev';  % default if cancelled
end

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
overlay_fig = create_overlay_figure();

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

    %% Select q2 unit for this file
    q2_units = {'rev', 'deg', 'rad'};
    cur_idx = find(strcmp(q2_units, cfg.q2_unit), 1);
    if isempty(cur_idx), cur_idx = 1; end
    [usel, uok] = listdlg('ListString', q2_units, ...
        'SelectionMode', 'single', ...
        'Name', 'q2 Unit', ...
        'PromptString', sprintf('q2 unit for "%s":', fname), ...
        'InitialValue', cur_idx, ...
        'ListSize', [200, 100]);
    if uok
        file_q2_unit = q2_units{usel};
        cfg.q2_unit = file_q2_unit;  % update default for next file
    else
        file_q2_unit = cfg.q2_unit;  % keep current default
    end

    %% Signal selection with repick loop (SIGM-01, SIGM-02, SIGM-03, D-01, D-02, D-03)
    hw_run  = Simulink.sdi.getRun(attempt.hw_run_id);
    sim_run = Simulink.sdi.getRun(attempt.sim_run_id);

    signal_ok = false;
    while true
        [cmd_name, q2_name] = select_signals(hw_run, team, cfg.cmd_keywords, cfg.q2_keywords);
        if isempty(cmd_name)
            fprintf('Signal selection cancelled.\n');
            break   % exit inner loop; signal_ok remains false
        end

        % Extract hw signals
        [hw_t_cmd, hw_cmd_data] = extract_signal(hw_run, cmd_name);
        [hw_t_q2,  hw_q2_data]  = extract_signal(hw_run, q2_name);
        if isempty(hw_t_cmd) || isempty(hw_t_q2)
            warning('scorer:extractfail', 'Failed to extract selected signals. Re-pick.');
            continue
        end

        % Defensive resample: cmd and q2 may have different time bases
        if ~isequal(hw_t_cmd, hw_t_q2)
            hw_q2_data = interp1(hw_t_q2, hw_q2_data, hw_t_cmd, 'linear');
        end

        % Extract sim signals (same names; fallback if not found)
        [sim_t_cmd, sim_cmd_data] = extract_signal(sim_run, cmd_name);
        [sim_t_q2,  sim_q2_data]  = extract_signal(sim_run, q2_name);

        if isempty(sim_t_cmd)
            fprintf('Signal "%s" not found in sim run. Pick manually.\n', cmd_name);
            [sim_cmd_name, ~] = select_signals(sim_run, struct(), cfg.cmd_keywords, cfg.q2_keywords);
            if ~isempty(sim_cmd_name)
                [sim_t_cmd, sim_cmd_data] = extract_signal(sim_run, sim_cmd_name);
            end
        end
        if isempty(sim_t_q2)
            fprintf('Signal "%s" not found in sim run. Pick manually.\n', q2_name);
            [~, sim_q2_name] = select_signals(sim_run, struct(), cfg.cmd_keywords, cfg.q2_keywords);
            if ~isempty(sim_q2_name)
                [sim_t_q2, sim_q2_data] = extract_signal(sim_run, sim_q2_name);
            end
        end

        % Defensive resample for sim signals
        if ~isempty(sim_t_cmd) && ~isempty(sim_t_q2) && ~isequal(sim_t_cmd, sim_t_q2)
            sim_q2_data = interp1(sim_t_q2, sim_q2_data, sim_t_cmd, 'linear');
        end

        % Preview: overlay hw + sim q2 and command, truncated at swing-up
        confirmed = plot_preview(hw_t_cmd, hw_cmd_data, hw_q2_data, ...
                                 sim_t_cmd, sim_cmd_data, sim_q2_data, ...
                                 cmd_name, q2_name, file_q2_unit);
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

    %% Compute metrics on pre-truncation data (METR-01..07, D-06/D-07)
    attempt.metrics = compute_metrics(aligned, cfg, file_q2_unit);

    %% Truncate signals 2s after swing-up (upright position)
    aligned = truncate_at_swingup(aligned, file_q2_unit, cfg.truncation_margin);

    %% Store aligned data and signal metadata in attempt struct (D-12)
    attempt.signals.cmd_name = cmd_name;
    attempt.signals.q2_name  = q2_name;
    attempt.signals.delta_s  = delta;
    attempt.signals.q2_unit  = file_q2_unit;
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

    %% Continue, undo, or finalize
    cmd = input('Load another file? [Enter] continue, ''undo'' remove last, ''done'' finalize: ', 's');
    cmd = lower(strtrim(cmd));
    if strcmp(cmd, 'done')
        break
    elseif strcmp(cmd, 'undo')
        [session, overlay_fig] = remove_last_attempt(session, team_idx, overlay_fig);
        continue
    end
end

%% Finalization
%[text] Session complete. Apply scoring rubric, print diagnostics, display leaderboard.

session.finalized = true;
fprintf('\n========================================\n');
fprintf('SESSION FINALIZED\n');
fprintf('========================================\n');

%[text] **Step 1:** Per-team diagnostic summary (D-15, OUTP-04)
try
    print_diagnostics(session, cfg);
catch e
    fprintf(2, 'Step 1 error (diagnostics): %s\n', e.message);
end

%[text] **Step 2:** Compute leaderboard table (SCOR-01..06, OUTP-01)
try
    T = compute_leaderboard(session, cfg);
catch e
    fprintf(2, 'Step 2 error (leaderboard): %s\n', e.message);
    T = table();
end

%[text] **Step 3:** Display leaderboard (D-10, D-11)
try
    disp_leaderboard(T, session, cfg);
catch e
    fprintf(2, 'Step 3 error (display): %s\n', e.message);
end

%[text] **Step 4:** Export results to CSV and xlsx (D-13, OUTP-02)
try
    export_results(T, cfg, session);
catch e
    fprintf(2, 'Step 4 error (export): %s\n', e.message);
end

%[text] **Step 5:** Score breakdown bar chart (D-12)
try
    plot_score_breakdown(T, cfg);
catch e
    fprintf(2, 'Step 5 error (plot): %s\n', e.message);
end

fprintf('\nSession state is in workspace variable "session".\n');
fprintf('Leaderboard table is in workspace variable "T".\n');

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

%[text] **q2\_label** — Returns the axis label string for the configured q2 unit.
%[text] No conversion is applied; `cfg.q2\_unit` declares what unit the raw SDI data is in.
function label = q2_label(unit)
    switch unit
        case 'rev', label = 'q2 [rev]';
        case 'deg', label = 'q2 [deg]';
        case 'rad', label = 'q2 [rad]';
        otherwise,  label = 'q2 [rev]';
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

function confirmed = plot_preview(hw_t, hw_cmd, hw_q2, sim_t, sim_cmd, sim_q2, cmd_name, q2_name, q2_unit)
%PLOT_PREVIEW Show a 3-subplot preview overlaying hw and sim signals.
%   Displays: (top) hw+sim q2 overlay, (mid) hw command, (bottom) sim command.
%   Signals are truncated at the first sample where |q2| reaches the upright
%   position (±pi rad / ±180 deg / ±0.5 rev) to keep command axis scale
%   reasonable during the swing-up phase.  (SIGM-03)

    % Determine upright threshold in data units
    switch q2_unit
        case 'rev', threshold = 0.5;
        case 'deg', threshold = 180;
        case 'rad', threshold = pi;
        otherwise,  threshold = 0.5;
    end

    % Truncate hw at first upright crossing
    hw_trunc = find(abs(hw_q2) >= threshold, 1, 'first');
    if isempty(hw_trunc), hw_trunc = numel(hw_t); end
    % Truncate sim at first upright crossing
    sim_trunc = numel(sim_t);
    if ~isempty(sim_q2)
        idx = find(abs(sim_q2) >= threshold, 1, 'first');
        if ~isempty(idx), sim_trunc = idx; end
    end

    fig = figure('Name', 'Signal Preview - confirm or cancel', ...
                 'NumberTitle', 'off', 'Position', [100, 100, 900, 650]);

    % Subplot 1: hw + sim q2 overlay (full length for offset assessment)
    ax1 = subplot(3, 1, 1);
    plot(ax1, hw_t, hw_q2, 'b-', 'DisplayName', 'HW q2');
    hold(ax1, 'on');
    if ~isempty(sim_t)
        plot(ax1, sim_t, sim_q2, 'r--', 'DisplayName', 'Sim q2');
    end
    hold(ax1, 'off');
    title(ax1, sprintf('q2 overlay: %s  (use to judge time offset)', q2_name));
    ylabel(ax1, q2_label(q2_unit));
    legend(ax1, 'show');
    grid(ax1, 'on');

    % Subplot 2: hw command (truncated at swing-up)
    ax2 = subplot(3, 1, 2);
    plot(ax2, hw_t(1:hw_trunc), hw_cmd(1:hw_trunc), 'b');
    title(ax2, sprintf('HW command (truncated at swing-up): %s', cmd_name));
    ylabel(ax2, 'Command');
    grid(ax2, 'on');

    % Subplot 3: sim command (truncated at swing-up)
    ax3 = subplot(3, 1, 3);
    if ~isempty(sim_t)
        plot(ax3, sim_t(1:sim_trunc), sim_cmd(1:sim_trunc), 'r');
    end
    title(ax3, sprintf('Sim command (truncated at swing-up): %s', cmd_name));
    ylabel(ax3, 'Command');
    xlabel(ax3, 'Time [s]');
    grid(ax3, 'on');

    linkaxes([ax2, ax3], 'xy');   % sync command subplot scales
    figure(fig);
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

function fig = create_overlay_figure()
%CREATE_OVERLAY_FIGURE Create the persistent overlay figure with attempt dropdown.
%   fig = CREATE_OVERLAY_FIGURE() creates a standard MATLAB figure window with a
%   uicontrol popupmenu at the top for browsing loaded attempts.  Attempt data is
%   stored in fig.UserData with 'attempts' and 'labels' cell arrays.
%   (OUTP-03, D-10)

    fig = figure('Name', 'Attempt Overlay', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 900, 650]);
    fig.UserData = struct('attempts', {{}}, 'labels', {{}});
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
%   q2 display unit is read from attempt.signals.q2\_unit (per-file unit).
%   X-axes are linked for synchronized zoom.  (D-09, D-11)

    figure(fig);

    % Delete only axes -- never use clf (would destroy the uicontrol dropdown)
    delete(findobj(fig, 'Type', 'axes'));

    % Extract aligned data; unit is per-attempt
    t   = attempt.aligned.t;
    q2_unit = attempt.signals.q2_unit;
    hw  = attempt.aligned.hw_q2;
    sim = attempt.aligned.sim_q2;
    cmd = attempt.aligned.hw_cmd;

    % Subplot 1 (top): hw q2 vs sim q2 overlay (D-09)
    ax1 = subplot(3, 1, 1, 'Parent', fig);
    plot(ax1, t, hw,  'b-',  'DisplayName', 'Hardware q2');
    hold(ax1, 'on');
    plot(ax1, t, sim, 'r--', 'DisplayName', 'Simulation q2');
    hold(ax1, 'off');
    ylabel(ax1, q2_label(q2_unit));
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
    ylabel(ax3, strrep(q2_label(q2_unit), 'q2', 'hw-sim'));
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

%[text] **remove\_last\_attempt** — Removes the most recent attempt from a team and
%[text] updates the overlay figure dropdown accordingly.

function [session, fig] = remove_last_attempt(session, team_idx, fig)
%REMOVE_LAST_ATTEMPT Undo the last attempt added for a given team.
%   Removes the last attempt from session.teams(team\_idx).attempts and syncs
%   the overlay figure's UserData and dropdown.

    n = numel(session.teams(team_idx).attempts);
    if n == 0
        fprintf('No attempts to remove for team "%s".\n', session.teams(team_idx).name);
        return
    end

    session.teams(team_idx).attempts(end) = [];
    fprintf('Removed attempt %d from team "%s".\n', n, session.teams(team_idx).name);

    % Sync overlay figure
    ud = fig.UserData;
    if ~isempty(ud.attempts)
        ud.attempts(end) = [];
        ud.labels(end)   = [];
        fig.UserData = ud;

        dd = findobj(fig, 'Tag', 'attempt_dropdown');
        if isempty(ud.labels)
            set(dd, 'String', {'(no attempts yet)'}, 'Value', 1);
            delete(findobj(fig, 'Type', 'axes'));
        else
            set(dd, 'String', ud.labels, 'Value', numel(ud.labels));
            draw_attempt_subplots(fig, ud.attempts{end});
        end
    end

    % Print updated progress
    fprintf('\n--- Session Progress ---\n');
    for i = 1:numel(session.teams)
        fprintf('  %s: %d attempt(s)\n', session.teams(i).name, numel(session.teams(i).attempts));
    end
    fprintf('\n');
end

%[text] **truncate\_at\_swingup** — Truncates all aligned signals at a configurable margin
%[text] after q2 first reaches the upright position (±0.5 rev / ±180 deg / ±$\pi$ rad).
%[text] This removes post-stabilization data where controller failures can dominate the
%[text] command scale.

function aligned = truncate_at_swingup(aligned, q2_unit, margin_s)
%TRUNCATE_AT_SWINGUP Truncate aligned signals after swing-up + margin.
%   aligned = TRUNCATE_AT_SWINGUP(aligned, q2\_unit, margin\_s) finds the first
%   sample where either |hw\_q2| or |sim\_q2| reaches the upright threshold, then
%   keeps data up to margin\_s seconds after that point.

    % Upright threshold in data units
    switch q2_unit
        case 'rev', threshold = 0.5;
        case 'deg', threshold = 180;
        case 'rad', threshold = pi;
        otherwise,  threshold = 0.5;
    end

    t = aligned.t;

    % Find first upright crossing in hw and sim
    hw_idx  = find(abs(aligned.hw_q2) >= threshold, 1, 'first');
    sim_idx = find(abs(aligned.sim_q2) >= threshold, 1, 'first');

    % Take earliest crossing; if neither reaches upright, keep everything
    cross_idx = min([hw_idx, sim_idx]);
    if isempty(cross_idx)
        return
    end

    % Truncation point: crossing time + margin
    t_cut = t(cross_idx) + margin_s;
    cut_idx = find(t >= t_cut, 1, 'first');
    if isempty(cut_idx) || cut_idx >= numel(t)
        return  % margin extends beyond data — keep everything
    end

    % Truncate all fields
    aligned.t      = t(1:cut_idx);
    aligned.hw_q2  = aligned.hw_q2(1:cut_idx);
    aligned.sim_q2 = aligned.sim_q2(1:cut_idx);
    aligned.hw_cmd = aligned.hw_cmd(1:cut_idx);
end

%[text]
%[text] ---
%[text] **Local Functions: Metric Computation**
%[text] These functions compute per-attempt metrics (swingup success, swingup time,
%[text] participation, and angular SMAPE) on pre-truncation aligned data (Phase 3).

function idx = first_sustained_idx(mask, N)
%FIRST_SUSTAINED_IDX Find start index of first window of N consecutive TRUE values.
%   idx = FIRST\_SUSTAINED\_IDX(mask, N) returns the start index of the first
%   contiguous block of length N where mask is TRUE.  Returns [] if no such
%   window exists.  Uses conv(..., 'valid') for vectorized sliding-window check.

    if N <= 1
        idx = find(mask, 1, 'first');
        return
    end
    counts  = conv(double(mask), ones(1, N), 'valid');
    win_idx = find(counts >= N, 1, 'first');
    idx     = win_idx;  % [] if not found; conv 'valid' preserves 1-based indexing
end

function smape = compute_smape_angular(hw_rad, sim_rad, epsilon)
%COMPUTE_SMAPE_ANGULAR Angular SMAPE with wrapping and denominator guard.
%   smape = COMPUTE\_SMAPE\_ANGULAR(hw\_rad, sim\_rad, epsilon) computes SMAPE
%   using the angular difference formula mod(hw-sim+pi, 2\*pi)-pi for the
%   numerator (handles wrapping near +/-pi per METR-07/D-12).  Samples where
%   the denominator (|hw|+|sim|)/2 < epsilon are excluded (METR-06/D-13).
%   Returns a percentage value, or NaN if no valid samples exist.

    % Angular difference numerator (METR-07, D-12)
    num   = abs(mod(hw_rad - sim_rad + pi, 2*pi) - pi);

    % Symmetric denominator
    denom = (abs(hw_rad) + abs(sim_rad)) / 2;

    % Denominator guard (METR-06, D-13)
    valid = denom >= epsilon;
    if ~any(valid)
        smape = NaN;
        return
    end

    smape = mean(num(valid) ./ denom(valid)) * 100;  % percent
end

function out = yesno(flag)
%YESNO Convert logical flag to 'YES'/'NO' string.
    if flag, out = 'YES'; else, out = 'NO'; end
end

%[text] **compute\_metrics** — Computes all per-attempt metrics from pre-truncation
%[text] aligned signals: participation flag, swingup success and time, and angular
%[text] SMAPE with the D-09 hybrid window policy (max(5s, time\_to\_first\_90deg)).
%[text] Unit conversion to radians is the first step (D-01/D-02). All thresholds
%[text] operate in radians regardless of the per-file display unit.

function metrics = compute_metrics(aligned, cfg, q2_unit)
%COMPUTE_METRICS Compute swingup metrics from pre-truncation aligned signals.
%   metrics = COMPUTE\_METRICS(aligned, cfg, q2\_unit) converts q2 signals to
%   radians (D-01/D-02), then computes:
%     - participation flag (METR-02, D-14): |q2| > pi/2 for >= 10 consecutive
%       samples on the hardware signal
%     - swingup success flag (METR-01, D-03/D-05): |abs(q2)-pi| < 2deg for
%       >= 1 continuous second; both +pi and -pi count as upright
%     - swingup time (METR-03, D-04): backdated to hold entry of first
%       qualifying 1-second window
%     - SMAPE with angular difference and D-09 hybrid window policy
%       (METR-04/05/06/07): max(5s, time\_to\_first\_90deg); ineligible if
%       |q2| never exceeds pi/2 (D-10)
%   Returns a flat metrics struct with fields: swingup\_success, swingup\_time,
%   participation, smape\_eligible, smape, smape\_window\_s.

    % --- D-01/D-02: Convert to radians at function entry ----------------
    % The aligned struct is NOT modified; all metric computation uses local
    % radian copies. Caller uses aligned struct in its native unit for plots.
    switch q2_unit
        case 'rev', scale = 2 * pi;
        case 'deg', scale = pi / 180;
        case 'rad', scale = 1;
        otherwise,  scale = 2 * pi;  % default: treat unknown unit as rev
    end
    hw_q2_rad  = aligned.hw_q2  * scale;
    sim_q2_rad = aligned.sim_q2 * scale;
    t          = aligned.t;

    % Derived constants
    tol_rad = cfg.swingup_tolerance_deg * pi / 180;   % 2 deg = 0.0349 rad
    Ts_est  = median(diff(t));                         % actual sample period [s]
    N_hold  = round(cfg.swingup_hold_time / Ts_est);  % samples for 1-second hold
    N_part  = 10;          % consecutive-sample filter for participation (METR-02)
    epsilon = 1e-3;        % denominator guard ~3x sensor noise floor [rad] (D-13)

    % --- METR-02 / D-14: Participation ----------------------------------
    % Noise-robust check: |q2| > pi/2 for at least 10 consecutive samples.
    part_mask = abs(hw_q2_rad) > cfg.participation_threshold;
    part_idx  = first_sustained_idx(part_mask, N_part);
    metrics.participation = ~isempty(part_idx);

    % --- METR-01 / D-03/D-04/D-05: Swingup success and time ------------
    % Both +pi and -pi count as upright (abs(q2) handles sign, D-05).
    % Time is backdated to the ENTRY of the first qualifying 1-second window
    % (D-04), not just the first crossing.
    band_mask = abs(abs(hw_q2_rad) - pi) < tol_rad;
    entry_idx = first_sustained_idx(band_mask, N_hold);
    metrics.swingup_success = ~isempty(entry_idx);
    if metrics.swingup_success
        metrics.swingup_time = t(entry_idx);  % D-04: entry time of sustained hold
    else
        metrics.swingup_time = NaN;
    end

    % --- METR-04/05/06/07 + D-09/D-10/D-11: SMAPE ----------------------
    % D-09 hybrid window: max(5s, time_to_first_90deg).
    % D-10: never reached pi/2 -> ineligible, skip SMAPE entirely.
    idx_90 = find(abs(hw_q2_rad) > pi/2, 1, 'first');

    if isempty(idx_90)
        % D-10: |q2| never reached pi/2 -- attempt ineligible for SMAPE
        metrics.smape_eligible = false;
        metrics.smape          = NaN;
        metrics.smape_window_s = NaN;
    else
        % D-09: extend window beyond 5s if first 90deg crossing is later
        t_win_end   = max(cfg.smape_fixed_duration, t(idx_90));
        win_end_idx = find(t >= t_win_end, 1, 'first');
        if isempty(win_end_idx)
            win_end_idx = numel(t);  % data shorter than window -- use all
        end
        hw_win  = hw_q2_rad(1:win_end_idx);
        sim_win = sim_q2_rad(1:win_end_idx);
        metrics.smape_eligible = true;
        metrics.smape          = compute_smape_angular(hw_win, sim_win, epsilon);
        metrics.smape_window_s = t(win_end_idx);
    end

    % --- D-08: Compact one-liner diagnostic after each attempt ----------
    if metrics.swingup_success
        t_str = sprintf('t=%.2fs', metrics.swingup_time);
    else
        t_str = 't=N/A';
    end
    if metrics.smape_eligible && ~isnan(metrics.smape)
        s_str = sprintf('SMAPE=%.1f%%', metrics.smape);
    elseif metrics.smape_eligible
        s_str = 'SMAPE=NaN';
    else
        s_str = 'SMAPE=N/A';
    end
    fprintf('Metrics: swingup=%s %s %s participation=%s\n', ...
        yesno(metrics.swingup_success), t_str, s_str, ...
        yesno(metrics.participation));
end

%[text]
%[text] ---
%[text] **Local Functions: Scoring and Leaderboard (Phase 4)**
%[text] These functions implement the competition scoring rubric: best-per-metric
%[text] aggregation, stepper competitive ranking (sports convention), BLDC absolute
%[text] band scoring, and leaderboard table construction.

function [best_time, best_smape, has_participation] = aggregate_best(team)
%AGGREGATE_BEST Select best-per-metric values across all attempts for a team.
%   [best\_time, best\_smape, has\_participation] = AGGREGATE\_BEST(team) iterates
%   over team.attempts, extracts the fastest successful swingup time and the
%   lowest SMAPE-eligible SMAPE value. Returns NaN for metrics with no qualifying
%   attempt. has\_participation is true if any attempt has participation == true.
%   Implements D-01, D-02, D-03 (SCOR-01).

    n = numel(team.attempts);
    times  = NaN(n, 1);
    smapes = NaN(n, 1);
    part_flag = false;

    for j = 1:n
        m = team.attempts{j}.metrics;
        if isfield(m, 'swingup_success') && m.swingup_success
            times(j) = m.swingup_time;
        end
        if isfield(m, 'smape_eligible') && m.smape_eligible && ~isnan(m.smape)
            smapes(j) = m.smape;
        end
        if isfield(m, 'participation') && m.participation
            part_flag = true;
        end
    end

    best_time  = min(times,  [], 'omitnan');
    best_smape = min(smapes, [], 'omitnan');

    % Guard: empty or all-NaN arrays return [] from min -- normalize to NaN
    if isempty(best_time)  || all(isnan(times)),  best_time  = NaN; end
    if isempty(best_smape) || all(isnan(smapes)), best_smape = NaN; end

    has_participation = part_flag;
end

function pts = assign_points_dense(values, point_table)
%ASSIGN_POINTS_DENSE Assign points to competitors using sports-convention dense ranking.
%   pts = ASSIGN\_POINTS\_DENSE(values, point\_table) assigns points to each
%   element of values (lower = better; NaN = did not qualify). Tied competitors
%   share the same rank and same (higher) point value. Dense ranking: ranks
%   increment by 1 after each distinct value, no gaps (D-04).
%   point\_table maps rank 1,2,3,... to point values; rank exceeding the table
%   length receives 0. (SCOR-02, SCOR-03, D-05, D-06)

    N   = numel(values);
    pts = zeros(N, 1);

    valid = ~isnan(values);
    if ~any(valid)
        return
    end

    v              = values(valid);
    [sorted_v, order] = sort(v, 'ascend');

    % Build dense ranks (ties share same rank, no gaps after tie)
    ranks = ones(numel(v), 1);
    for k = 2:numel(sorted_v)
        if sorted_v(k) == sorted_v(k-1)
            ranks(k) = ranks(k-1);   % tie: same rank (D-04)
        else
            ranks(k) = ranks(k-1) + 1;  % dense increment
        end
    end

    % Map ranks to points
    rank_pts = zeros(numel(v), 1);
    for k = 1:numel(v)
        r = ranks(k);
        if r <= numel(point_table)
            rank_pts(k) = point_table(r);
        else
            rank_pts(k) = 0;
        end
    end

    % Unsort: write back in original order
    pts_valid         = zeros(numel(v), 1);
    pts_valid(order)  = rank_pts;
    pts(valid)        = pts_valid;
end

function pts = score_bldc_smape(smape_pct, bands, band_pts)
%SCORE_BLDC_SMAPE Absolute band scoring for the BLDC team SMAPE.
%   pts = SCORE\_BLDC\_SMAPE(smape\_pct, bands, band\_pts) returns the point
%   value for the given SMAPE percentage using upper-exclusive band boundaries.
%   bands = [40, 80, 120, 160]; band\_pts = [4, 3, 2, 1, 0].
%   If smape\_pct < bands(k), return band\_pts(k) (upper-exclusive, D-07).
%   If smape\_pct is NaN or >= all bands, return band\_pts(end).

    if isnan(smape_pct)
        pts = 0;
        return
    end

    pts = band_pts(end);   % default: 160+% case
    for k = 1:numel(bands)
        if smape_pct < bands(k)
            pts = band_pts(k);
            return
        end
    end
end

function T = compute_leaderboard(session, cfg)
%COMPUTE_LEADERBOARD Compute full scoring table for all teams.
%   T = COMPUTE\_LEADERBOARD(session, cfg) aggregates best-per-metric values
%   for each team, assigns stepper competitive points (dense ranking), assigns
%   BLDC absolute band points, adds participation points, computes totals and
%   stepper ranks. Returns an 8-column MATLAB table. (SCOR-01..06, OUTP-01,
%   D-01..D-09)

    N = numel(session.teams);

    % Preallocate
    team_names  = {cfg.teams.name}';
    team_types  = {cfg.teams.type}';
    best_times  = NaN(N, 1);
    best_smapes = NaN(N, 1);
    part_flags  = false(N, 1);
    time_pts    = zeros(N, 1);
    smape_pts   = zeros(N, 1);
    part_pts    = zeros(N, 1);

    % Aggregate best values for each team
    for i = 1:N
        [best_times(i), best_smapes(i), part_flags(i)] = ...
            aggregate_best(session.teams(i));
        part_pts(i) = double(part_flags(i));  % 1pt if any participation (SCOR-05, D-09)
    end

    % Stepper competitive ranking (SCOR-02, SCOR-03)
    stepper_mask = strcmp(team_types, 'stepper');
    time_pts(stepper_mask)  = assign_points_dense( ...
        best_times(stepper_mask),  cfg.time_points);
    smape_pts(stepper_mask) = assign_points_dense( ...
        best_smapes(stepper_mask), cfg.smape_points);

    % BLDC absolute band scoring (SCOR-04, D-07)
    bldc_mask = strcmp(team_types, 'bldc');
    for i = find(bldc_mask)'
        smape_pts(i) = score_bldc_smape( ...
            best_smapes(i), cfg.bldc_smape_bands, cfg.bldc_smape_pts);
    end

    % Total points (SCOR-06)
    total_pts = time_pts + smape_pts + part_pts;

    % Stepper ranks on total (descending); BLDC rank stays NaN (D-08)
    ranks = NaN(N, 1);
    stepper_totals = total_pts(stepper_mask);
    [sorted_tot, ord] = sort(stepper_totals, 'descend');
    stepper_ranks = ones(sum(stepper_mask), 1);
    for k = 2:numel(sorted_tot)
        if sorted_tot(k) == sorted_tot(k-1)
            stepper_ranks(k) = stepper_ranks(k-1);
        else
            stepper_ranks(k) = stepper_ranks(k-1) + 1;
        end
    end
    rank_unsorted             = zeros(sum(stepper_mask), 1);
    rank_unsorted(ord)        = stepper_ranks;
    ranks(stepper_mask)       = rank_unsorted;

    % Build output table (OUTP-01)
    T = table(string(team_names), best_times, best_smapes, ...
              time_pts, smape_pts, part_pts, total_pts, ranks, ...
              'VariableNames', {'Team', 'BestSwingupTime', 'BestSMAPE', ...
                                'TimePoints', 'SMAPEPoints', ...
                                'ParticipationPoint', 'TotalPoints', 'Rank'});
end

%[text] **print\_diagnostics** -- Prints per-team summary to the command window before
%[text] the leaderboard: N attempts loaded, best swingup time, best SMAPE, and
%[text] participation status (OUTP-04, D-15).

function print_diagnostics(session, cfg)
%PRINT_DIAGNOSTICS Print per-team summary before the leaderboard.
%   PRINT\_DIAGNOSTICS(session, cfg) iterates over all teams and prints
%   the number of attempts, best swingup time, best SMAPE, and participation
%   status to the command window. Reuses aggregate\_best. (OUTP-04, D-15)

    fprintf('\n--- Per-Team Diagnostics ---\n');
    for i = 1:numel(session.teams)
        t     = session.teams(i);
        n_att = numel(t.attempts);
        [bt, bs, has_p] = aggregate_best(t);

        % Format best time
        if isnan(bt)
            time_str = 'N/A';
        else
            time_str = sprintf('%.2f s', bt);
        end

        % Format best SMAPE
        if isnan(bs)
            smape_str = 'N/A';
        else
            smape_str = sprintf('%.1f%%', bs);
        end

        fprintf('  %s (%s): %d attempt(s)\n', t.name, t.type, n_att);
        fprintf('    Best swingup time : %s\n', time_str);
        fprintf('    Best SMAPE        : %s\n', smape_str);
        fprintf('    Participation     : %s\n', yesno(has_p));
    end
    fprintf('\n');
end

%[text] **disp\_leaderboard** -- Displays the stepper leaderboard table (D-10) and a
%[text] separate BLDC summary section below (D-11).

function disp_leaderboard(T, session, cfg)
%DISP_LEADERBOARD Display the stepper leaderboard table and BLDC summary.
%   DISP\_LEADERBOARD(T, session, cfg) prints the stepper teams as a MATLAB
%   table (D-10), then prints a separate BLDC section below with band label,
%   SMAPE points, participation, and total points (D-11).

    stepper_mask = strcmp({cfg.teams.type}, 'stepper')';
    bldc_mask    = strcmp({cfg.teams.type}, 'bldc')';

    % --- Stepper leaderboard (D-10) ---
    fprintf('\n==================== STEPPER LEADERBOARD ====================\n');
    disp(T(stepper_mask, :));

    % --- BLDC section (D-11) ---
    if any(bldc_mask)
        fprintf('\n==================== BLDC TEAM ====================\n');
        row = T(bldc_mask, :);

        % Format best SMAPE
        if isnan(row.BestSMAPE)
            smape_str = 'N/A';
            band_str  = 'N/A';
        else
            smape_val = row.BestSMAPE;
            smape_str = sprintf('%.1f%%', smape_val);
            % Determine band label from cfg.bldc_smape_bands
            bands = cfg.bldc_smape_bands;   % e.g. [40, 80, 120, 160]
            edges = [0, bands];              % lower edges for label construction
            band_str = sprintf('%.0f-%.0f%%', edges(end), inf);  % fallback: 160+
            for k = 1:numel(bands)
                if smape_val < bands(k)
                    band_str = sprintf('%.0f-%.0f%%', edges(k), bands(k));
                    break
                end
            end
        end

        fprintf('  Team          : %s\n', char(row.Team));
        fprintf('  Best SMAPE    : %s  (band: %s)\n', smape_str, band_str);
        fprintf('  SMAPE points  : %.1f\n', row.SMAPEPoints);
        fprintf('  Participation : %s (%.0f pt)\n', ...
            yesno(row.ParticipationPoint > 0), row.ParticipationPoint);
        fprintf('  Total points  : %.1f / 5\n', row.TotalPoints);
    end
    fprintf('\n');
end

%[text] **export\_results** -- Auto-saves the leaderboard table to CSV and xlsx in
%[text] the configured export directory with timestamped filenames (D-13). Both stepper
%[text] and BLDC teams are included; BLDC Rank column is NaN (D-14). The xlsx write
%[text] is wrapped in try/catch in case the file is locked by Excel. Also saves
%[text] the full session struct to .mat for post-hoc inspection.

function export_results(T, cfg, session)
%EXPORT_RESULTS Auto-save leaderboard table to CSV, xlsx, and .mat.
%   EXPORT\_RESULTS(T, cfg, session) creates the export directory if needed,
%   generates a timestamped filename, writes CSV (always), xlsx (with
%   try/catch for locked-file robustness, T-04-03), and saves the full
%   session struct and table to .mat for post-hoc inspection.
%   Both stepper and BLDC rows are included; BLDC Rank column is NaN (D-14).

    % Create export directory if it does not exist
    if ~exist(cfg.export_dir, 'dir')
        mkdir(cfg.export_dir);
    end

    % Timestamped filename using datetime API (not legacy datestr -- Research Pitfall 4)
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmm'));
    base = fullfile(cfg.export_dir, sprintf('%s_%s', cfg.export_stem, timestamp));

    % CSV export (always succeeds -- reliable fallback)
    writetable(T, [base '.csv']);
    fprintf('Saved: %s\n', [base '.csv']);

    % xlsx export with try/catch (T-04-03: file may be locked by Excel)
    try
        writetable(T, [base '.xlsx']);
        fprintf('Saved: %s\n', [base '.xlsx']);
    catch e
        warning('scorer:xlsxfail', ...
            'xlsx export failed: %s\nClose Excel and retry. CSV was saved.', ...
            e.message);
    end

    % Save session struct and table to .mat for post-hoc inspection
    mat_file = [base '.mat'];
    save(mat_file, 'session', 'T');
    fprintf('Saved: %s\n', mat_file);
end

%[text] **plot\_score\_breakdown** -- Creates a grouped bar chart showing the score
%[text] breakdown per team (stepper + BLDC). Three bar groups per team: TimePoints,
%[text] SMAPEPoints, and ParticipationPoint (D-12).

function plot_score_breakdown(T, cfg)
%PLOT_SCORE_BREAKDOWN Grouped bar chart of score components for all teams.
%   PLOT\_SCORE\_BREAKDOWN(T, cfg) builds an N\_teams x 3 score matrix from
%   the leaderboard table and displays a grouped bar chart with one bar per
%   score component (TimePoints, SMAPEPoints, ParticipationPoint). Total
%   points are annotated above each team's bar group. (D-12, OUTP-05)

    % Build score matrix: N_teams x 3
    score_matrix = [T.TimePoints, T.SMAPEPoints, T.ParticipationPoint];

    % Create figure
    fig = figure('Name', 'Score Breakdown', 'NumberTitle', 'off'); %#ok<NASGU>

    % Grouped bar chart
    b = bar(score_matrix, 'grouped');

    % Color scheme
    b(1).FaceColor = [0.2, 0.6, 0.9];  % blue  -- time points
    b(2).FaceColor = [0.9, 0.5, 0.1];  % orange -- SMAPE points
    b(3).FaceColor = [0.3, 0.8, 0.3];  % green  -- participation

    % x-axis team labels
    set(gca, 'XTickLabel', cellstr(T.Team), 'XTickLabelRotation', 15);

    % Axis labels and title
    ylabel('Points');
    title('Competition Score Breakdown');

    % Legend
    legend({'Time Points', 'SMAPE Points', 'Participation'}, ...
           'Location', 'northeast');

    % Grid
    grid on;

    % Annotate total points above each team's bar group
    for i = 1:height(T)
        total = T.TotalPoints(i);
        text(i, max(score_matrix(i, :)) + 0.2, ...
             sprintf('%.1f', total), ...
             'HorizontalAlignment', 'center', ...
             'FontWeight', 'bold');
    end
end

%[appendix]{"version":"1.0"}
