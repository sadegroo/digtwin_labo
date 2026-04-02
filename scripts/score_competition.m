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

%% Session Initialization
%[text] Initialize the session state struct that accumulates results across all loaded files.

session = struct();
session.teams = cfg.teams;
for i = 1:numel(session.teams)
    session.teams(i).attempts = {};
end
session.finalized = false;

fprintf('Session started. %d teams configured.\n', numel(cfg.teams));

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

    %% Assign to team (per D-06: listdlg)
    sel = listdlg('ListString', team_names, ...
                  'SelectionMode', 'single', ...
                  'Name', 'Assign to Team', ...
                  'PromptString', sprintf('Assign "%s" to which team?', fname), ...
                  'ListSize', [300, 200]);
    if isempty(sel)
        fprintf('Team selection cancelled. Skipping this file.\n');
        continue
    end

    % Append attempt to selected team (per D-07: no overwrite)
    team_idx = sel;
    session.teams(team_idx).attempts{end+1} = attempt;
    fprintf('Added attempt %d for team "%s" (%s).\n', ...
        numel(session.teams(team_idx).attempts), ...
        session.teams(team_idx).name, ...
        session.teams(team_idx).type);

    % Print session progress
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
fprintf('Proceed to Phase 2 (signal selection and alignment).\n');

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

%[appendix]{"version":"1.0"}
