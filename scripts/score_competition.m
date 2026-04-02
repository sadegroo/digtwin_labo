%% Swingup Competition Scoring
%[text] Scores team swingup attempts from .mldatx files. Session-based: load files one
%[text] pair at a time, assign to teams, finalize to produce leaderboard.
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
%[text] Interactive loop: load file pairs, assign to teams. Type 'done' to finalize.

% --- Session loop added in Plan 02 ---

function attempt = load_file_pair(hw_file, sim_file)
%LOAD_FILE_PAIR Load hardware and simulation .mldatx files, discriminate runs.
%   attempt = LOAD_FILE_PAIR(hw\_file, sim\_file) clears SDI, loads both files,
%   asserts exactly 2 runs, discriminates hw/sim by SimMode, and returns an
%   attempt struct. Returns empty [] on failure.

    attempt = [];

    % Clear SDI to prevent run contamination (LOAD-01, per D-02)
    Simulink.sdi.clear;

    % Load both files
    try
        Simulink.sdi.load(hw_file);
        Simulink.sdi.load(sim_file);
    catch e
        warning('scorer:loadfail', 'Failed to load files: %s', e.message);
        return
    end

    % Assert exactly 2 runs (per D-02)
    ids = Simulink.sdi.getAllRunIDs();
    if numel(ids) ~= 2
        warning('scorer:badruncount', ...
            'Expected 2 runs after loading file pair, got %d. Skipping.', numel(ids));
        return
    end

    % Access both runs
    r1 = Simulink.sdi.getRun(ids(1));
    r2 = Simulink.sdi.getRun(ids(2));

    % Display run summary (per D-01)
    fprintf('\n--- Run Summary ---\n');
    fprintf('Run 1: "%s" | SimMode: %s | Signals: %d | Time: [%.3f, %.3f] s\n', ...
        r1.Name, r1.SimMode, r1.SignalCount, r1.StartTime, r1.StopTime);
    fprintf('Run 2: "%s" | SimMode: %s | Signals: %d | Time: [%.3f, %.3f] s\n', ...
        r2.Name, r2.SimMode, r2.SignalCount, r2.StartTime, r2.StopTime);

    % Discriminate hardware vs simulation by SimMode (per D-01, RESEARCH primary recommendation)
    hw_id = [];
    sim_id = [];
    for i = 1:numel(ids)
        r = Simulink.sdi.getRun(ids(i));
        if strcmp(r.SimMode, 'external')
            hw_id = ids(i);
        elseif strcmp(r.SimMode, 'normal')
            sim_id = ids(i);
        end
    end

    % Fallback: if SimMode did not resolve both, use signal count (more signals = hardware)
    if isempty(hw_id) || isempty(sim_id)
        warning('scorer:simmode', ...
            'SimMode did not resolve both runs. Falling back to signal count.');
        if r1.SignalCount >= r2.SignalCount
            hw_id = ids(1);
            sim_id = ids(2);
        else
            hw_id = ids(2);
            sim_id = ids(1);
        end
    end

    % Display assignment
    hw_run  = Simulink.sdi.getRun(hw_id);
    sim_run = Simulink.sdi.getRun(sim_id);
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
    attempt.hw_file     = hw_file;
    attempt.sim_file    = sim_file;
    attempt.metrics     = struct();   % placeholder for Phase 3
end

%[appendix]{"version":"1.0"}
