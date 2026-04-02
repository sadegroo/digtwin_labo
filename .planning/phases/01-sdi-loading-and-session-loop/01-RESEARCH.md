# Phase 1: SDI Loading and Session Loop - Research

**Researched:** 2026-04-02
**Domain:** Simulink Data Inspector (SDI) API, MATLAB interactive scripting, session state management
**Confidence:** HIGH

## Summary

Phase 1 builds the session loop that loads `.mldatx` files, discriminates hardware from simulation runs, assigns files to teams, accumulates results, and detects the finalize command. All SDI API behavior was verified experimentally against `R2025b` using the project's example `.mldatx` files (`example_data/swingup_video.mldatx`, `example_data/swingup_sim.mldatx`).

The primary finding that changes the original assumption from CONTEXT.md: **each `.mldatx` file from the competition contains exactly 1 run, not 2.** A team submits two separate files (one hardware, one simulation). The script must call `Simulink.sdi.load` twice per team — once per file — and collect both run IDs before proceeding. The 2-run assertion in D-02 must be restated as: after loading both files for a team, exactly 2 runs must be present in the SDI session.

Run discrimination is solved definitively: `run.SimMode` is `'external'` for hardware runs and `'normal'` for simulation runs. This property was observed consistently across all test files and requires no heuristic. Signal count (hardware run has more signals than simulation run in our examples) provides a valid fallback if `SimMode` is ambiguous.

**Primary recommendation:** Use `SimMode` for run discrimination — it is deterministic and requires no scorer confirmation in the normal case. Display run summaries for scorer review only when `SimMode` is not `'external'` or `'normal'`.

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01:** Use a heuristic to identify hardware vs simulation runs, then display a brief summary (name, signal count, time range) and let the scorer confirm or swap. Guards against unvalidated heuristic concern.
- **D-02:** Assert exactly 2 runs per `.mldatx` file. Any other count triggers a warning and the file is skipped.
- **D-03:** Use `uigetfile('*.mldatx')` for file selection. Use `input()` for team assignment and finalize command.
- **D-04:** The finalize keyword is `'done'` — typing `done` at the team assignment prompt ends the session loop.
- **D-05:** Define teams in `cfg.teams` as a struct array with fields `name` (string) and `type` (`'stepper'` or `'bldc'`). Default: 4 stepper teams + 1 BLDC team.
- **D-06:** On each file load, present the team list via `listdlg` so the scorer selects by clicking.
- **D-07:** Multiple files for the same team append to that team's attempt list (no overwrite).
- **D-08:** Session state is a struct `session` with field `teams`, where each team entry holds: `name`, `type`, and `attempts` (a struct array). Each attempt stores the raw run IDs, file path, and a placeholder for metrics.
- **D-09:** Wrap file loading in `try/catch`. On failure, display with `warning()`, skip the file, continue the session loop.

### Claude's Discretion

- Internal struct layout details (field names, nesting depth) — Claude can choose the most practical structure as long as it supports the requirements.
- `Simulink.sdi.clear` call placement and SDI cleanup strategy — implementation detail.
- Exact heuristic for run discrimination (signal density vs creation order vs `getLatest()`) — researcher should investigate SDI API to determine the most reliable approach.

### Deferred Ideas (OUT OF SCOPE)

None — discussion stayed within phase scope.

</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| LOAD-01 | Script loads `.mldatx` files via `Simulink.sdi.load` with `Simulink.sdi.clear` called before each file to prevent run contamination | Verified: `Simulink.sdi.clear` removes all runs (tested: 0 runs after clear). `Simulink.sdi.load` is the correct function name in R2025b. |
| LOAD-02 | Script separates hardware run (archived) from simulation run (recent) within each `.mldatx` file | Verified: `run.SimMode` is `'external'` for hardware, `'normal'` for simulation. Deterministic — no heuristic needed. |
| LOAD-03 | Script gracefully skips corrupt or unloadable `.mldatx` files with `try/catch`, logging the error and continuing | Verified: loading a non-`.mldatx` file throws `simulation_data_repository:sdr:SessionFileNotFound` — caught by `try/catch`. |
| LOAD-04 | Script operates in an incremental session loop: scorer provides one file at a time; script processes immediately before waiting for next | Research clarifies: each file has 1 run; the loop needs to accept 2 files per team (hw + sim) before processing. |
| LOAD-05 | Script maintains a session state struct accumulating per-team results; new file appends to attempt list without clearing | Verified: `session.teams(i).attempts` as cell array supports `{end+1}` appending pattern. |
| LOAD-06 | Script provides a "finalize" command; finalization triggers competitive ranking and leaderboard | Research confirms: `strcmpi(cmd, 'done')` detects the keyword at the `input()` prompt. |

</phase_requirements>

---

## Critical Finding: File-per-Run Structure

**`.mldatx` files each contain exactly 1 run.** This was verified experimentally:

- `swingup_video.mldatx` → 1 run, `SimMode='external'` (hardware), 17 signals
- `swingup_sim.mldatx` → 1 run, `SimMode='normal'` (simulation), 14 signals
- A combined `.mldatx` (saved via `Simulink.sdi.save` after loading both) → 2 runs

**Implication for D-02:** The "2 runs per `.mldatx` file" assertion must be reframed. The intended workflow is:

1. Scorer provides the hardware `.mldatx` file (1 run loaded)
2. Scorer provides the simulation `.mldatx` file (1 more run loaded, total = 2)
3. Script asserts `numel(Simulink.sdi.getAllRunIDs()) == 2` after both loads

**Option A (two-file workflow):** Prompt for file 1 (hw), load without clear, prompt for file 2 (sim), load without clear, assert count == 2.
**Option B (combined-file workflow):** Teams pre-save a combined `.mldatx` file; script loads it and asserts count == 2 after a single load.

Both options are supported by the SDI API. The planner must pick one. Based on the competition context (teams likely export from SDI separately), Option A is more practical.

---

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `Simulink.sdi` | R2025b built-in | Load `.mldatx`, enumerate runs, access signals | Only supported API for `.mldatx` format |
| `uigetfile` | R2025b built-in | File picker dialog | Prevents path typos, native MATLAB UI |
| `listdlg` | R2025b built-in | Team selection from a list | Enforces valid team names, no typos |
| `input()` | R2025b built-in | Command-line prompt for finalize command | Simple, no toolbox dependency |
| `warning()` | R2025b built-in | Non-fatal error reporting | Standard MATLAB convention |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `Simulink.sdi.save` | R2025b built-in | Save session for debugging | Optional: save session state before finalize |
| `struct` array with cell fields | MATLAB core | Session state accumulation | Use `{}` for `attempts` field, not `[]` |

**No external toolboxes required.** Only Simulink (for SDI API) and base MATLAB.

---

## Architecture Patterns

### Recommended Script Structure

```
score_competition.m (Live Script .m format)
├── %% Configuration            — cfg struct with teams, thresholds
├── %% Session Initialization   — session struct, SDI clear
├── %% Session Loop             — while loop: load files, pick team, accumulate
│   ├── uigetfile (hw file)
│   ├── uigetfile (sim file)
│   ├── SDI load (2 files)
│   ├── assert 2 runs
│   ├── discriminate hw/sim
│   ├── scorer confirm/swap
│   ├── listdlg team assignment
│   └── append attempt to session
└── %% Finalization             — break on 'done', hand off to Phase 2+
```

### Pattern 1: SDI Load-and-Assert

**What:** Clear SDI, load two files, assert exactly 2 runs.
**When to use:** Every team file pair.

```matlab
% Source: verified experimentally in R2025b
Simulink.sdi.clear;
Simulink.sdi.load(hw_file);
Simulink.sdi.load(sim_file);
ids = Simulink.sdi.getAllRunIDs();  % returns int32 array
assert(numel(ids) == 2, ...
    'Expected 2 runs, got %d. Check file pair.', numel(ids));
```

### Pattern 2: SimMode-Based Run Discrimination

**What:** Use `run.SimMode` to identify hardware vs simulation run deterministically.
**When to use:** After asserting 2 runs exist.
**Fallback:** If `SimMode` is neither `'external'` nor `'normal'`, fall back to signal count comparison (more signals = hardware run).

```matlab
% Source: verified experimentally — SimMode 'external'=hardware, 'normal'=simulation
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
% Fallback if SimMode didn't resolve both
if isempty(hw_id) || isempty(sim_id)
    % Use signal count: hardware run has more signals
    [~, hw_idx] = max([r1.SignalCount, r2.SignalCount]);
    % ... assign accordingly
end
```

### Pattern 3: Session State Struct

**What:** Accumulate all team data in a single `session` struct.
**Key:** Use a cell array for `attempts` (not struct array) to allow heterogeneous attempt data across phases.

```matlab
% Initialization
session = struct();
session.teams = cfg.teams;  % copy name/type fields
for i = 1:numel(session.teams)
    session.teams(i).attempts = {};  % cell array, NOT []
end

% Appending an attempt
attempt = struct();
attempt.hw_run_id  = hw_id;     % int32 SDI run ID
attempt.sim_run_id = sim_id;    % int32 SDI run ID
attempt.file_path  = file_path; % char — absolute path
attempt.metrics    = struct();  % placeholder, populated in Phase 3

team_idx = find(strcmp({session.teams.name}, chosen_team_name));
session.teams(team_idx).attempts{end+1} = attempt;
```

### Pattern 4: Session Loop with Finalize Detection

**What:** Interactive while loop that checks for `'done'` at every prompt.
**When to use:** Main script body.

```matlab
% Source: standard MATLAB interactive pattern
while true
    % Prompt for hardware file
    [hw_fname, hw_dir] = uigetfile('*.mldatx', 'Select HARDWARE run');
    if isequal(hw_fname, 0)
        fprintf('File selection cancelled.\n');
        continue
    end
    hw_file = fullfile(hw_dir, hw_fname);

    % Prompt for simulation file
    [sim_fname, sim_dir] = uigetfile('*.mldatx', 'Select SIMULATION run');
    if isequal(sim_fname, 0)
        fprintf('File selection cancelled.\n');
        continue
    end
    sim_file = fullfile(sim_dir, sim_fname);

    % Load and validate
    try
        Simulink.sdi.clear;
        Simulink.sdi.load(hw_file);
        Simulink.sdi.load(sim_file);
        ids = Simulink.sdi.getAllRunIDs();
        if numel(ids) ~= 2
            warning('scorer:badfile', 'Expected 2 runs, got %d. Skipping.', numel(ids));
            continue
        end
    catch e
        warning('scorer:loadfail', 'Load failed: %s', e.message);
        continue
    end

    % Discriminate runs (see Pattern 2)
    % ...

    % Team assignment
    team_names = {cfg.teams.name};
    sel = listdlg('ListString', team_names, ...
                  'SelectionMode', 'single', ...
                  'Name', 'Assign to team', ...
                  'PromptString', 'Select team for this file:');
    if isempty(sel), continue; end

    % Check finalize
    cmd = input('Type team number or ''done'' to finalize: ', 's');
    if strcmpi(cmd, 'done')
        break
    end

    % Append attempt
    % ...
end
```

**Note on finalize integration with D-06:** D-03 says `input()` for team assignment and finalize. D-06 says `listdlg` for team selection. The resolved pattern is: `listdlg` first (click-based), then optionally `input()` after listdlg for the `done` check. Simplest: a separate `input()` prompt after each file pair that only accepts `'done'` or `'continue'`/`Enter`.

### Anti-Patterns to Avoid

- **Calling `Simulink.sdi.getRunIDList`:** Does not exist in R2025b. Use `Simulink.sdi.getAllRunIDs()` instead.
- **Using `Simulink.sdi.getLatestRun`:** Does not exist in R2025b. Use `getAllRunIDs` and pick the last element if recency is needed, or use `RunIndex` property.
- **Calling `Simulink.sdi.getNumRuns`:** Does not exist in R2025b.
- **Using `run.SimulationStartTime`:** Does not exist in R2025b.
- **Calling `Simulink.sdi.getRepository`:** Does not exist in R2025b.
- **Loading both files without `Simulink.sdi.clear`:** Results in run accumulation across team attempts, corrupting counts.
- **Initializing attempts field as `[]`:** Must use `{}` (empty cell array) for correct `{end+1}` appending.
- **Assuming `listdlg` is non-blocking in batch mode:** `listdlg` and `uigetfile` require a display; the script is always run interactively.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| File loading from `.mldatx` | Custom binary parser | `Simulink.sdi.load` | Proprietary format; only SDI can read it |
| Run enumeration | Maintain your own run list | `Simulink.sdi.getAllRunIDs()` | SDI is the source of truth |
| File selection dialog | String `input()` with path validation | `uigetfile('*.mldatx')` | Native dialog, handles edge cases (cancel, invalid paths) |
| Team list presentation | Free-text `input()` for team name | `listdlg` | Prevents typos, enforces valid roster |

**Key insight:** The SDI session IS the run database for this script. Don't duplicate its state in your own data structures — store only the run IDs returned by `getAllRunIDs()` and query the SDI session when needed.

---

## Common Pitfalls

### Pitfall 1: Run Contamination Between Teams
**What goes wrong:** Script loads file for Team B without clearing SDI first; IDs from Team A's session are still present, causing `numel(getAllRunIDs()) == 3` or more.
**Why it happens:** `Simulink.sdi.load` accumulates runs; it does not replace them.
**How to avoid:** Always call `Simulink.sdi.clear` before loading a new team's files.
**Warning signs:** `numel(ids) > 2` after loading 2 files.

### Pitfall 2: One File Has 1 Run, Not 2
**What goes wrong:** Scorer loads only the hardware file (not the simulation file), script asserts 2 runs and fails.
**Why it happens:** Two separate files are needed for a complete team submission (one hardware, one simulation).
**How to avoid:** Prompt for both files explicitly before loading either. Validate both paths before any SDI calls.
**Warning signs:** `numel(ids) == 1` after clear + load.

### Pitfall 3: SimMode Not Set (Edge Case)
**What goes wrong:** A run saved outside Simulink or from an older MATLAB version may have `SimMode = ''` or `SimMode = 'Not available'`.
**Why it happens:** SimMode is metadata set by Simulink at run time; older exports may lack it.
**How to avoid:** Implement fallback to signal count comparison. Log a warning and ask scorer to confirm assignment.
**Warning signs:** Neither `hw_id` nor `sim_id` is assigned after the SimMode loop.

### Pitfall 4: `uigetfile` Cancel Returns 0
**What goes wrong:** Scorer presses Cancel in the file dialog; `hw_fname` is `0` (numeric), and `fullfile(0, '')` fails or produces garbage.
**Why it happens:** `uigetfile` returns `0` for the filename on cancel, not an error.
**How to avoid:** Check `if isequal(hw_fname, 0); continue; end` immediately after every `uigetfile` call.
**Warning signs:** MATLAB error about numeric filename argument.

### Pitfall 5: `listdlg` Cancel Returns Empty
**What goes wrong:** Scorer closes the listdlg without selecting a team; `sel` is `[]`.
**Why it happens:** `listdlg` returns `[]` if the user cancels.
**How to avoid:** Check `if isempty(sel); continue; end` immediately after `listdlg`.
**Warning signs:** Indexing error on `cfg.teams(sel)`.

### Pitfall 6: Struct Array with Cell Field Initialization
**What goes wrong:** `struct('attempts', {[]})` initializes attempts as a numeric empty array; `{end+1}` assignment then fails type checks.
**Why it happens:** MATLAB struct literal syntax is subtle — `{[]}` in a struct literal creates a struct array where each element has `[]`, not a cell array.
**How to avoid:** After struct array creation, explicitly loop and set each `.attempts = {}`.
**Warning signs:** `class(session.teams(1).attempts)` is `'double'` not `'cell'`.

---

## Code Examples

Verified patterns from direct R2025b testing:

### Loading and Run Count Assertion
```matlab
% Source: verified in R2025b against example_data/ files
Simulink.sdi.clear;
Simulink.sdi.load(hw_file);    % 1 run added (SimMode='external')
Simulink.sdi.load(sim_file);   % 1 run added (SimMode='normal')
ids = Simulink.sdi.getAllRunIDs();  % returns int32 row vector
% ids is empty int32 after Simulink.sdi.clear (verified)
assert(numel(ids) == 2, 'Expected 2 runs, got %d', numel(ids));
```

### Run Property Access
```matlab
% Source: verified — full property list from R2025b Run object
r = Simulink.sdi.getRun(ids(1));
r.Name         % char — e.g., 'Run 1: RRpendulum_digtwin_swingup'
r.SimMode      % char — 'external' | 'normal' | 'accelerator' | ...
r.SignalCount  % double — number of logged signals
r.StartTime    % double — [s]
r.StopTime     % double — [s]
r.DateCreated  % datetime object
r.RunIndex     % double — 1-based index in this SDI session
r.Model        % char — Simulink model name
r.SLVersion    % char — e.g., 'Simulink 24.2 (R2024b)'
```

### Signal Access
```matlab
% Source: verified in R2025b
r = Simulink.sdi.getRun(hw_id);

% By index (1-based)
sig = r.getSignalByIndex(1);

% By name (returns array if multiple matches)
sigs = r.getSignalsByName('q2_rev');  % returns Simulink.sdi.Signal array

% Get all signal IDs
sigIDs = r.getAllSignalIDs();  % returns ID array

% Access timeseries data
ts = sig.Values;   % MATLAB timeseries object
ts.Time            % time vector [s]
ts.Data            % data vector
sig.NumPoints      % number of samples
sig.Name           % signal name
sig.BlockPath      % Simulink block path
sig.Domain         % 'Time' or 'Parameters'
```

### Error Message Format from Corrupt File
```matlab
% Source: verified in R2025b
% e.identifier = 'simulation_data_repository:sdr:SessionFileNotFound'
% e.message = '<path> is not a valid Simulation Data Inspector file.'
```

### Session Struct Initialization
```matlab
% Source: verified MATLAB struct array pattern
cfg.teams(1) = struct('name', 'Alpha', 'type', 'stepper');
cfg.teams(2) = struct('name', 'Beta',  'type', 'stepper');
cfg.teams(3) = struct('name', 'Gamma', 'type', 'stepper');
cfg.teams(4) = struct('name', 'Delta', 'type', 'stepper');
cfg.teams(5) = struct('name', 'BLDC',  'type', 'bldc');

session = struct();
session.teams = cfg.teams;
for i = 1:numel(session.teams)
    session.teams(i).attempts = {};  % must be {} not []
end
```

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `Simulink.sdi.getRunIDList` | `Simulink.sdi.getAllRunIDs()` | Pre-R2025b → R2025b | API rename; old name throws "unable to resolve" |
| `Simulink.sdi.getNumRuns` | `numel(Simulink.sdi.getAllRunIDs())` | — | Function does not exist; compute from ID list |
| `Simulink.sdi.getLatestRun` | `Simulink.sdi.getRun(ids(end))` | — | Function does not exist; use last ID in array |
| Signal density heuristic | `run.SimMode` property | R2025b | SimMode is deterministic; no heuristic needed |

**Deprecated/outdated (do not use):**
- `Simulink.sdi.getRunIDList` — renamed, not in R2025b
- `Simulink.sdi.getNumRuns` — not in R2025b
- `Simulink.sdi.getLatestRun` — not in R2025b
- `Simulink.sdi.getRepository` — not in R2025b
- `run.SimulationStartTime` — not a valid Run property in R2025b

---

## Open Questions

1. **Two-file vs combined-file workflow**
   - What we know: Each `.mldatx` from the project has 1 run. Teams can save a combined 2-run file using SDI export. The script can support either model.
   - What's unclear: Whether competition teams will submit combined files (one `.mldatx` with 2 runs) or two separate files.
   - Recommendation: Support two separate files (prompts hw then sim separately). This is more forgiving and doesn't require teams to know how to combine SDI sessions. The planner should clarify with the instructor before implementing.

2. **Finalize command integration with `listdlg`**
   - What we know: D-03 specifies `input()` for finalize detection; D-06 specifies `listdlg` for team selection. These are two separate interactions.
   - What's unclear: Exact UX flow — does `input()` come before or after `listdlg`? Or is there a separate "finalize?" prompt after each successful file load?
   - Recommendation: After each successful file pair load, show `listdlg` for team assignment. Then ask `input('Load another file? [Enter] or type ''done'': ', 's')`. This keeps `listdlg` as the team picker and `input` as the loop control.

3. **SimMode for externally saved files**
   - What we know: `SimMode = 'external'` for hardware runs captured via Simulink External Mode; `'normal'` for regular simulation.
   - What's unclear: If a student saves their SDI session from a non-Simulink context, `SimMode` may be empty string or `'Not available'`.
   - Recommendation: Implement the signal count fallback and always display the run summary for scorer confirmation (as D-01 requires).

---

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| MATLAB R2025b | Runtime | Yes | R2025b (25.2.0.3150157 Update 4) | — |
| Simulink (for SDI) | `Simulink.sdi.*` | Yes | R2025b built-in | No fallback — SDI is required |
| `uigetfile` | File selection (D-03) | Yes | Base MATLAB | — |
| `listdlg` | Team selection (D-06) | Yes | Base MATLAB | — |
| Display / GUI | `uigetfile`, `listdlg` | Yes | Windows 11 interactive session | No fallback — script requires interactive use |
| `.mldatx` test files | Development/validation | Yes | `example_data/swingup_video.mldatx`, `example_data/swingup_sim.mldatx`, `example_data/combined_test.mldatx` | — |

**Missing dependencies with no fallback:** None.

**Missing dependencies with fallback:** None.

---

## Project Constraints (from CLAUDE.md)

These directives apply to all code written in Phase 1:

1. **Project open check:** Every script must begin with `if isempty(matlab.project.rootProject); openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj'); end`
2. **Live Script .m format:** Use `%[text]` for rich text, `%%` for section breaks, `%[appendix]{"version":"1.0"}` at end; escape `_` as `\_` inside `%[text]` blocks
3. **Git commit rule:** Only commit files registered in `digtwin_labo.prj`. Script must be added to the project before committing.
4. **File location:** `scripts/score_competition.m` per ROADMAP (not nested in a subdirectory)
5. **Config struct pattern:** Tunable parameters go in `cfg` struct at top; consistent with project convention (`params` struct in parameter scripts)
6. **`_num` suffix convention:** Does NOT apply to scoring script (this convention is for physical parameters in EOM/controller scripts)
7. **No MATLAB App Designer / GUI:** Plain script with `listdlg`/`uigetfile` is the specified approach

---

## Sources

### Primary (HIGH confidence)
- Direct MATLAB R2025b execution — all SDI API calls verified against live files
  - `Simulink.sdi.clear`, `Simulink.sdi.load`, `Simulink.sdi.getAllRunIDs`, `Simulink.sdi.getRun` — confirmed working
  - `run.SimMode`, `run.SignalCount`, `run.Name`, `run.RunIndex`, `run.DateCreated` — confirmed as valid properties
  - `run.getSignalByIndex`, `run.getAllSignalIDs`, `run.getSignalsByName` — confirmed working
  - `sig.Values` returns `timeseries`; `sig.Name`, `sig.NumPoints`, `sig.BlockPath` confirmed
  - Error identifier: `simulation_data_repository:sdr:SessionFileNotFound` for invalid files
- `example_data/swingup_video.mldatx` — 1 run, SimMode='external', 17 signals
- `example_data/swingup_sim.mldatx` — 1 run, SimMode='normal', 14 signals
- `example_data/combined_test.mldatx` — 2-run file created by `Simulink.sdi.save` after loading both

### Secondary (MEDIUM confidence)
- MathWorks documentation (implicit, not directly fetched — API behavior confirmed via live testing)
- Project's `scripts/RRpendulum_swingup_controldesign.m` — patterns for `openProject` check, struct layout, Live Script format

### Tertiary (LOW confidence)
- None.

---

## Metadata

**Confidence breakdown:**
- SDI API surface (function names, properties): HIGH — all verified in live R2025b session
- Run discrimination via SimMode: HIGH — confirmed on actual project files
- File-per-run structure: HIGH — verified empirically; 1 run per `.mldatx` in example data
- Session struct design: HIGH — verified struct/cell array MATLAB patterns
- Interactive loop design (uigetfile, listdlg, input): HIGH — all functions confirmed present; pattern is standard MATLAB
- Finalize UX integration: MEDIUM — the exact interaction sequence between listdlg and input() is implementation-defined; Open Question #2

**Research date:** 2026-04-02
**Valid until:** 2026-05-02 (SDI API is stable; low risk of change within 30 days)

%[appendix]{"version":"1.0"}
