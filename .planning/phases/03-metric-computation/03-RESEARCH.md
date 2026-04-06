# Phase 3: Metric Computation - Research

**Researched:** 2026-04-06
**Domain:** MATLAB signal processing — angle wrapping, sustained-hold detection, SMAPE computation
**Confidence:** HIGH (domain is pure MATLAB numerics; no external APIs)

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**Unit Normalization**
- D-01: Convert aligned hw_q2 and sim_q2 to radians at the start of metric computation, regardless of per-file display unit. All thresholds, angle wrapping formulas, and comparisons operate in radians.
- D-02: Conversion happens inside `compute_metrics` as its first step. The `attempt.aligned` struct stays in its native unit for plotting. Caller does not pre-convert.

**Swingup Detection**
- D-03: Success requires `|abs(q2) - pi| < 2deg` (0.0349 rad) sustained for at least 1 continuous second.
- D-04: Swingup time is backdated to the ENTRY of the first sustained 1-second hold, not just any crossing.
- D-05: Both +pi and -pi count as upright. Detection uses `abs(q2)` so sign does not matter.

**Metrics Timing**
- D-06: Metrics computed immediately after alignment, BEFORE truncation. Order: load -> team -> signals -> align -> compute_metrics -> truncate -> store -> overlay.
- D-07: Metrics use full pre-truncation aligned data.
- D-08: After computation, print one-liner: `"Metrics: swingup=YES t=3.42s SMAPE=12.3% participation=YES"`.

**SMAPE Window Policy**
- D-09: SMAPE window = `max(5s, time_to_first_90deg)` where time_to_first_90deg is first sample where `|q2| > pi/2` on hw signal.
- D-10: If `|q2|` never reaches pi/2, skip SMAPE entirely and mark attempt as ineligible.
- D-11: If participation achieved, SMAPE computed over window; attempt marked eligible.

**SMAPE Computation**
- D-12: Use angular difference formula `mod(hw - sim + pi, 2*pi) - pi` for wrapping near +/-pi.
- D-13: Denominator guard: exclude samples where `|hw_q2| + |sim_q2| < epsilon`.

**Participation**
- D-14: Participation = `|q2| > pi/2` at any point during attempt on hw signal (flag separate from SMAPE eligibility).

### Claude's Discretion
- Epsilon value for SMAPE denominator guard (reasonable default given data precision and radian range)
- Internal function signature for `compute_metrics` (inputs: aligned struct, cfg, q2_unit; output: metrics struct)
- Whether participation check uses a consecutive-sample filter (METR-02 mentions 10 consecutive samples) or simple any() check
- How to handle edge case where data is shorter than 5s (use available data)

### Deferred Ideas (OUT OF SCOPE)
None — discussion stayed within phase scope.

</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| METR-01 | Swingup success: q2 crosses ±π AND holds within ±2° for at least 1 continuous second | Sustained-hold detection pattern using `conv` or sliding-window `all()` on boolean mask |
| METR-02 | Participation: |q2| exceeds π/2 at any point (noise-robust: 10 consecutive samples) | Consecutive-sample filter via `conv(in_band, ones(1,N), 'valid') >= N` |
| METR-03 | Swingup time: from t=0 to first ±π crossing on hw signal (entry of sustained hold per D-04) | Index into `aligned.t` at entry of first qualifying window |
| METR-04 | SMAPE via `interp1` resampling (sim already resampled onto hw grid in align_signals) | interp1 already done in Phase 2; direct element-wise computation is correct here |
| METR-05 | Configurable SMAPE window: fixed / angle / swingup modes (D-09 resolves to hybrid: `max(5s, time_to_first_90deg)`) | Window-end index lookup by time comparison |
| METR-06 | Division-by-zero guard: denominator check before SMAPE sample inclusion | Boolean mask on denominator; epsilon chosen as 1e-3 rad (see below) |
| METR-07 | Angle wrapping via `mod(hw-sim+pi, 2*pi)-pi` for numerator | Verified MATLAB formula; handles +pi/-pi discontinuity correctly |

</phase_requirements>

---

## Summary

Phase 3 is self-contained numerical processing: given `attempt.aligned` (already in the native q2 unit), convert to radians, then run four sequential sub-computations: participation check, swingup success detection, swingup time extraction, and SMAPE. All sub-computations are pure MATLAB arithmetic with no external calls.

The most conceptually subtle part is the sustained-hold detector (D-03/D-04). The naive approach of checking `|abs(q2) - pi| < tol` and using `find(..., 1)` gives the FIRST ENTRY time but does not enforce the 1-second hold. The correct pattern uses a sliding-window `all()` to find the first window of N consecutive samples that are all within tolerance, then backdate the reported time to the start of that window.

SMAPE near ±pi requires the angular difference formula in the numerator (D-12). Without it, a hardware reading of +3.13 rad and a simulation reading of -3.13 rad would produce a numerator of 6.26 rad instead of the correct ~0.02 rad. The denominator guard (D-13) excludes samples where both signals are near zero. Epsilon = 1e-3 rad is appropriate: 1 mrad is well below sensor noise floor (hardware encoder is ~0.3 mrad/count at 2000 counts/rev), so any sample pair with |hw|+|sim| < 1e-3 is genuinely near-zero, not just noise.

**Primary recommendation:** Implement `compute_metrics` as a single local function added to `score_competition.m`. Call it between `align_signals` (line 202) and `truncate_at_swingup` (line 205). The function receives `(aligned, cfg, q2_unit)` and returns a flat metrics struct. This matches the established local-function pattern throughout the script.

---

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| MATLAB base | R2025b | Arithmetic, logical indexing, `mod`, `conv`, `interp1` | All operations are built-in; no toolbox required |
| Simulink Data Inspector API | R2025b | Already used in Phases 1-2; data is already extracted | Phase 3 consumes extracted arrays only — no SDI calls needed |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| None | — | — | Phase 3 is pure array math; no additional toolboxes required |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| `conv`-based sliding window | `movsum` / `movmean` | `movsum` is cleaner for the hold check but requires R2016a+; both are fine in R2025b |
| `mod(hw-sim+pi, 2*pi)-pi` | `atan2(sin(hw-sim), cos(hw-sim))` | Both produce identical results for scalar angles; `mod` is marginally faster on vectors |

**Installation:** No installation required. All operations are built-in MATLAB.

---

## Architecture Patterns

### Recommended Project Structure

`compute_metrics` is a LOCAL FUNCTION inside `score_competition.m`. No new files are created.

```
score_competition.m
├── Session loop body (lines 66–244)
│   ├── [EXISTING] align_signals(...)          ← line ~201
│   ├── [NEW]      compute_metrics(...)         ← insert here (D-06)
│   └── [EXISTING] truncate_at_swingup(...)     ← line ~205
└── Local functions (lines 263+)
    ├── [EXISTING] load_attempt
    ├── [EXISTING] select_signals
    ├── [EXISTING] extract_signal
    ├── [EXISTING] align_signals
    ├── [EXISTING] truncate_at_swingup
    └── [NEW]      compute_metrics              ← add here
```

### Pattern 1: Sustained-Hold Detection

**What:** Find the first contiguous window of N samples where a boolean condition is TRUE for all samples. Report the time at the START of that window.

**When to use:** METR-01 (1-second hold at ±pi), METR-02 (10-consecutive-sample participation).

**Example:**
```matlab
% Sustained-hold: find first window where all N samples satisfy condition
% Source: standard MATLAB sliding-window pattern
function idx = first_sustained_idx(mask, N)
    % Returns the start index of the first window of length N
    % where mask(idx:idx+N-1) are all true.
    % Returns [] if no such window exists.
    if N <= 1
        idx = find(mask, 1, 'first');
        return
    end
    kernel  = ones(1, N);
    counts  = conv(double(mask), kernel, 'valid');  % length = numel(mask)-N+1
    win_idx = find(counts >= N, 1, 'first');        % first qualifying window start
    if isempty(win_idx)
        idx = [];
    else
        idx = win_idx;   % index into original mask (conv 'valid' preserves 1-based)
    end
end
```

Usage for swingup detection at 2 kHz (N = hold_time * sample_rate):
```matlab
% N samples = 1 second * 2000 Hz = 2000 samples (hw data at ~2 kHz)
% Use actual sample rate: N = round(cfg.swingup_hold_time / median(diff(t)))
in_band = abs(abs(hw_q2_rad) - pi) < tol_rad;   % boolean mask
N       = round(cfg.swingup_hold_time / median(diff(t)));
entry_idx = first_sustained_idx(in_band, N);
success   = ~isempty(entry_idx);
```

### Pattern 2: SMAPE With Angle Wrapping and Denominator Guard

**What:** Element-wise symmetric mean absolute percentage error using angular difference, with per-sample exclusion when denominator is below epsilon.

**When to use:** METR-04, METR-06, METR-07.

**Example:**
```matlab
% SMAPE with angular difference numerator and denominator guard
% Source: METR-07 requirement + D-12/D-13 decisions
function smape = compute_smape_angular(hw_rad, sim_rad, epsilon)
    % Angular difference handles wrapping near +/-pi (METR-07)
    num = abs(mod(hw_rad - sim_rad + pi, 2*pi) - pi);

    % Denominator = (|hw| + |sim|) / 2  -- symmetric SMAPE convention
    denom = (abs(hw_rad) + abs(sim_rad)) / 2;

    % Exclude samples where denominator is below epsilon (METR-06, D-13)
    valid = denom >= epsilon;
    if ~any(valid)
        smape = NaN;   % no valid samples
        return
    end

    smape = mean(num(valid) ./ denom(valid)) * 100;   % percent
end
```

### Pattern 3: Unit Conversion at Function Entry (D-01/D-02)

**What:** Convert from display unit to radians as the very first step inside `compute_metrics`. The incoming `aligned` struct stays untouched.

**When to use:** D-01 mandates this. Prevents unit-mismatch bugs if the scorer uses rev or deg display unit.

**Example:**
```matlab
function metrics = compute_metrics(aligned, cfg, q2_unit)
    % D-01/D-02: convert to radians immediately; aligned struct is NOT modified
    switch q2_unit
        case 'rev', scale = 2 * pi;
        case 'deg', scale = pi / 180;
        case 'rad', scale = 1;
        otherwise,  scale = 2 * pi;  % default to rev
    end
    hw_q2_rad  = aligned.hw_q2  * scale;
    sim_q2_rad = aligned.sim_q2 * scale;
    t          = aligned.t;
    % ... rest of computation in radians
end
```

### Pattern 4: SMAPE Window Endpoint Computation (D-09)

**What:** Determine the index at which SMAPE window ends. Window = `max(5s, time to first |q2| > pi/2)`.

**Example:**
```matlab
% D-09: SMAPE window = max(5s, time_to_first_90deg)
thr_90deg = pi / 2;
idx_90deg = find(abs(hw_q2_rad) > thr_90deg, 1, 'first');

if isempty(idx_90deg)
    % D-10: never reached pi/2 -- ineligible
    metrics.smape_eligible = false;
    metrics.smape = NaN;
else
    metrics.smape_eligible   = true;
    t_90deg     = t(idx_90deg);
    t_win_end   = max(cfg.smape_fixed_duration, t_90deg);
    win_end_idx = find(t >= t_win_end, 1, 'first');
    if isempty(win_end_idx)
        win_end_idx = numel(t);  % use all available data (edge case: data < 5s)
    end
    hw_win  = hw_q2_rad(1:win_end_idx);
    sim_win = sim_q2_rad(1:win_end_idx);
    metrics.smape = compute_smape_angular(hw_win, sim_win, epsilon);
end
```

### Anti-Patterns to Avoid

- **Using `find(in_band, 1, 'first')` for swingup time without hold check:** Reports first crossing even if it immediately falls back. MUST use the sliding-window pattern (Pattern 1). Swingup time is backdated to the entry of the FIRST qualifying window, not first crossing.
- **Using `diff(hw - sim)` for SMAPE numerator:** Plain subtraction fails at the +pi/-pi discontinuity. Always use `mod(hw - sim + pi, 2*pi) - pi`.
- **Using `mean(num ./ denom)` without excluding near-zero denominators:** Produces Inf or very large SMAPE values that dominate the mean. Always apply the epsilon mask first.
- **Converting units after calling `compute_metrics`:** D-02 is explicit — conversion happens INSIDE the function. Do not pre-convert in the session loop.
- **Running `compute_metrics` on truncated data:** D-06/D-07 require metrics before truncation. The 1-second hold detection and SMAPE window need the full aligned data.
- **Hard-coding sample rate as 2000 Hz:** Compute N dynamically from `median(diff(t))` to handle any actual sample rate stored in the data.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Sliding window of N consecutive TRUE samples | Custom for-loop searching contiguous regions | `conv(double(mask), ones(1,N), 'valid') >= N` | One-liner, vectorized, handles all edge cases |
| Angle wrapping in subtraction | Custom `if` branches for +/-pi crossings | `mod(a - b + pi, 2*pi) - pi` | Built-in, handles all quadrant combinations correctly |
| Sample rate estimation | Store and pass Ts separately | `median(diff(t))` | Robust to any timing jitter in the stored time vector |

**Key insight:** All Phase 3 operations are single-line MATLAB built-ins. The complexity is in the ORDERING of operations (conversion first, metrics before truncation, backdate to window entry) not in the computation itself.

---

## Common Pitfalls

### Pitfall 1: Swingup Time Off-By-One (Crossing vs. Hold Entry)
**What goes wrong:** `swingup_time = t(find(in_band, 1))` finds the first sample where q2 enters the tolerance band — but this may be a brief excursion that doesn't sustain for 1 second. The reported time is wrong (too early) and does not satisfy D-04.
**Why it happens:** D-04 specifies backdating to the entry of the FIRST SUSTAINED hold. This requires the sliding-window pattern, not `find`.
**How to avoid:** Use `first_sustained_idx` (Pattern 1). The returned index is already the start of the qualifying window.
**Warning signs:** Swingup time reported for an attempt where the pendulum only briefly touches ±pi.

### Pitfall 2: SMAPE Blow-Up Near t=0
**What goes wrong:** Near t=0, both hw_q2 and sim_q2 start from rest (q2 ≈ 0 = hanging-down position). The denominator `(|hw| + |sim|)/2` is near zero, producing SMAPE values of 1000%+ that dominate the mean.
**Why it happens:** The SMAPE formula has a mathematical singularity when both signals are near zero — which is guaranteed at the start of every attempt.
**How to avoid:** Apply the epsilon mask (D-13). Recommended epsilon = 1e-3 rad. Samples with `(|hw| + |sim|)/2 < epsilon` are excluded from the mean.
**Warning signs:** SMAPE > 200% on an attempt that visually looks well-matched.

### Pitfall 3: Wrong Scale Factor for Unit Conversion
**What goes wrong:** Data stored in 'rev' should multiply by `2*pi` to get radians. If the conversion is reversed (divide by `2*pi`) or the wrong unit is assumed, all thresholds are wrong by a factor of 6+.
**Why it happens:** The per-file `q2_unit` is set interactively and varies between teams/files.
**How to avoid:** Always read `q2_unit` from `attempt.signals.q2_unit`, not from `cfg.q2_unit` (which may have been updated by the most recent listdlg). In `compute_metrics`, the third argument `q2_unit` should come from `attempt.signals.q2_unit`.
**Warning signs:** Participation threshold never triggered even for strong swingups, or triggered on noise.

### Pitfall 4: `conv` Index Offset for Sustained-Hold Backdating
**What goes wrong:** `conv(mask, ones(1,N), 'valid')` returns a vector of length `numel(mask) - N + 1`. Index `win_idx` into this vector corresponds to original indices `win_idx : win_idx+N-1`. Confusing 'valid' with 'full' or 'same' shifts the reported time.
**Why it happens:** `conv` mode 'valid' starts from 1, so `win_idx = 1` maps correctly to `mask(1:N)`. No offset correction is needed with 'valid'. This is the correct mode.
**How to avoid:** Always use `conv(..., 'valid')`. Verify with a synthetic test: a mask of `[0 0 1 1 1 0]` with N=3 should return `win_idx=3` (hold starts at index 3).
**Warning signs:** Swingup time is 0.5s earlier or later than expected on a synthetic test.

### Pitfall 5: Data Shorter Than SMAPE Window
**What goes wrong:** If the attempt data is only 3 seconds long but `smape_fixed_duration = 5`, the `find(t >= t_win_end)` returns empty. The SMAPE window silently covers zero samples.
**Why it happens:** D-09 says "use available data" in this edge case, but a naive implementation might set `win_end_idx = []`.
**How to avoid:** If `win_end_idx` is empty, set it to `numel(t)`. This is the graceful degradation mandated by the context.
**Warning signs:** SMAPE returns NaN for a short attempt that should be eligible.

### Pitfall 6: Participation as Simple `any()` vs. 10-Consecutive-Sample Filter
**What goes wrong:** METR-02 explicitly mentions "10 consecutive samples" for noise robustness. Using `any(abs(hw_q2_rad) > pi/2)` would report participation for a single noise spike.
**Why it happens:** D-14 says "at any point" but the METR-02 acceptance criterion specifies the 10-sample requirement. These must be reconciled: D-14 describes the threshold value (pi/2), METR-02 describes the noise filter.
**How to avoid:** Use `first_sustained_idx(abs(hw_q2_rad) > pi/2, 10)` for the participation check. Participation = `~isempty(result)`. This satisfies both D-14 and METR-02.
**Warning signs:** Participation flag triggered on attempts where q2 never clearly exceeded pi/2 (only noise spikes).

---

## Code Examples

### Full `compute_metrics` Function Skeleton

```matlab
function metrics = compute_metrics(aligned, cfg, q2_unit)
%COMPUTE_METRICS Compute swingup metrics from pre-truncation aligned signals.
%   metrics = COMPUTE_METRICS(aligned, cfg, q2\_unit) converts q2 signals to
%   radians (D-01/D-02), then computes:
%     - participation flag (METR-02, D-14)
%     - swingup success flag and time (METR-01, METR-03, D-03/D-04/D-05)
%     - SMAPE with angular difference and window policy (METR-04/05/06/07, D-09)
%   Returns a flat metrics struct.

    % --- D-01/D-02: Convert to radians --------------------------------
    switch q2_unit
        case 'rev', scale = 2 * pi;
        case 'deg', scale = pi / 180;
        case 'rad', scale = 1;
        otherwise,  scale = 2 * pi;
    end
    hw_q2_rad  = aligned.hw_q2  * scale;
    sim_q2_rad = aligned.sim_q2 * scale;
    t          = aligned.t;

    % Derived constants
    tol_rad  = cfg.swingup_tolerance_deg * pi / 180;   % 2 deg = 0.0349 rad
    Ts_est   = median(diff(t));                         % actual sample period [s]
    N_hold   = round(cfg.swingup_hold_time / Ts_est);  % samples for 1s hold
    N_part   = 10;                                      % consecutive-sample filter
    epsilon  = 1e-3;                                    % denominator guard [rad]

    % --- METR-02 / D-14: Participation --------------------------------
    part_mask = abs(hw_q2_rad) > cfg.participation_threshold;
    part_idx  = first_sustained_idx(part_mask, N_part);
    metrics.participation = ~isempty(part_idx);

    % --- METR-01 / D-03/D-04/D-05: Swingup success & time ------------
    band_mask    = abs(abs(hw_q2_rad) - pi) < tol_rad;
    entry_idx    = first_sustained_idx(band_mask, N_hold);
    metrics.swingup_success = ~isempty(entry_idx);
    if metrics.swingup_success
        metrics.swingup_time = t(entry_idx);   % D-04: entry time of sustained hold
    else
        metrics.swingup_time = NaN;
    end

    % --- METR-04/05/06/07 + D-09/D-10/D-11: SMAPE --------------------
    thr_90 = pi / 2;
    idx_90 = find(abs(hw_q2_rad) > thr_90, 1, 'first');

    if isempty(idx_90)
        % D-10: never reached pi/2 -- ineligible
        metrics.smape_eligible = false;
        metrics.smape          = NaN;
        metrics.smape_window_s = NaN;
    else
        metrics.smape_eligible = true;
        t_win_end   = max(cfg.smape_fixed_duration, t(idx_90));  % D-09
        win_end_idx = find(t >= t_win_end, 1, 'first');
        if isempty(win_end_idx)
            win_end_idx = numel(t);   % edge case: data shorter than window
        end
        hw_win  = hw_q2_rad(1:win_end_idx);
        sim_win = sim_q2_rad(1:win_end_idx);
        metrics.smape          = compute_smape_angular(hw_win, sim_win, epsilon);
        metrics.smape_window_s = t(win_end_idx);
    end

    % --- D-08: One-liner diagnostic print ----------------------------
    if metrics.swingup_success
        t_str = sprintf('t=%.2fs', metrics.swingup_time);
    else
        t_str = 't=N/A';
    end
    if metrics.smape_eligible
        s_str = sprintf('SMAPE=%.1f%%', metrics.smape);
    else
        s_str = 'SMAPE=N/A';
    end
    fprintf('Metrics: swingup=%s %s %s participation=%s\n', ...
        yesno(metrics.swingup_success), t_str, s_str, ...
        yesno(metrics.participation));
end

function out = yesno(flag)
    if flag, out = 'YES'; else, out = 'NO'; end
end

function smape = compute_smape_angular(hw_rad, sim_rad, epsilon)
    num   = abs(mod(hw_rad - sim_rad + pi, 2*pi) - pi);
    denom = (abs(hw_rad) + abs(sim_rad)) / 2;
    valid = denom >= epsilon;
    if ~any(valid)
        smape = NaN;
        return
    end
    smape = mean(num(valid) ./ denom(valid)) * 100;
end

function idx = first_sustained_idx(mask, N)
    if N <= 1
        idx = find(mask, 1, 'first');
        return
    end
    counts  = conv(double(mask), ones(1, N), 'valid');
    win_idx = find(counts >= N, 1, 'first');
    idx     = win_idx;   % [] if not found; window start aligns with original index
end
```

### Session Loop Integration Point

```matlab
% --- Insert between align_signals and truncate_at_swingup (D-06) ---
aligned = align_signals(hw_t_cmd, hw_cmd_data, hw_q2_data, ...
                        sim_t_cmd, sim_cmd_data, sim_q2_data, delta);

% [NEW - Phase 3] Compute metrics on pre-truncation aligned data (D-07)
attempt.metrics = compute_metrics(aligned, cfg, file_q2_unit);

% Truncate signals 2s after swing-up (operates on aligned after metrics)
aligned = truncate_at_swingup(aligned, file_q2_unit, cfg.truncation_margin);
```

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Percentage-based SMAPE `|a-b|/((|a|+|b|)/2)` | Angular-difference SMAPE `|wrap(a-b)|/((|a|+|b|)/2)` | Required for ±pi-valued signals | Prevents false large errors at wrap boundary |
| Fixed-threshold crossing for swingup time | Sustained-hold with backdating | METR-01 requirement | More defensible: brief excursions excluded |
| Global cfg.q2_unit | Per-file q2_unit stored in attempt.signals.q2_unit | Phase 2 D-03 decision | Each file can use different display unit |

**Deprecated/outdated:**
- `timeseries` arithmetic for SMAPE: REQUIREMENTS.md (METR-04) explicitly says "use `interp1` for resampling, not timeseries arithmetic". Phase 2 already resampled sim onto hw grid in `align_signals`, so Phase 3 performs direct element-wise computation on the already-aligned vectors.

---

## Open Questions

1. **Epsilon value confirmation (D-13, Claude's Discretion)**
   - What we know: encoder resolution ~0.3 mrad/count; epsilon = 1e-3 rad is ~3× sensor noise floor
   - What's unclear: whether the instructor wants this exposed as a `cfg` parameter or kept internal
   - Recommendation: implement as a local constant inside `compute_metrics` for now; can be promoted to `cfg` in Phase 4 if needed

2. **Participation check: 10-consecutive vs. any() (D-14 vs. METR-02)**
   - What we know: D-14 says "at any point"; METR-02 says "10 consecutive samples"
   - What's unclear: context marks this as Claude's Discretion
   - Recommendation: implement with 10-consecutive-sample filter (METR-02 is the acceptance criterion; D-14 describes the threshold value, not the filter policy)

3. **`cfg.smape_window` field present but D-09 resolves to a specific policy**
   - What we know: `cfg.smape_window = 'fixed'` is set in the script; D-09 overrides the 3-mode design with a hybrid policy
   - What's unclear: should the planner update the cfg documentation comment, or keep the 'fixed'/'angle'/'swingup' flexibility for future use?
   - Recommendation: implement D-09 hybrid policy unconditionally (ignore `cfg.smape_window` for now); update the cfg comment to note this

---

## Environment Availability

Step 2.6: SKIPPED — Phase 3 is pure MATLAB array arithmetic with no external dependencies beyond the project's own script. All required functions (`mod`, `conv`, `find`, `median`, `interp1`) are MATLAB built-ins available in any R2025b installation.

---

## Validation Architecture

Step 4: nyquist_validation is explicitly set to `false` in `.planning/config.json`. Section omitted per workflow configuration.

---

## Project Constraints (from CLAUDE.md)

| Directive | Constraint |
|-----------|------------|
| Live Script `.m` format | New code in `compute_metrics` must use `%[text]` for documentation blocks, `%%` for section breaks, `%[appendix]{"version":"1.0"}` at end |
| MATLAB R2025b | Use R2025b-compatible syntax only; no features from later releases |
| `compute_metrics` lives in `score_competition.m` | Do NOT create a separate `compute_metrics.m` file; local functions only |
| Project open check | Not applicable to `compute_metrics` (local function, not a standalone script) |
| No new files without project registration | Phase 3 adds only a local function — no new `.m` files to register |
| `_num` suffix for numeric parameters | Not applicable to this function (parameters come from `cfg` struct, not standalone variables) |
| 4-space indentation, 80-100 char soft limit | Apply to all new code |

---

## Sources

### Primary (HIGH confidence)
- MATLAB built-in documentation — `conv`, `mod`, `find`, `median`, `interp1` behavior verified by direct inspection of existing script usage patterns
- `scripts/score_competition.m` lines 532–604 — `align_signals` output struct layout (field names, units, conventions)
- `scripts/score_competition.m` lines 786–824 — `truncate_at_swingup` pattern for threshold-based index detection (reusable logic for METR-01)
- `.planning/phases/03-metric-computation/03-CONTEXT.md` — All locked decisions verified directly

### Secondary (MEDIUM confidence)
- METR-02 requirement text (10 consecutive samples) cross-referenced with D-14 (participation threshold) to resolve the participation check implementation
- SMAPE angular formula `mod(a-b+pi, 2*pi)-pi` verified against known identity `atan2(sin(a-b), cos(a-b))` — identical for scalar angles, identical element-wise on vectors

### Tertiary (LOW confidence)
- None — all findings are verifiable from existing code and project decisions

---

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — no external libraries; all pure MATLAB
- Architecture: HIGH — local function pattern matches existing code throughout script
- Pitfalls: HIGH — derived from inspection of existing code + standard MATLAB numerical gotchas
- Code examples: HIGH — synthesized from locked decisions; directly traceable to CONTEXT.md decision IDs

**Research date:** 2026-04-06
**Valid until:** Stable indefinitely (no external APIs; MATLAB built-in behavior does not change across minor releases)
