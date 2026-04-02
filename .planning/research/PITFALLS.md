# Domain Pitfalls: MATLAB Swingup Competition Scoring

**Domain:** MATLAB scoring script — SDI file parsing, signal alignment, SMAPE, swingup detection
**Researched:** 2026-04-02
**Overall confidence:** HIGH (SDI API behavior from official MathWorks docs; SMAPE math from first principles; angle/time issues from MATLAB documentation and community)

---

## Critical Pitfalls

Mistakes that produce wrong scores or silent wrong answers.

---

### Pitfall 1: Simulink.sdi.load Triggers a GUI Dialog Without a Clear-First

**What goes wrong:**
`Simulink.sdi.load('file.mldatx')` without first calling `Simulink.sdi.clear` will trigger an interactive dialog asking "Clear existing data or Append?" This blocks the script in non-interactive (batch) execution and produces undefined behavior when called in a loop over multiple teams — runs from different teams bleed into the same session.

**Why it happens:**
SDI persists its session state across script invocations. The load function only clears when explicitly instructed or when the user selects "Clear" in the dialog.

**Consequences:**
- `Simulink.sdi.getAllRunIDs` returns IDs from multiple teams' files mixed together.
- Run indexing (first run = hardware, second run = simulation) breaks silently.
- No error is raised; wrong data is processed as if it were correct.

**Prevention:**
Always call `Simulink.sdi.clear` immediately before `Simulink.sdi.load` in any automated script:
```matlab
Simulink.sdi.clear
Simulink.sdi.load('team_file.mldatx')
```

**Detection (warning sign):**
`Simulink.sdi.getRunCount` returning more than 2 after loading a single `.mldatx` that should contain exactly 2 runs. Assert this invariant at load time.

**Phase:** Phase 1 (SDI loading infrastructure)

---

### Pitfall 2: Archive vs. Recent Run Order Is Not Guaranteed by Position

**What goes wrong:**
The assumption that run index 1 = hardware (archived) and run index 2 = simulation (recent) is fragile. The ordering returned by `Simulink.sdi.getAllRunIDs` depends on the `setAppendRunToTop` setting at the time the file was saved. Different teams may have different SDI configurations. A team that ran simulation first then hardware, or who re-ran simulation, will have a different ordering.

**Why it happens:**
SDI run ordering is a display preference (`setAppendRunToTop`), not a semantic property. There is no dedicated `isArchived` property on `Simulink.sdi.Run` that reliably marks archive vs. work-area membership in all MATLAB versions.

**Consequences:**
Hardware and simulation signals are swapped. Time alignment is computed on the wrong pair. SMAPE is computed on noise or the wrong channel.

**Prevention:**
Identify runs by content, not position. Use `run.Description` and `run.Name` properties (set by Simulink when archiving) to identify which run is archived. Alternatively, check signal characteristics: hardware runs at 2 kHz produce signal vectors with ~2000× more samples per second than simulation runs at a coarser Ts. Present the scorer with run names and sample counts for manual confirmation before proceeding.

**Detection:**
Check `run.NumSignals` and the time span / sample count of each run. The 2 kHz hardware run will have ~2000 samples/second; simulation runs at typical Ts values (1/1000 or 1/100) will have different densities.

**Phase:** Phase 1 (SDI loading), Phase 2 (signal mapping UI)

---

### Pitfall 3: SMAPE Division by Zero When Both Signals Are Near Zero

**What goes wrong:**
The standard SMAPE formula `2*|a-b| / (|a| + |b|)` is undefined at every sample where both `a` and `b` are exactly or nearly zero. For the pendulum angle q2, this happens at the start of the run when the pendulum is hanging at rest (q2 ≈ 0). These samples generate `0/0 = NaN` (or `Inf` if only one is zero), which silently propagates into the mean and produces `NaN` for the entire SMAPE score.

**Why it happens:**
MATLAB's division by zero produces `NaN` silently for double arithmetic. If the window includes pre-swingup samples, many denominator values are near zero.

**Consequences:**
SMAPE returns `NaN` for all teams whose windows include the resting phase. Ranking is impossible from `NaN` values.

**Prevention:**
Three-part defense:
1. Restrict the SMAPE window to after the detected start time (first non-zero accel/torque command), eliminating the resting samples.
2. Add a per-sample guard: exclude samples where `(|a| + |b|) < epsilon` (e.g., `epsilon = 1e-3` rad) from the mean rather than dividing.
3. Assert that final SMAPE is finite before using it in scoring.

```matlab
denom = abs(hw) + abs(sim);
valid = denom > 1e-3;
smape = mean(2 * abs(hw(valid) - sim(valid)) ./ denom(valid)) * 100;
```

**Detection:**
`isnan(smape)` or `isinf(smape)` after computation. Always check.

**Phase:** Phase 3 (SMAPE computation)

---

### Pitfall 4: Angle Wrapping Discontinuity Near q2 = ±π Corrupts SMAPE

**What goes wrong:**
The pendulum angle q2 uses the convention: 0 = hanging down, ±π = inverted upright. In hardware, the sensor may report the upright position as +π. In simulation (or a different run), it may report -π. These are physically identical, but `|π - (-π)| = 2π ≈ 6.28` rad — giving a massive false SMAPE error during the exact window being scored (the upright stabilization phase).

Additionally, the derivative near ±π produces phantom spikes: a signal oscillating by ±0.01 rad around π will alternately read +3.13 and -3.13, making the time series look like a 6.26 rad oscillation.

**Why it happens:**
MATLAB's encoder decoding returns values in `(-π, π]`. Hardware and simulation may independently wrap to different sides. `wrapToPi` is not idempotent across the comparison pair.

**Consequences:**
SMAPE is inflated by up to 2π per sample during the stabilization window. A team that achieves excellent physical stabilization gets a worse score than a team with noisier stabilization at a different angular sign.

**Prevention:**
Before computing SMAPE, unwrap both signals to a continuous representation, then re-align to a common reference angle:
```matlab
hw_unwrapped  = unwrap(hw_q2);
sim_unwrapped = unwrap(sim_q2);
% Re-center: remove mean offset after alignment window
offset = mean(hw_unwrapped - sim_unwrapped);
sim_unwrapped = sim_unwrapped + offset;
```
Alternatively, compute the angular error as `mod(hw - sim + pi, 2*pi) - pi` (wraps difference to `(-π, π]`) per sample before taking absolute value.

**Detection:**
Plot both raw signals before computing SMAPE. Any step discontinuity of magnitude ≈ 2π indicates unhandled wrapping.

**Phase:** Phase 3 (SMAPE computation), Phase 2 (signal validation plots)

---

### Pitfall 5: Time Alignment Offset Carries Over Into All Derived Metrics

**What goes wrong:**
The start time (first non-zero accel/torque command) is used to align hardware and simulation signals. If this reference point is identified with even 1 sample error (0.5 ms at 2 kHz), the SMAPE window shifts by one sample. For a slow simulation signal (e.g., Ts = 5 ms), a 1-sample hardware error becomes a 10-sample relative misalignment — introducing a systematic phase error into SMAPE.

A more dangerous variant: if the accel/torque command signal has small non-zero values at rest due to sensor noise, quantization, or a non-zero initial condition, the "first non-zero" detection fires too early and the entire alignment is off by the noise duration.

**Why it happens:**
Finding the "first non-zero" value requires a threshold, not `> 0`. The correct threshold is team-specific (depends on their actuator command range).

**Consequences:**
SMAPE is systematically wrong for all teams. Swingup time is wrong. Rankings may swap between teams that are genuinely close.

**Prevention:**
Use a hysteresis-style threshold: the start event is the first sample where `|cmd| > alpha_threshold` for at least `N_min` consecutive samples (e.g., `N_min = 5`). This rejects single-sample noise spikes. Make `alpha_threshold` configurable per team type (stepper vs. BLDC have different command scales).

```matlab
threshold = 0.05 * max(abs(cmd));  % 5% of peak command
sustained = movmean(abs(cmd) > threshold, N_min) >= 1;
t_start_idx = find(sustained, 1, 'first');
```

**Detection:**
Visually inspect the aligned signals at t=0. The simulation and hardware q2 traces should start from the same initial angle. If they start offset in angle, alignment is wrong.

**Phase:** Phase 2 (time alignment), Phase 3 (metric computation)

---

## Moderate Pitfalls

---

### Pitfall 6: interp1 Returns NaN for Out-of-Range Query Points

**What goes wrong:**
When resampling simulation signal onto hardware time grid (or vice versa), `interp1` by default returns `NaN` for any query point outside the simulation's time span. If the hardware run is 2 seconds longer than the simulation (the team stopped the simulation early), the tail of the SMAPE window becomes all `NaN` and the mean silently drops those samples — or goes fully `NaN`.

**Prevention:**
Clip the SMAPE computation window to the intersection of both signals' time spans:
```matlab
t_start = max(t_hw(1),   t_sim(1));
t_end   = min(t_hw(end), t_sim(end));
t_common = t_hw(t_hw >= t_start & t_hw <= t_end);
```
Assert that `t_common` is non-empty and spans at least the required window duration.

**Detection:**
After interpolation, `any(isnan(sim_resampled))` warns of truncation.

**Phase:** Phase 3 (SMAPE computation)

---

### Pitfall 7: Multiple π-Crossings Produce Ambiguous Swingup Time

**What goes wrong:**
The swingup time is defined as "first time q2 crosses ±π". In a failed attempt, the pendulum may reach near-vertical and then fall back, producing a brief crossing. A noisy encoder may chatter at the upright position, producing dozens of crossings within milliseconds. Using `find(abs(q2) > pi*0.99, 1)` gives the first such instant, which may be a transient and not the true sustained upright.

The requirement is that q2 stays within ±2° of ±π for at least 1 second. The crossing time and the sustained-upright time are different events. Conflating them gives a swingup time that predates the actual catch.

**Prevention:**
Separate detection into two phases:
1. Find first `|q2| > pi - deg2rad(2)` — this is the crossing candidate.
2. Verify that from this point, `|q2| > pi - deg2rad(2)` holds for a continuous 1-second window.
3. If the 1-second window test fails, advance past the current crossing and search again.

```matlab
upright_threshold = pi - deg2rad(2);
in_zone = abs(q2) >= upright_threshold;
% Find first index where in_zone stays true for >= 2000 samples (1s @ 2kHz)
t_swingup = find_sustained_crossing(t_hw, in_zone, 2000);
```

**Detection:**
Plot the crossings. Count how many times the signal enters and exits the upright zone before settling.

**Phase:** Phase 2 (swingup detection)

---

### Pitfall 8: ZOH vs. Linear Interpolation Mismatch for Discrete Signals

**What goes wrong:**
Hardware signals at 2 kHz are zero-order-hold (ZOH) in nature: the MCU outputs a constant command between updates. Simulation signals at a coarser Ts are also ZOH. If `interp1` with `'linear'` method is used for resampling, it introduces a linear ramp between samples that never existed in the physical system. This inflates SMAPE around step transitions (e.g., accel command switching polarity during swingup).

**Prevention:**
Use `'previous'` interpolation method for ZOH signals (command outputs):
```matlab
cmd_resampled = interp1(t_sim, cmd_sim, t_hw, 'previous', 'extrap');
```
For smooth state signals (q2, v2), linear interpolation is appropriate.

**Detection:**
Visually compare original and resampled signals on the same plot. A ZOH signal resampled with `'linear'` will show characteristic triangular ripple at transition edges.

**Phase:** Phase 3 (SMAPE computation)

---

### Pitfall 9: timeseries Object Arithmetic Silently Resamples

**What goes wrong:**
MATLAB `timeseries` objects perform automatic resampling when you apply arithmetic operators (`+`, `-`, `./`) between two `timeseries` objects with different time bases. The resampling uses `'linear'` interpolation by default. Code like `err_ts = hw_ts - sim_ts` appears to work but the result is on a merged (union) time grid, not the original hardware grid, and uses linear interpolation for the simulation values.

This is a particularly subtle pitfall because the result `err_ts` is valid-looking and produces no warning.

**Prevention:**
Never subtract `timeseries` objects directly for SMAPE. Instead, explicitly extract `Values` and `Time` arrays and use `interp1` with your chosen method and a controlled time grid:
```matlab
hw_vals  = hw_ts.Data;
hw_time  = hw_ts.Time;
sim_vals = interp1(sim_ts.Time, sim_ts.Data, hw_time, 'linear');
```

**Detection:**
Check `length(result_ts.Time)` against `length(hw_ts.Time)`. If they differ, automatic resampling has occurred.

**Phase:** Phase 3 (SMAPE computation)

---

### Pitfall 10: Signal Data Type Is single, Not double

**What goes wrong:**
Embedded Coder generates code using `single` precision floating point for the hardware target (ARM Cortex). The MCU logs data as `single`. When the SDI exports this signal via `signal.export()` → `timeseries`, the `Data` field is `single`. Arithmetic comparisons and SMAPE calculations in `double` will silently upcast the values, but if the code ever does explicit type checks or uses `typecast`, mismatches occur. More practically, `single` precision means the angular resolution near ±π is ~10⁻⁷ rad, which is fine, but `NaN` checks after division may behave differently from `double` NaN.

**Prevention:**
Explicitly cast extracted signal data to `double` immediately after export:
```matlab
ts = sig.export();
vals = double(ts.Data);
time = double(ts.Time);
```

**Detection:**
`class(ts.Data)` returns `'single'` for hardware-logged signals.

**Phase:** Phase 1 (SDI loading), Phase 3 (SMAPE computation)

---

## Minor Pitfalls

---

### Pitfall 11: unwrap Breaks on Oscillation at Exactly ±π

**What goes wrong:**
`unwrap` corrects phase jumps by assuming any change larger than π is a wrap artifact. If the pendulum truly oscillates around the upright position with an amplitude >π (a failed catch), `unwrap` will interpret the legitimate large oscillation as wrapping and produce a monotonically drifting signal that bears no resemblance to reality. The unwrapped signal continues accumulating offset forever.

**Prevention:**
Apply `unwrap` only within the stabilization window (after the ±π crossing is confirmed). Do not apply `unwrap` to the full run length. For the pre-swingup phase where q2 legitimately swings through ±π multiple times, compute SMAPE on the wrapped signal using the angular difference formula `mod(hw - sim + pi, 2*pi) - pi`.

**Phase:** Phase 3 (SMAPE computation)

---

### Pitfall 12: Participation Check Using Absolute Value of Wrapped Angle

**What goes wrong:**
The participation criterion is `|q2| > π/2`. If q2 is logged as a value near ±π and the encoder wraps to the other side during the measurement (e.g., reads 3.14 at one sample and -3.14 at the next), a simple `max(abs(q2)) > pi/2` test will always return true even if the pendulum never actually reached 90°. Conversely, a noisy signal that briefly dips below the threshold due to encoder jitter may fail a point-in-time check.

**Prevention:**
Use a brief sustained window for the participation check (e.g., at least 10 consecutive samples with `|q2| > π/2`). This rejects noise spikes while correctly accepting genuine crossings.

**Phase:** Phase 2 (swingup detection)

---

### Pitfall 13: Run Name / Description Contains Non-ASCII or Special Characters

**What goes wrong:**
Teams may name their Simulink model with special characters (umlauts, accented letters, spaces, parentheses). When the SDI logs the run name, these appear in `run.Name`. If the script uses `run.Name` in a file path (for output CSVs or log files) or in a `strcmp` comparison for team identification, non-ASCII characters cause `fopen` errors or mismatches on Windows.

**Prevention:**
Never use `run.Name` directly in file paths. Use team identifier strings provided by the scorer at the top of the script. Sanitize `run.Name` with `regexprep(name, '[^\w]', '_')` before any file path construction.

**Phase:** Phase 1 (SDI loading), Phase 4 (output/reporting)

---

### Pitfall 14: SMAPE Is Not Symmetric With Respect to the Scoring Direction

**What goes wrong:**
SMAPE is named "symmetric" but its interpretation is asymmetric when comparing hardware (ground truth) to simulation (prediction). The metric penalizes large simulation over-estimates and under-estimates differently in practice because the denominator depends on the sum of both magnitudes. When hardware signal is near zero (pendulum still rising) and simulation overshoots, the SMAPE per sample can exceed 200%. Teams whose simulation is more aggressive than hardware are penalized disproportionately compared to teams whose simulation under-predicts.

This is a fairness concern rather than a bug, but it can lead to instructor disputes.

**Prevention:**
Document the exact SMAPE formula used in the scoring rubric. Consider providing the formula in the output report so teams can verify. Use the standard two-coefficient form: `smape = mean(2*|hw-sim| / (|hw| + |sim|)) * 100` and note the range is [0%, 200%] not [0%, 100%].

**Phase:** Phase 4 (output/reporting)

---

## Phase-Specific Warnings

| Phase Topic | Likely Pitfall | Mitigation |
|-------------|----------------|------------|
| SDI file loading loop | Session contamination across files (Pitfall 1) | `Simulink.sdi.clear` before each load |
| Run identification | Archive/recent ordering wrong (Pitfall 2) | Identify by signal density, not index |
| Signal data extraction | `single` precision from hardware runs (Pitfall 10) | Cast to `double` immediately |
| Time alignment / start detection | Noise fires start too early (Pitfall 5) | Hysteresis threshold, sustained window |
| Resampling for SMAPE | Linear interp on ZOH command signals (Pitfall 8) | Use `'previous'` for commands |
| Resampling for SMAPE | `timeseries` arithmetic auto-resamples (Pitfall 9) | Extract `.Data` and `.Time`, use `interp1` |
| Resampling for SMAPE | `interp1` returns NaN outside span (Pitfall 6) | Clip to intersection of time spans |
| SMAPE computation | Division by zero near q2=0 (Pitfall 3) | Exclude samples where denominator < epsilon |
| SMAPE computation | Angle wrapping inflates error at ±π (Pitfall 4) | Angular difference formula or unwrap+offset |
| SMAPE computation | `unwrap` breaks on large legitimate oscillations (Pitfall 11) | Apply `unwrap` only in stabilization window |
| Swingup detection | Multiple/spurious crossings (Pitfall 7) | Require sustained 1-second window |
| Participation check | Transient crossings / wrap artifacts (Pitfall 12) | Require 10-sample sustained crossing |
| Output reporting | Non-ASCII team names in file paths (Pitfall 13) | Sanitize names; use scorer-provided IDs |
| Score interpretation | SMAPE range misunderstood as 0-100% (Pitfall 14) | Document formula and range explicitly |

---

## Sources

- [Simulink.sdi.load — MathWorks documentation](https://www.mathworks.com/help/simulink/slref/simulink.sdi.load.html)
- [Simulink.sdi.clear — MathWorks documentation](https://www.mathworks.com/help/simulink/slref/simulink.sdi.clear.html)
- [Simulink.sdi.setAutoArchiveMode — MathWorks documentation](https://www.mathworks.com/help/simulink/slref/simulink.sdi.setautoarchivemode.html)
- [Simulink.sdi.setAppendRunToTop — MathWorks documentation](https://www.mathworks.com/help/simulink/slref/simulink.sdi.setappendruntotop.html)
- [Simulink.sdi.Signal export — MathWorks documentation](https://www.mathworks.com/help/simulink/slref/simulink.sdi.signal.export.html)
- [Simulink.sdi.Signal convertDataType — MathWorks documentation](https://www.mathworks.com/help/simulink/slref/simulink.sdi.signal.convertdatatype.html)
- [Inspect and Compare Data Programmatically — MathWorks](https://www.mathworks.com/help/simulink/ug/record-and-inspect-signal-data-programmatically.html)
- [timeseries resample — MathWorks documentation](https://www.mathworks.com/help/matlab/ref/timeseries.resample.html)
- [unwrap — MathWorks documentation](https://www.mathworks.com/help/matlab/ref/unwrap.html)
- [interp1 NaN out-of-range behavior — MathWorks Answers](https://www.mathworks.com/matlabcentral/answers/413891-how-to-use-inerpolation-option-with-end-values-if-i-use-interp1-command-it-s-giving-nan-as-the-outp)
- [Symmetric Mean Absolute Percentage Error — Wikipedia](https://en.wikipedia.org/wiki/Symmetric_mean_absolute_percentage_error)
- [The Symmetric Mean Absolute Percentage Error: Unnecessary or Dangerous — MDPI](https://www.mdpi.com/2571-9394/8/2/24)
- [Floating point comparison pitfalls — MATLAB BugFinder](https://www.mathworks.com/help/bugfinder/ref/floatingpointcomparisonwithequalityoperators.html)
