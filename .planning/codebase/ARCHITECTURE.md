# Architecture

**Analysis Date:** 2026-04-02

## Pattern Overview

**Overall:** Modular control pipeline with symbolic derivation, numerical design, and hardware deployment layers

**Key Characteristics:**
- Clear execution pipeline: symbolic EOM derivation → parameter substitution → controller/observer design → Simulink simulation → code generation
- Single model per control strategy (one `.slx` per algorithm: FSFB, swing-up, UKF observer)
- Shared hardware interface library (`resources/lib/digtwin_labo_lib.slx`) decouples model development from embedded constraints
- `.mat` file convention for design artifact traceability: all design files follow `.par` (codegen-safe), `.meta` (traceability), `.mlobj` (MATLAB objects) struct pattern
- Two-DOF pendulum dynamics modeled via Euler-Lagrange with Denavit-Hartenberg kinematics

## Layers

**Symbolic Derivation Layer:**
- Purpose: Derive and cache continuous-time nonlinear dynamics using Symbolic Math Toolbox
- Location: `scripts/RRpendulum_forkin_dyn_noimage.m`
- Contains: Forward kinematics (DH transformations), kinetic/potential energy, Euler-Lagrange EOM derivation
- Depends on: `resources/functions/DH_full.m`, `resources/functions/derive_EOM.m`
- Used by: All downstream design and control scripts
- Outputs: `data/RRpendulum_EOM.mat` (cached symbolic equations, linearization matrices A/B)

**Parameter Definition Layer:**
- Purpose: Single source of truth for physical constants and hardware specifications
- Location: `scripts/RRpendulum_Parameters_num_BLDC.m` (primary), `scripts/stepper/RRpendulum_Parameters_num.m` (minimal variant)
- Contains: Mechanism parameters (mass, length, damping), actuation limits, encoder resolution, initial conditions
- Depends on: None (pure data)
- Used by: All control design scripts
- Outputs: `data/RRpendulum_params_BLDC.mat` (hierarchical struct: `.mech`, `.act`, `.sens`, `.ic`)

**Control Design Layer:**
- Purpose: Synthesize controller/observer gains using Control System Toolbox and custom algorithms
- Locations:
  - `scripts/RRpendulum_FSFB_controldesign_torque.m` — LQR full state feedback (continuous linearization → discrete via ZOH)
  - `scripts/RRpendulum_UKF_design.m` — Unscented Kalman Filter for state estimation (encoder-only measurements)
  - `scripts/RRpendulum_swingup_controldesign.m` — Energy-based swing-up (Astrom-Furuta) with hybrid catch modes
  - `scripts/RRpendulum_BLDC_drive_params.m` — Motor/drive parameter extraction from STM32 MCSDK headers
- Contains: Linearization around inverted equilibrium, LQR gain computation, observer tuning, energy controller design
- Depends on: `data/RRpendulum_EOM.mat`, `data/RRpendulum_params_BLDC.mat`
- Used by: Simulink models (via loaded `.mat` files)
- Outputs: Design `.mat` files with `.par` (doubles only, codegen-safe), `.meta` (timestamps, descriptions), `.mlobj` (Control System objects)

**Dynamics Computation Layer:**
- Purpose: Provide numerical ODE evaluation for simulation and observer propagation
- Location: `resources/functions/RRpendulum_dynamics_ct.m` (auto-generated from symbolic derivation)
- Contains: Continuous-time state derivative `xdot = f(x, u)` in explicit form (generated via `matlabFunction`)
- Depends on: State vector `z = [q1; v1; q2; v2]` and control input `tau_1`
- Used by: UKF observer (via `rk4_step`), Simulink S-functions, numerical integration
- Pattern: Auto-generated numeric form avoids symbolic overhead during simulation/codegen

**Observer & Estimation Layer:**
- Purpose: Estimate pendulum state from encoder-only measurements
- Location: `resources/functions/ukf_observer_step.m`
- Contains: Unscented Kalman Filter predict+update cycle with hardened Cholesky repair, Joseph-form covariance update, NaN firewall
- Depends on: `resources/functions/RRpendulum_dynamics_ct.m` (via `rk4_step` for RK4 propagation)
- Used by: Simulink controller models, real-time embedded control
- Design pattern: RK4 discretization of continuous dynamics, diagonal-loading Cholesky for numerical robustness

**Simulation & Code Generation Layer:**
- Purpose: Test control strategies via Simulink and generate deployable C code
- Locations: `models/stepper/RRpendulum_digtwin_FSFB.slx` (kinematic), `models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx` (primary with motor model)
- Contains: State feedback control blocks, observer blocks, saturation logic, SPI interface to MCU
- Depends on: Design `.mat` files (loaded as `Parameter` scope variables in MATLAB Function blocks)
- Used by: Simulink codegen for embedded deployment
- Deployment target: Raspberry Pi (SPI to STM32F401 MCU at 2 MHz, 1 kHz control loop)

**Hardware Interface Layer:**
- Purpose: Decouple high-level control from platform-specific SPI communication
- Location: `resources/lib/digtwin_labo_lib.slx`
- Contains: SPI protocol handling, torque command serialization (milli-Nm), encoder reading (counts → rad conversion)
- Depends on: STM32 firmware (`invpend_BLDC`) for SPI message structure
- Used by: All Simulink models requiring real hardware
- Critical constraint: Must stay synchronized with MCU firmware; changes require firmware coordination

## Data Flow

**Symbolic Derivation to Design:**

1. `RRpendulum_forkin_dyn_noimage.m` runs:
   - Defines symbolic variables (angles q, velocities v, parameters m, l, r, g, b_1, b_2, Iz_1)
   - Computes DH forward kinematics → Jacobian → kinetic + potential energy
   - Applies Euler-Lagrange (via `derive_EOM.m`) with viscous friction (Q = [tau_1 - b_1*q_dot_1; -b_2*q_dot_2])
   - Generates symbolic nonlinear EOMs: `z_dot = f(z, u)`
   - Computes Jacobians A = ∂f/∂z, B = ∂f/∂u for linearization
   - Saves everything to `data/RRpendulum_EOM.mat`

2. Design scripts load EOM and params:
   - `RRpendulum_FSFB_controldesign_torque.m`: Linearizes around z_eq = [0; 0; π; 0] (inverted), designs discrete LQR
   - `RRpendulum_UKF_design.m`: Tunes observer covariances Q, R for state estimation
   - Each saves design struct with `.par` (numbers only) and `.meta` (strings, timestamps)

3. Simulink models load design files:
   - MATLAB Function blocks declare `Parameter` scope variables pointing to loaded design struct fields
   - `K`, `Kd`, `Nbar` feedback gains passed directly; no recalculation
   - UKF parameters (Q, R, Wm, Wc) embedded for observer execution

**Runtime Control Flow (Embedded):**

1. Raspberry Pi SPI read encoder counts (q1, q2) → convert to radians via `q1_cpt`, `q2_cpt`
2. UKF observer predict: integrate dynamics via RK4 over sample time Ts
3. UKF observer update: measurement correction with encoder measurements
4. Full state feedback: `u = -Kd * x_hat + Nbar * q1_ref`
5. Saturation: clamp torque to ±u_sat
6. SPI write torque command (milli-Nm) to STM32; MCU converts to Iq current, executes FOC @ 16 kHz

**State Management:**

- **State representation:** Always `z = [q1; v1; q2; v2]` across all scripts/models/functions (one unified convention)
  - q1: horizontal joint angle (rad), driven
  - v1: horizontal joint velocity (rad/s)
  - q2: pendulum angle (rad), free; q2 = π is downward rest
  - v2: pendulum angular velocity (rad/s)
- **State storage:** In `.mat` files within design structs (e.g., `ukf.par.ic` for initial UKF state covariance P0)
- **State propagation:** Continuous-time ODE via `RRpendulum_dynamics_ct.m`, discretized via RK4 in observer

## Key Abstractions

**Euler-Lagrange Formulation:**
- Purpose: Derive nonlinear dynamics from energy principles without ad-hoc force balance
- Examples: `scripts/RRpendulum_forkin_dyn_noimage.m` (symbolic), `resources/functions/derive_EOM.m` (implementation)
- Pattern: L = T - V, then d/dt(∂L/∂q_dot) - ∂L/∂q = Q (generalized forces including friction)
- Enables: Symbolic linearization, validation via energy conservation, easy parameter substitution

**Denavit-Hartenberg Forward Kinematics:**
- Purpose: Map joint angles to end-effector position via standard SE(3) transformations
- Examples: `resources/functions/DH_full.m` (core), used in `RRpendulum_forkin_dyn_noimage.m`
- Pattern: Cumulative homogeneous transforms T0→1, T1→2, output position p = T_full * [0; 0; -l; 1]
- Benefit: Modular joint addition, easy Jacobian computation

**LQR Discrete Design Pattern:**
- Purpose: Synthesize linear feedback gains for inverted position stabilization
- Location: `scripts/RRpendulum_FSFB_controldesign_torque.m`
- Pattern:
  1. Linearize nonlinear EOM around equilibrium z_eq = [0; 0; π; 0]
  2. Form continuous ss: sys = ss(A_lin_num, B_lin_num, C_num, 0)
  3. Discretize via ZOH: sysd = c2d(sys, Ts)
  4. LQR gain: Kd = lqrd(A_lin, B_lin, Q, R, Ts) with Q = diag([100, 1, 10, 1]), R = 50000
  5. Feedforward: Nbar = -1/(C*((Ad-eye-Bd*Kd)^-1)*Bd) for q1 tracking
- Output: K (continuous), Kd (discrete), Nbar (feedforward gain)

**Unscented Kalman Filter (UKF):**
- Purpose: Estimate 4D state from 2D encoder measurements (q1, q2 only)
- Location: `resources/functions/ukf_observer_step.m`
- Pattern:
  1. **Predict:** Generate 9 sigma points (2*nx+1 = 2*4+1), propagate via RK4 over Ts
  2. **Update:** Measurement prediction Y_pred = C*X_pred, Joseph-form covariance update I-K*C
  3. **Robustness:** Diagonal-loading Cholesky repair, symmetry enforcement, NaN firewall (reset on numerical failure)
  4. **Codegen hardening:** No symbolic objects, explicit loop unrolling, state bounds (P_floor, velocity clamps)
- Design choices:
  - Encoder quantization noise: R = π²/(3*N²) per channel (from quantization theory)
  - Process noise acceleration model: Q tunable for ua1, ua2 (acceleration variance)
  - Wan-Merwe sigma points with alpha = 0.5 (avoids extreme weights)

**Energy-Based Swing-Up Control:**
- Purpose: Swing pendulum from rest (q2 = π) to inverted (q2 = 0) via energy feedback, then catch with FSFB
- Location: `scripts/RRpendulum_swingup_controldesign.m`, `resources/functions/swingup_energy_controller.m`
- Pattern: Three modes
  1. **Mode 0 (Swing-up):** Pump energy toward E0 = m*g*l via torque τ = sgn(v2)*|K_energy| (energy feedback)
  2. **Mode 1 (Stabilize):** Partial FSFB (q2, v1, v2 only; no q1) waits for energy to settle below E_latch for N_dwell samples
  3. **Mode 2 (Track):** Full FSFB with linear q1 ramp to desired position
- Reference: Astrom & Furuta (2000)

**Design Struct Convention (.par/.meta/.mlobj):**
- Purpose: Enable traceability and codegen-safe parameter passing
- Pattern: All design files save struct with three fields
  - `.par`: Doubles, logicals, nested structs of doubles only (codegen-safe; pass directly to MATLAB Function block Parameter scope)
  - `.meta`: Strings, timestamps, script name, description (metadata for audit trail)
  - `.mlobj`: MATLAB objects (`ss`, `datetime`, etc.) useful in MATLAB but excluded from codegen
- Example (from `FSFB_torque_design.mat`):
  ```
  design.par.lqr.K      % continuous gain
  design.par.lqr.Kd     % discrete gain
  design.par.lqr.Nbar   % feedforward
  design.par.plant.A    % linearized A matrix
  design.par.plant.B    % linearized B matrix
  design.meta.script    % "RRpendulum_FSFB_controldesign_torque.m"
  design.meta.timestamp % datetime
  design.mlobj.plant    % ss(A, B, C, D) object
  ```

## Entry Points

**Script Entry (Initialization & Design):**
- Location: `scripts/RRpendulum_forkin_dyn_noimage.m`
- Triggers: User runs this first to derive and cache symbolic EOM
- Responsibilities: Symbolic kinematics/dynamics derivation, Jacobian computation, caching to `data/RRpendulum_EOM.mat`

**Design Entry (Controller Synthesis):**
- Location: `scripts/RRpendulum_FSFB_controldesign_torque.m`, `RRpendulum_UKF_design.m`, etc.
- Triggers: After symbolic derivation; run in sequence for multiple design artifacts
- Responsibilities: Load cached EOM/params, compute gains/observer parameters, save to `.mat`

**Simulation Entry (Testing):**
- Location: `models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx` (primary model)
- Triggers: Open in Simulink, load design files into workspace, run `sim('RRpendulum_digtwin_FSFB_BLDC')`
- Responsibilities: Execute discrete-time control loop with observer, evaluate closed-loop behavior

**Code Generation Entry (Deployment):**
- Location: Simulink model + Embedded Coder configuration
- Triggers: `slbuild('RRpendulum_digtwin_FSFB_BLDC')` from MATLAB command line
- Responsibilities: Verify hardware interface (SPI block), generate deployable C code to `codegen/<modelname>_ert_rtw/`

## Error Handling

**Strategy:** Defensive programming for numerical robustness in observer and codegen contexts

**Patterns:**

1. **NaN/Inf Firewall (UKF):**
   - Location: `resources/functions/ukf_observer_step.m`, line 33-37
   - Implementation: Check `~isfinite(x_hat)` before propagation; reset to safe state if failed
   - Rationale: Embedded codegen cannot halt on NaN; must fail gracefully to safe state (pendulum hanging down)

2. **Cholesky Repair (UKF):**
   - Location: `resources/functions/ukf_observer_step.m`, line 42-50
   - Implementation: Attempt Cholesky; if fails, add diagonal loading (1e-8*I) and retry; fallback to scaled identity
   - Rationale: Covariance loss-of-positive-definiteness can occur from numerical drift; repair maintains stability

3. **Symmetry Enforcement (UKF):**
   - Location: `resources/functions/ukf_observer_step.m`, lines 41, 75, 97
   - Implementation: `P_new = 0.5*(P_new + P_new')` after covariance updates
   - Rationale: Floating-point roundoff can break symmetry; explicit enforcement ensures numerical consistency

4. **Parameter Validation (Design Scripts):**
   - Pattern: Control design scripts check controllability/observability via `rank(ctrb(...))`, `rank(obsv(...))`
   - Location: `scripts/RRpendulum_FSFB_controldesign_torque.m`, lines 71-72
   - Raises error if rank < 4 (system not fully controllable/observable)

5. **Saturation Logic (Control):**
   - Implementation: Torque command clipped to ±u_sat (0.2 Nm) in Simulink saturation block
   - Rationale: Motor/inverter cannot exceed 0.2 Nm; prevents integrator windup

## Cross-Cutting Concerns

**Logging:** Console output via `fprintf` in design scripts for audit trail (script completion, file save locations). No persistent logging framework; results stored in `.mat` for reproducibility.

**Validation:** Parameter range checks in design scripts (e.g., mass, length > 0; damping ≥ 0). Observer tuning via covariance analysis (P initial, innovation statistics). LQR stability verified via closed-loop pole inspection.

**Authentication:** None (standalone MATLAB research tool). Hardware interface relies on STM32 firmware trust.

**Configuration Management:** All runtime parameters loaded from `.mat` files (design artifacts) to avoid hardcoding. Design scripts source physical constants from `RRpendulum_params_BLDC.mat` (single source of truth).

**Synchronization:** Hardware interface library (`resources/lib/digtwin_labo_lib.slx`) must be manually synchronized with STM32 firmware SPI message structure; breaking changes flagged in git history and CLAUDE.md.

---

*Architecture analysis: 2026-04-02*
