# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MATLAB/Simulink digital twin laboratory for a **rotary RR (2-DOF) inverted pendulum** (STEVAL-EDUKIT01). Covers the full pipeline: symbolic derivation of kinematics/dynamics, Simulink simulation, LQR controller design, embedded C code generation (Raspberry Pi / STM32), and real hardware deployment.

## Key Commands

```matlab
% Open the MATLAB project
openProject('digtwin_labo.prj')

% Symbolic derivation (run once per session, use .m not .mlx for programmatic runs)
run('scripts/RRpendulum_forkin_dyn_noimage.m')

% Load physical parameters into workspace
run('scripts/RRpendulum_Parameters_num.m')

% Kinematic control setup + simulation
run('scripts/RRpendulum_kinctrl_numericalsetup.m')
sim('RRpendulum_digtwin')

% Full state feedback controller design (computes K, Kd gains)
run('scripts/RRpendulum_FSFB_controldesign.m')

% Generate embedded C code from a Simulink model
slbuild('RRpendulum_digtwin')
```

## Architecture

### Execution Pipeline (order matters)
1. **Symbolic derivation** (`scripts/RRpendulum_forkin_dyn_noimage.m`) — DH kinematics + Euler-Lagrange EOMs using Symbolic Math Toolbox
2. **Parameter substitution** (`scripts/RRpendulum_Parameters_num.m`) — single source of truth for all physical constants
3. **Controller design** (e.g., `scripts/RRpendulum_FSFB_controldesign.m`) — LQR gains via Control System Toolbox
4. **Simulink simulation** (`models/*.slx`) — one model per control strategy
5. **Code generation** — Embedded Coder targeting ARM (Raspberry Pi SPI at 2 MHz → MCU)

### State Vector Convention
All scripts use: `z = [q1; v1; q2; v2]` where q=angle (rad), v=angular velocity (rad/s). q1=horizontal rotation (driven), q2=pendulum angle (free).

### Dual .m/.mlx Pattern
Several scripts exist as both `.mlx` (Live Script with embedded images/formatting) and `.m` (plain script for programmatic use). Use `.m` variants for automated/programmatic execution.

### One Model Per Control Strategy
Each control algorithm has a dedicated `.slx` model (e.g., `RRpendulum_digtwin_FSFB.slx` for state feedback, `RRpendulum_digtwin_swingup.slx` for swing-up). The shared hardware interface lives in `resources/lib/digtwin_labo_lib.slx`.

## Critical Files

- `scripts/RRpendulum_Parameters_num.m` — all physical parameters (mass, length, damping, saturation limits, sample time Ts=1/2000)
- `resources/lib/digtwin_labo_lib.slx` — hardware interface library; **must stay synchronized with MCU firmware**
- `resources/functions/DH_full.m` — Denavit-Hartenberg transformation computation
- `resources/functions/derive_EOM.m` — Euler-Lagrange equation derivation

## Toolbox Dependencies

Simulink, Symbolic Math Toolbox, Control System Toolbox, Embedded Coder, Simulink Coder. MATLAB R2025b or later.

## MATLAB MCP Usage Rule

Before running any MATLAB code via MCP, always ensure the project is open by prepending this idempotent check:

```matlab
if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
```

This guarantees all project folders (`scripts/`, `resources/functions/`, `models/`, etc.) are on the MATLAB path. The check is safe to call repeatedly — it won't re-open an already-loaded project.

## Git Commit Rule

Only commit files that are registered in the MATLAB project (`digtwin_labo.prj`). The project file is the single source of truth for which files belong to the repository. Before staging, query the project file list:

```matlab
prj = matlab.project.rootProject;
for i = 1:numel(prj.Files), disp(prj.Files(i).Path); end
```

Files **not** in the project (e.g., `data/`, `UI/`, `CLAUDE.md`, ad-hoc scripts) must stay untracked even if they exist in the working tree.

## Conventions

- Physical parameter variables end with `_num` suffix (e.g., `m_num`, `l_num`, `g_num`)
- Generated code lands in `codegen/<modelname>_ert_rtw/` (git-ignored)
- Binary MATLAB files (`.slx`, `.mlx`, `.mat`, `.mldatx`) use `merge=mlAutoMerge` in `.gitattributes`
- **New scripts must use the Live Script `.m` format**: use `%[text]` for rich text annotations (supports markdown bold `**...**` and LaTeX `$...$`), `%%` for section breaks, and `%[appendix]{"version":"1.0"}` at the end. Escape `_` as `\_` and `\` as `\\` inside `%[text]` lines. This renders as a Live Script in the MATLAB Live Editor while remaining a plain-text, diffable `.m` file.

<!-- GSD:project-start source:PROJECT.md -->
## Project

**Swingup Competition Scoring Script**

A MATLAB scoring tool for the digital twin swingup competition. It processes team submissions (`.mldatx` files from Simulink Data Inspector), validates successful swingups, computes swingup times and simulation accuracy (SMAPE), and produces a ranked leaderboard with plots. Used by the instructor to grade 5 teams (4 stepper + 1 BLDC) after the competition.

**Core Value:** Correctly and fairly score every team's swingup attempts — accurate time extraction, proper signal alignment, and transparent SMAPE computation — so grades are defensible.

### Constraints

- **Toolbox**: Must use Simulink Data Inspector API (`Simulink.sdi.*`) — this is how `.mldatx` files are structured
- **MATLAB version**: R2025b (current project version)
- **Existing project**: Script lives in the existing `digtwin_labo` project, under `scripts/`
- **Plain-text .m format**: New scripts must use Live Script `.m` format per project conventions
<!-- GSD:project-end -->

<!-- GSD:stack-start source:codebase/STACK.md -->
## Technology Stack

## Languages
- MATLAB - Core simulation, symbolic math, controller design, parameter scripts
- Simulink - Model-based design, simulation, and embedded code generation
- C/C++ - Generated embedded code (Embedded Coder output for Raspberry Pi and STM32)
- C Header Files - STM32 MCSDK firmware parameter definitions (parsed and integrated via `scripts/RRpendulum_BLDC_drive_params.m`)
## Runtime
- MATLAB R2025b or later
- MATLAB Project system (`digtwin_labo.prj`) - manages code organization and dependencies
- Lockfile: Project file `digtwin_labo.prj` (XML format)
## Frameworks
- Simulink - Version R2025b - Model-based simulation, signal flow, hardware interface library blocks
- Embedded Coder - Version R2025b - C code generation targeting ARM (Raspberry Pi, STM32)
- Simulink Coder (RTW/Real-Time Workshop) - Code generation back-end, ERT target (Embedded Real-Time)
- Symbolic Math Toolbox - Automated derivation of equations of motion (EOM), forward kinematics, Jacobians
- Control System Toolbox - LQR controller design, continuous-to-discrete conversion, system analysis
- Simulink Design Verifier - Reports in `sdireports/` (design verification, test generation)
- Embedded Coder (codegen) - Target: ERT (Embedded Real-Time), output in `codegen/[modelname]_ert_rtw/`
- Simulink Real-Time Workshop - Back-end for code generation
## Key Dependencies
- **Symbolic Math Toolbox** - Required for `RRpendulum_forkin_dyn_noimage.m` to derive equations of motion
- **Control System Toolbox** - Required for controller design scripts
- **Embedded Coder** - Required for C code generation
- **MATLAB Function blocks** in Simulink - for embedding custom numerical code
- **Simscape** - Optional, used in some variant models for DC motor electrical simulation
## Configuration
- MATLAB project configuration: `digtwin_labo.prj`
- MATLAB path: Automatically managed by `openProject()` to include `scripts/`, `resources/functions/`, `models/`, `data/`
- Sample time (hardware interface): `Ts = 1/1000` (1 kHz) for SPI communication with STM32 MCU
- Simulink model configurations (per `.slx` file):
- Single source of truth: `scripts/RRpendulum_Parameters_num_BLDC.m`
- All `.mat` design files follow a **`.par` / `.meta` / `.mlobj`** convention:
## Platform Requirements
- MATLAB R2025b or later
- Windows 11 or compatible (project currently on Windows Enterprise 10.0.22631)
- Toolboxes: Simulink, Symbolic Math Toolbox, Control System Toolbox, Embedded Coder, Simulink Coder
- Compiler: GCC/ARM cross-compiler for STM32/Raspberry Pi (required only for actual deployment)
- **Raspberry Pi** - Target for high-level control (LQR, UKF, swing-up) via Simulink Embedded Coder
- **STM32F401** - Motor control (Field-Oriented Control at 16 kHz) running ST MCSDK firmware
- **STEVAL-EDUKIT01** - STMicroelectronics educational 2-DOF rotary pendulum kit
## External Integrations / Hardware Interfaces
- STM32 Motor Control SDK (MCSDK) - Parameter extraction via C header parsing
- SPI interface library: `resources/lib/digtwin_labo_lib.slx`
<!-- GSD:stack-end -->

<!-- GSD:conventions-start source:CONVENTIONS.md -->
## Conventions

## Naming Patterns
- Script format: PascalCase with underscores, topic-descriptive names
- Auto-generated functions: Similar pattern with generation metadata
- Variant management: Live Script (`.mlx`) + plain `.m` pairs for programmatic use
- **Symbolic variables:** Lowercase with underscores, explicit `(t)` for time-dependent
- **Numeric parameters:** Lowercase ending with `_num` suffix (single source of truth convention)
- **States:** Compact vector notation
- **Matrices:** Uppercase for system matrices, lowercase for intermediate results
- Camel case with descriptive names, parameters in meaningful order
- Helper functions: Prefixed with lowercase, placed as nested functions
- Constraint: Parameter order convention: states → inputs → gains/tuning → physical parameters
- All variables are numeric (double precision) unless explicitly symbolic
- Symbolic Math Toolbox creates symbolic variables via `syms` declarations at script top
- No explicit type annotations (MATLAB infers types automatically)
## Code Style
- **Line length:** 80-100 character soft limit (evident in function signatures and comments)
- **Indentation:** 4 spaces (standard MATLAB convention)
- **Spacing:** 
- **Matrix notation:** Multi-line matrices align on `[` brackets
- No explicit linter configuration detected (`.eslintrc`, `.mlintrc` not present)
- Code adheres to MATLAB R2025b standards (no deprecated syntax)
- Auto-generated code (from `matlabFunction`) preserves Symbolic Math Toolbox conventions (temporary variables `t2`, `t3`, etc.)
## Import Organization
- All functions called without explicit import/namespace qualification
- Functions in `resources/functions/` added to path via MATLAB project
- Critical path check in CLAUDE.md enforces:
- `DH_full.m` → called by forward kinematics in `RRpendulum_forkin_dyn_noimage.m`
- `derive_EOM.m` → called to generate equations of motion (Lagrangian approach)
- `RRpendulum_dynamics_ct.m` → called by `ukf_observer_step.m` for state propagation
- `RRpendulum_totalIz1.m` → called by `swingup_energy_controller.m` for pose-dependent inertia
- `rk4_step` → nested inside `ukf_observer_step.m` for numerical integration
## Error Handling
- Explicit error checks in critical functions
- Auto-generated functions (e.g., `RRpendulum_dynamics_ct.m`) assume valid inputs (no checks)
- `ukf_observer_step.m` includes explicit robustness check (line 33):
- Covariance matrix symmetry enforced (line 41, 75, 97 in `ukf_observer_step.m`):
- Cholesky decomposition with fallback repair (lines 42-50):
- Covariance bounds enforce physical limits (lines 100-108):
- Control saturation implemented in `swingup_energy_controller.m` (lines 40-43):
## Logging
- **Script progress:** Section markers with `%[text]` annotations in Live Scripts
- **Validation output:** `disp()` calls to confirm matrix shapes and values
- **No runtime logging:** No log file generation or timestamped output detected
## Comments
- **Above function definitions:** Full docstring in help format (MATLAB standard)
- **At complex algorithmic steps:** Explain the "why" not the "what"
- **For non-obvious numerical values:** Explain units and rationale
- Plain `.m` files use `%[text]` blocks for markdown-like documentation
- Symbolic output tagged with `%[output:hex_id]` for linked cell outputs (Live Editor feature)
- Uses LaTeX for mathematical expressions: `$\\dot{x} = Ax + Bu$`
- Array notation: `x\_i = \[q\_i;; \\dot{q}\_i\]` (note: doubled semicolon for readable column vector)
- Example (line 32 in `RRpendulum_decoupledLuenberger.m`):
## Function Design
- Ordered consistently: states → inputs → gains/tuning → physical constants
- Example order from `swingup_energy_controller`:
- No default parameter values (all explicitly passed)
- **Single output:** Returned directly without explicit `output =` syntax
- **Multiple outputs:** Comma-separated in return signature
- Auto-generated functions use compact intermediate variable names
## Module Design
- All functions in `resources/functions/` are public (no underscore prefix convention)
- Scripts generate workspace variables (no module concept; everything is global after `run`)
- Symbolic math kept separate (`derive_EOM`, `DH_full`)
- Auto-generated numeric functions stored in `resources/functions/`
- Control scripts in `scripts/` call both, then design controllers
- No nested namespaces (MATLAB R2025b doesn't support package nesting used here)
- Functions organized by purpose: dynamics, kinematics, control, estimation
- Cross-script variables rely on consistent naming convention (`*_num` for numeric parameters)
<!-- GSD:conventions-end -->

<!-- GSD:architecture-start source:ARCHITECTURE.md -->
## Architecture

## Pattern Overview
- Clear execution pipeline: symbolic EOM derivation → parameter substitution → controller/observer design → Simulink simulation → code generation
- Single model per control strategy (one `.slx` per algorithm: FSFB, swing-up, UKF observer)
- Shared hardware interface library (`resources/lib/digtwin_labo_lib.slx`) decouples model development from embedded constraints
- `.mat` file convention for design artifact traceability: all design files follow `.par` (codegen-safe), `.meta` (traceability), `.mlobj` (MATLAB objects) struct pattern
- Two-DOF pendulum dynamics modeled via Euler-Lagrange with Denavit-Hartenberg kinematics
## Layers
- Purpose: Derive and cache continuous-time nonlinear dynamics using Symbolic Math Toolbox
- Location: `scripts/RRpendulum_forkin_dyn_noimage.m`
- Contains: Forward kinematics (DH transformations), kinetic/potential energy, Euler-Lagrange EOM derivation
- Depends on: `resources/functions/DH_full.m`, `resources/functions/derive_EOM.m`
- Used by: All downstream design and control scripts
- Outputs: `data/RRpendulum_EOM.mat` (cached symbolic equations, linearization matrices A/B)
- Purpose: Single source of truth for physical constants and hardware specifications
- Location: `scripts/RRpendulum_Parameters_num_BLDC.m` (primary), `scripts/stepper/RRpendulum_Parameters_num.m` (minimal variant)
- Contains: Mechanism parameters (mass, length, damping), actuation limits, encoder resolution, initial conditions
- Depends on: None (pure data)
- Used by: All control design scripts
- Outputs: `data/RRpendulum_params_BLDC.mat` (hierarchical struct: `.mech`, `.act`, `.sens`, `.ic`)
- Purpose: Synthesize controller/observer gains using Control System Toolbox and custom algorithms
- Locations:
- Contains: Linearization around inverted equilibrium, LQR gain computation, observer tuning, energy controller design
- Depends on: `data/RRpendulum_EOM.mat`, `data/RRpendulum_params_BLDC.mat`
- Used by: Simulink models (via loaded `.mat` files)
- Outputs: Design `.mat` files with `.par` (doubles only, codegen-safe), `.meta` (timestamps, descriptions), `.mlobj` (Control System objects)
- Purpose: Provide numerical ODE evaluation for simulation and observer propagation
- Location: `resources/functions/RRpendulum_dynamics_ct.m` (auto-generated from symbolic derivation)
- Contains: Continuous-time state derivative `xdot = f(x, u)` in explicit form (generated via `matlabFunction`)
- Depends on: State vector `z = [q1; v1; q2; v2]` and control input `tau_1`
- Used by: UKF observer (via `rk4_step`), Simulink S-functions, numerical integration
- Pattern: Auto-generated numeric form avoids symbolic overhead during simulation/codegen
- Purpose: Estimate pendulum state from encoder-only measurements
- Location: `resources/functions/ukf_observer_step.m`
- Contains: Unscented Kalman Filter predict+update cycle with hardened Cholesky repair, Joseph-form covariance update, NaN firewall
- Depends on: `resources/functions/RRpendulum_dynamics_ct.m` (via `rk4_step` for RK4 propagation)
- Used by: Simulink controller models, real-time embedded control
- Design pattern: RK4 discretization of continuous dynamics, diagonal-loading Cholesky for numerical robustness
- Purpose: Test control strategies via Simulink and generate deployable C code
- Locations: `models/stepper/RRpendulum_digtwin_FSFB.slx` (kinematic), `models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx` (primary with motor model)
- Contains: State feedback control blocks, observer blocks, saturation logic, SPI interface to MCU
- Depends on: Design `.mat` files (loaded as `Parameter` scope variables in MATLAB Function blocks)
- Used by: Simulink codegen for embedded deployment
- Deployment target: Raspberry Pi (SPI to STM32F401 MCU at 2 MHz, 1 kHz control loop)
- Purpose: Decouple high-level control from platform-specific SPI communication
- Location: `resources/lib/digtwin_labo_lib.slx`
- Contains: SPI protocol handling, torque command serialization (milli-Nm), encoder reading (counts → rad conversion)
- Depends on: STM32 firmware (`invpend_BLDC`) for SPI message structure
- Used by: All Simulink models requiring real hardware
- Critical constraint: Must stay synchronized with MCU firmware; changes require firmware coordination
## Data Flow
- **State representation:** Always `z = [q1; v1; q2; v2]` across all scripts/models/functions (one unified convention)
- **State storage:** In `.mat` files within design structs (e.g., `ukf.par.ic` for initial UKF state covariance P0)
- **State propagation:** Continuous-time ODE via `RRpendulum_dynamics_ct.m`, discretized via RK4 in observer
## Key Abstractions
- Purpose: Derive nonlinear dynamics from energy principles without ad-hoc force balance
- Examples: `scripts/RRpendulum_forkin_dyn_noimage.m` (symbolic), `resources/functions/derive_EOM.m` (implementation)
- Pattern: L = T - V, then d/dt(∂L/∂q_dot) - ∂L/∂q = Q (generalized forces including friction)
- Enables: Symbolic linearization, validation via energy conservation, easy parameter substitution
- Purpose: Map joint angles to end-effector position via standard SE(3) transformations
- Examples: `resources/functions/DH_full.m` (core), used in `RRpendulum_forkin_dyn_noimage.m`
- Pattern: Cumulative homogeneous transforms T0→1, T1→2, output position p = T_full * [0; 0; -l; 1]
- Benefit: Modular joint addition, easy Jacobian computation
- Purpose: Synthesize linear feedback gains for inverted position stabilization
- Location: `scripts/RRpendulum_FSFB_controldesign_torque.m`
- Pattern:
- Output: K (continuous), Kd (discrete), Nbar (feedforward gain)
- Purpose: Estimate 4D state from 2D encoder measurements (q1, q2 only)
- Location: `resources/functions/ukf_observer_step.m`
- Pattern:
- Design choices:
- Purpose: Swing pendulum from rest (q2 = π) to inverted (q2 = 0) via energy feedback, then catch with FSFB
- Location: `scripts/RRpendulum_swingup_controldesign.m`, `resources/functions/swingup_energy_controller.m`
- Pattern: Three modes
- Reference: Astrom & Furuta (2000)
- Purpose: Enable traceability and codegen-safe parameter passing
- Pattern: All design files save struct with three fields
- Example (from `FSFB_torque_design.mat`):
## Entry Points
- Location: `scripts/RRpendulum_forkin_dyn_noimage.m`
- Triggers: User runs this first to derive and cache symbolic EOM
- Responsibilities: Symbolic kinematics/dynamics derivation, Jacobian computation, caching to `data/RRpendulum_EOM.mat`
- Location: `scripts/RRpendulum_FSFB_controldesign_torque.m`, `RRpendulum_UKF_design.m`, etc.
- Triggers: After symbolic derivation; run in sequence for multiple design artifacts
- Responsibilities: Load cached EOM/params, compute gains/observer parameters, save to `.mat`
- Location: `models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx` (primary model)
- Triggers: Open in Simulink, load design files into workspace, run `sim('RRpendulum_digtwin_FSFB_BLDC')`
- Responsibilities: Execute discrete-time control loop with observer, evaluate closed-loop behavior
- Location: Simulink model + Embedded Coder configuration
- Triggers: `slbuild('RRpendulum_digtwin_FSFB_BLDC')` from MATLAB command line
- Responsibilities: Verify hardware interface (SPI block), generate deployable C code to `codegen/<modelname>_ert_rtw/`
## Error Handling
## Cross-Cutting Concerns
<!-- GSD:architecture-end -->

<!-- GSD:workflow-start source:GSD defaults -->
## GSD Workflow Enforcement

Before using Edit, Write, or other file-changing tools, start work through a GSD command so planning artifacts and execution context stay in sync.

Use these entry points:
- `/gsd:quick` for small fixes, doc updates, and ad-hoc tasks
- `/gsd:debug` for investigation and bug fixing
- `/gsd:execute-phase` for planned phase work

Do not make direct repo edits outside a GSD workflow unless the user explicitly asks to bypass it.
<!-- GSD:workflow-end -->

<!-- GSD:profile-start -->
## Developer Profile

> Profile not yet configured. Run `/gsd:profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->
