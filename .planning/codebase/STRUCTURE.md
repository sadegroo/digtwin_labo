# Codebase Structure

**Analysis Date:** 2026-04-02

## Directory Layout

```
digtwin_labo/
├── scripts/                        # MATLAB analysis, design, and parameter scripts
│   ├── RRpendulum_forkin_dyn_noimage.m       # Symbolic EOM derivation (primary)
│   ├── RRpendulum_forkin_dyn.mlx             # Symbolic EOM (Live Script variant with images)
│   ├── RRpendulum_Parameters_num_BLDC.m      # Physical parameters (primary)
│   ├── RRpendulum_Parameters_num.m           # Physical parameters (minimal variant)
│   ├── RRpendulum_FSFB_controldesign_torque.m # LQR full state feedback design
│   ├── RRpendulum_FSFB_controldesign_accel.m  # LQR acceleration control variant
│   ├── RRpendulum_UKF_design.m               # Unscented Kalman Filter observer design
│   ├── RRpendulum_swingup_controldesign.m    # Energy-based swing-up controller design
│   ├── RRpendulum_BLDC_drive_params.m        # Motor/drive parameter extraction
│   ├── RRpendulum_decoupledLuenberger.m      # Luenberger observer (legacy)
│   ├── analyze_ZOH_performance.m             # Zero-order hold analysis
│   ├── ident_damping.mlx                     # Damping identification (Live Script)
│   ├── BLDC/                                 # BLDC-specific scripts
│   │   ├── RRpendulum_BLDC_drive_params.m
│   │   ├── RRpendulum_Parameters_num_BLDC.m
│   │   ├── RRpendulum_FSFB_controldesign_accel.m
│   │   ├── RRpendulum_FSFB_controldesign_torque.m
│   │   ├── RRpendulum_UKF_design.m
│   │   ├── RRpendulum_decoupledLuenberger.m
│   │   └── RRpendulum_swingup_controldesign.m
│   └── stepper/                              # Stepper motor control scripts
│       ├── RRpendulum_Parameters_num.m
│       ├── RRpendulum_FSFB_controldesign.mlx
│       ├── RRpendulum_kinctrl_numericalsetup.m
│       ├── RRpendulum_kinctrl_numericalsetup.mlx
│       ├── RRpendulum_kinctrl_simulation.mlx
│       └── RRpendulum_FSFB_sensitivity_analysis.m
│
├── models/                         # Simulink simulation and control models
│   ├── RRpendulum_digtwin.slx               # Base digital twin model
│   ├── RRpendulum_digtwin_BLDC.slx          # BLDC motor interface variant
│   ├── RRpendulum_digtwin_FSFB_BLDC.slx     # Primary: UKF + LQR + BLDC (codegen-ready)
│   ├── stepper/                             # Stepper motor control models
│   │   ├── RRpendulum_digtwin.slx
│   │   ├── RRpendulum_digtwin_FSFB.slx
│   │   ├── RRpendulum_digtwin_swingup.slx
│   │   ├── RRpendulum_digtwin_swingup2.slx
│   │   ├── RRpendulum_digtwin_swingup_buttonctrl.slx
│   │   ├── RRpendulum_digtwin_swingup_buttonctrl.slx.r2024b
│   │   └── RRpendulum_forcedmovement.slx
│   ├── BLDC/                                # BLDC motor control models
│   │   ├── RRpendulum_digtwin_BLDC.slx
│   │   ├── RRpendulum_digtwin_FSFB_BLDC.slx
│   │   └── RRpendulum_digtwin_FSFB_BLDC_noUI.slx
│   ├── _Oldversions/                        # Archive of previous model versions
│   │   └── RRpendulum_digtwin_BLDC.slx.r2024b
│   └── _testmodels/                         # Diagnostic and test models
│       ├── rpitest_blink.slx
│       └── rpitest_SPI.slx
│
├── resources/                      # Shared libraries, functions, and assets
│   ├── functions/                  # Core utility functions
│   │   ├── RRpendulum_dynamics_ct.m          # Continuous-time nonlinear dynamics (auto-generated)
│   │   ├── derive_EOM.m                      # Euler-Lagrange equation derivation
│   │   ├── DH_full.m                         # Denavit-Hartenberg transformation matrices
│   │   ├── RRpendulum_totalIz1.m             # Total inertia at joint 1 (auto-generated)
│   │   ├── ukf_observer_step.m               # UKF predict+update cycle
│   │   ├── swingup_energy_controller.m       # Astrom-Furuta energy control law
│   │   ├── rotationMatrixToZYXEuler.m        # Rotation matrix to Euler angles
│   │   └── PendulumEnergy.m                  # Total mechanical energy calculation
│   ├── lib/                        # Hardware interface library
│   │   ├── digtwin_labo_lib.slx              # Shared SPI interface (CRITICAL: sync with MCU firmware)
│   │   └── Oldversions/                      # Previous hardware interface versions
│   ├── images/                     # UI and visualization resources
│   │   └── [image files for Live Scripts]
│   └── project/                    # MATLAB project metadata (auto-generated)
│       └── [internal MATLAB project structure]
│
├── data/                           # Design artifacts and calibration data (.mat files)
│   ├── RRpendulum_EOM.mat                   # Symbolic equations of motion (derived once, reused)
│   ├── RRpendulum_params_BLDC.mat           # Physical parameters struct (single source of truth)
│   ├── FSFB_torque_design.mat               # LQR controller design (K, Kd, Nbar gains)
│   ├── UKF_design.mat                       # UKF observer design (noise covariances, sigma weights)
│   ├── swingup_design.mat                   # Swing-up controller design
│   ├── BLDC_drive_params.mat                # Motor/drive electrical parameters
│   ├── damping_model.mat                    # Identified damping from model fitting
│   ├── damping_real.mat                     # Damping coefficients from hardware
│   └── video.mldatx                         # Example video recording
│
├── example_data/                   # Simulation results and example hardware recordings
│   └── [simulation output logs, recorded runs]
│
├── codegen/                        # Generated embedded C code (git-ignored)
│   ├── RRpendulum_digtwin_FSFB_BLDC_ert_rtw/
│   ├── RRpendulum_digtwin_FSFB_BLDC_noUI_ert_rtw/
│   ├── RRpendulum_digtwin_swingup2_ert_rtw/
│   ├── RRpendulum_digtwin_swingup_buttonctrl_ert_rtw/
│   ├── slprj/                                # Simulink build cache
│   └── [other ERT build artifacts]
│
├── UI/                             # Real-time control workbench interface (untracked)
│   └── [MATLAB GUI/dashboard files]
│
├── pdf/                            # Reference documentation and reports
│   └── [PDF documents]
│
├── sdireports/                     # Simulink Design Verifier reports
│   └── [verification/design reports]
│
├── digtwin_labo.prj               # MATLAB project file (registry of tracked files)
├── CLAUDE.md                       # AI assistant guidelines (conventions, setup rules)
├── README.md                       # Project overview and getting started guide
├── .gitattributes                  # Git configuration for binary files (mlAutoMerge)
├── .gitignore                      # Exclude generated code and temporary files
└── .git/                           # Git repository metadata
```

## Directory Purposes

**scripts/:**
- Purpose: MATLAB analysis, design, and parameter definition scripts
- Contains: `.m` (plain text, programmatic) and `.mlx` (Live Scripts, interactive with images)
- Key files: Symbolic derivation, parameter definition, controller/observer design
- Naming: `RRpendulum_[functionality].m` or `.mlx`
- Execution order: Symbolic → Parameters → Design (specific control strategy) → Simulink simulation

**scripts/BLDC/ and scripts/stepper/:**
- Purpose: Motor-specific implementations
- BLDC: Brushless DC motor with FOC (field-oriented control) on STM32, UKF observer required
- stepper: Stepper motor open-loop, kinematic control sufficient, legacy designs
- Pattern: Each motor variant has parallel design scripts

**models/:**
- Purpose: Simulink simulation models for control testing and code generation
- One model per control strategy (FSFB, swing-up, swingup2, swingup_buttonctrl, forced_movement)
- Naming: `RRpendulum_digtwin_[STRATEGY]_[MOTOR].slx`
- Key model: `models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx` (primary production model)

**models/BLDC/ and models/stepper/:**
- Purpose: Motor-specific Simulink variants
- BLDC: Uses motor torque interface, UKF observer, SPI to STM32
- stepper: Kinematic control, open-loop, legacy
- Shared interface: Both reference `resources/lib/digtwin_labo_lib.slx`

**resources/functions/:**
- Purpose: Reusable MATLAB functions for dynamics, control, and state estimation
- Types:
  - Symbolic derivation helpers: `derive_EOM.m`, `DH_full.m`
  - Auto-generated numerics: `RRpendulum_dynamics_ct.m`, `RRpendulum_totalIz1.m` (generated from `.m` scripts via `matlabFunction`)
  - Observer: `ukf_observer_step.m` (embeddable UKF predict+update)
  - Control: `swingup_energy_controller.m` (energy feedback law)
  - Utilities: `rotationMatrixToZYXEuler.m`, `PendulumEnergy.m`
- All used by: Design scripts, Simulink models, codegen targets

**resources/lib/:**
- Purpose: Shared hardware interface library
- Critical file: `digtwin_labo_lib.slx` (SPI protocol, encoder/torque conversion)
- Constraint: MUST stay synchronized with STM32 firmware (`invpend_BLDC`) message structure
- Breaking changes require firmware coordination

**resources/images/:**
- Purpose: Graphics assets for Live Scripts (`.mlx` files)
- Contains: Diagrams, plots, schematic images used in interactive documentation

**data/:**
- Purpose: Design artifacts (controller gains, observer parameters, physical constants)
- Pattern: All `.mat` files follow `.par` (codegen-safe), `.meta` (traceability), `.mlobj` (MATLAB objects) convention
- Key files:
  - `RRpendulum_EOM.mat`: Symbolic dynamics (derived once, reused by all design scripts)
  - `RRpendulum_params_BLDC.mat`: Physical constants (hierarchical: `.mech`, `.act`, `.sens`, `.ic`)
  - Design outputs: FSFB, UKF, swingup designs (loaded into Simulink as Parameter scope variables)
- Why tracked: Design reproducibility; codegen requires `.mat` availability

**example_data/:**
- Purpose: Simulation logs and recorded hardware runs
- Contains: Time-series simulation output, real pendulum recordings
- Use case: Validation, benchmarking, visualization

**codegen/:**
- Purpose: Generated embedded C code (git-ignored)
- Structure: Subdirectory per model (`modelname_ert_rtw/`)
- Contents: Auto-generated C/H files from Simulink Embedded Coder
- Deployment: Code copied to Raspberry Pi for compilation and deployment

**UI/:**
- Purpose: Real-time control dashboard (untracked)
- Status: Not committed to git; ad-hoc development
- Use: Interactive control during hardware testing

**pdf/:**
- Purpose: Reference documentation (control theory, hardware datasheets)
- Contents: Technical reports, design notes

**sdireports/:**
- Purpose: Simulink Design Verifier reports (verification artifacts)
- Contents: Property verification results, coverage analysis

## Key File Locations

**Entry Points:**
- `scripts/RRpendulum_forkin_dyn_noimage.m`: Start here; derives and caches symbolic EOM to `data/RRpendulum_EOM.mat`
- `scripts/RRpendulum_Parameters_num_BLDC.m`: Load after symbolic derivation; defines all physical constants
- `scripts/RRpendulum_FSFB_controldesign_torque.m`: Design LQR controller (primary control strategy)
- `scripts/RRpendulum_UKF_design.m`: Design state observer
- `models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx`: Primary simulation/deployment model

**Configuration:**
- `data/RRpendulum_params_BLDC.mat`: Physical parameters (single source of truth)
  - Hierarchical struct: `.mech` (mass, length, damping), `.act` (torque limits), `.sens` (encoder resolution), `.ic` (initial conditions)
  - All downstream scripts load this file to substitute numerical values
- `resources/lib/digtwin_labo_lib.slx`: Hardware SPI interface (must sync with firmware)

**Core Logic:**
- `resources/functions/RRpendulum_dynamics_ct.m`: Continuous-time state derivative `xdot = f(x, u)`
  - Used by: UKF observer (RK4 integration), Simulink nonlinear plant
  - Auto-generated from symbolic form via `matlabFunction`
- `resources/functions/ukf_observer_step.m`: UKF predict+update (encoder-only state estimation)
  - Input: x_hat, P (state estimate, covariance), u, y (control, measurement), design params
  - Output: x_hat_new, P_new (updated estimate and covariance)
- `resources/functions/derive_EOM.m`: Lagrangian → equations of motion via Euler-Lagrange
  - Used by: `RRpendulum_forkin_dyn_noimage.m` for symbolic derivation
- `resources/functions/DH_full.m`: Joint angles → homogeneous transforms via Denavit-Hartenberg
  - Used by: Kinematics, forward/inverse calculations in design scripts

**Testing:**
- `models/_testmodels/rpitest_blink.slx`: GPIO test (SPI diagnostics)
- `models/_testmodels/rpitest_SPI.slx`: SPI communication test
- `example_data/`: Simulation logs for regression testing (informal; no formal test suite)

## Naming Conventions

**Files:**

| Pattern | Example | Purpose |
|---------|---------|---------|
| `RRpendulum_[function].m` | `RRpendulum_forkin_dyn_noimage.m` | Main script entry points |
| `RRpendulum_[function].mlx` | `RRpendulum_forkin_dyn.mlx` | Interactive Live Script variant (images, formatting) |
| `[motor]/RRpendulum_*` | `BLDC/RRpendulum_Parameters_num_BLDC.m` | Motor-specific implementations |
| `*_noimage.m` | `RRpendulum_forkin_dyn_noimage.m` | Plain script variant (no embedded images; for programmatic use) |
| `*_design.m` | `RRpendulum_FSFB_controldesign_torque.m` | Controller/observer synthesis scripts |
| `*.slx` | `RRpendulum_digtwin_FSFB_BLDC.slx` | Simulink models (binary; version controlled via mlAutoMerge) |

**Directories:**

| Pattern | Example | Purpose |
|---------|---------|---------|
| Lowercase `motor_type/` | `BLDC/`, `stepper/` | Motor-specific code variants |
| Uppercase `FUNCTIONNAME/` | `resources/functions/` | Utility libraries |
| `_prefix` | `_Oldversions/`, `_testmodels/` | Archive/test artifacts (not primary) |

**Variable Naming (MATLAB conventions):**

| Category | Pattern | Example |
|----------|---------|---------|
| Physical parameters | `[symbol]_num` suffix | `m_num`, `l_num`, `g_num`, `b1_num` |
| Symbolic variables | Bare name | `m`, `l`, `g` (in symbolic derivation scripts) |
| Numerical parameters (nested struct) | `.mech.*`, `.act.*`, `.sens.*` | `params.mech.m`, `params.act.u_sat` |
| State vector | `z` or `x` | `z = [q1; v1; q2; v2]` (unified across all code) |
| Design struct fields | `.par`, `.meta`, `.mlobj` | `design.par.lqr.K`, `design.meta.timestamp` |
| Encoder measurements | `[q][joint]_cpt` | `q1_cpt`, `q2_cpt` (counts per turn) |

## Where to Add New Code

**New Feature (e.g., different control strategy):**
- Primary code:
  - Design script: `scripts/RRpendulum_[STRATEGY]_controldesign.m`
    - Load EOM from `data/RRpendulum_EOM.mat`
    - Load params from `data/RRpendulum_params_BLDC.mat`
    - Save design to `data/[STRATEGY]_design.mat` (follow `.par`/`.meta`/`.mlobj` pattern)
  - Simulink model: `models/BLDC/RRpendulum_digtwin_[STRATEGY]_BLDC.slx`
    - Reference hardware interface `resources/lib/digtwin_labo_lib.slx`
    - Load design file as Simulink Parameter variables

**New Component/Module (e.g., different observer):**
- Implementation: `resources/functions/[observer_name]_step.m`
  - Input/output signature compatible with Simulink MATLAB Function blocks
  - Hardened for codegen (no symbolic objects, explicit finite-checking, loop unrolling)
  - Include embedded RK4 or equivalent discrete-time propagation
- Integration: Modify design script to use new observer, save parameters to design `.mat`
- Testing: Create parallel `models/_testmodels/test_[observer_name].slx` for validation

**Utilities/Helpers:**
- Shared functions: `resources/functions/[utility_name].m`
  - Pure functions (no global state, stateless)
  - Documented input/output signatures
  - Include comments for symbolic vs. numeric usage
- Used by: Design scripts and Simulink models

**Motor Variant (new actuator type):**
- Parallel structure:
  - Scripts: `scripts/[motor]/RRpendulum_*.m` (copy/adapt from BLDC or stepper)
  - Models: `models/[motor]/RRpendulum_digtwin_*.slx`
  - Hardware interface: Modify `resources/lib/digtwin_labo_lib.slx` (or create variant library)
- Constraint: Keep hardware interface synchronized with firmware changes

## Special Directories

**codegen/:**
- Purpose: Generated embedded C code (built by Simulink Embedded Coder)
- Generated: `slbuild('RRpendulum_digtwin_FSFB_BLDC')` creates subdirectories
- Committed: No (git-ignored via `.gitignore`)
- Contents: `ert_rtw/` subdirectories containing C/H source, build logs, report files
- Deployment: Copy `*.c` and `*.h` to Raspberry Pi, compile with GCC

**slprj/:**
- Purpose: Simulink build cache (internal compilation artifacts)
- Generated: Automatically by Simulink during model load/save
- Committed: No (ignored)
- Safe to delete: Yes; Simulink regenerates on next load

**resources/project/:**
- Purpose: MATLAB project metadata (internal; do not modify)
- Generated: Automatically by `openProject()`
- Committed: Partially (`.prj` registry file only; internal XML versioning ignored)

**.planning/codebase/:**
- Purpose: Architecture/structure analysis documents (this directory)
- Contents: ARCHITECTURE.md, STRUCTURE.md, CONVENTIONS.md, TESTING.md, CONCERNS.md
- Committed: Yes (guides future development and code generation by AI)
- Updated: When significant changes to codebase patterns occur

---

*Structure analysis: 2026-04-02*
