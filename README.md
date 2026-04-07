# Digital Twin Laboratory - RR Pendulum Control System

A MATLAB/Simulink project for a rotary RR (2-DOF) inverted pendulum (STEVAL-EDUKIT01). Covers the full pipeline: symbolic derivation of kinematics/dynamics, Simulink simulation, LQR controller design, UKF state estimation, energy-based swing-up, embedded C code generation (Raspberry Pi / STM32), and real hardware deployment.

## Getting Started

1. **Open the project** in MATLAB:
   ```matlab
   openProject('digtwin_labo.prj')
   ```

### Simulation vs Hardware

The Simulink models use a **variant subsystem** that automatically switches between simulation and hardware mode. When you run the model normally (play button), it uses the simulated plant. When you deploy to the Raspberry Pi in **external mode** (Hardware tab), the variant block switches to the real SPI interface — no manual configuration needed.

### Stepper

The stepper model `RRpendulum_digtwin.slx` loads `RRpendulum_Parameters_num.m` automatically via a model callback — just open and simulate:

   ```matlab
   open_system('models/RRpendulum_digtwin.slx')
   sim('RRpendulum_digtwin')
   ```

### BLDC

1. **Generate all parameters** (symbolic derivation, physical parameters, controller, observer, and swing-up design):
   ```matlab
   run('scripts/BLDC/RRpendulum_BLDC_genAllParams.m')
   ```

2. **Simulate**:
   ```matlab
   open_system('models/BLDC/RRpendulum_digtwin_FSFB_BLDC.slx')
   sim('RRpendulum_digtwin_FSFB_BLDC')
   ```

## Project Overview

This digital twin framework enables:
- **Symbolic Derivation**: Euler-Lagrange EOMs via Symbolic Math Toolbox, cached in `.mat` for reuse
- **Controller Design**: LQR full state feedback, energy-based swing-up with hybrid catch
- **State Estimation**: Unscented Kalman Filter (UKF) with RK4 propagation, hardened for embedded codegen
- **Motor Drive Modelling**: Extract BLDC torque/speed controller parameters from STM32 MCSDK firmware
- **Hardware Integration**: Deploy generated code to Raspberry Pi (SPI at 2 MHz to STM32 MCU)
- **System Identification**: Estimate damping and dynamic parameters from real data

## Directory Structure

```
digtwin_labo/
├── models/                    # Simulink simulation and control models
├── scripts/                   # MATLAB analysis, design, and parameter scripts
├── resources/
│   ├── functions/            # Core utility functions (dynamics, UKF, swing-up)
│   ├── images/               # UI/visualization resources
│   └── lib/                  # Hardware interface library (digtwin_labo_lib.slx)
├── data/                      # Design artefacts (.mat), calibration data
├── example_data/             # Simulation results and example recordings
├── codegen/                  # Generated embedded C code (git-ignored)
├── UI/                       # Real-time control workbench interface
└── sdireports/               # Simulink Design Verifier reports
```

## State Vector Convention

All scripts use: `z = [q1; v1; q2; v2]` where q = angle (rad), v = angular velocity (rad/s).
- `q1`: horizontal rotation (driven joint)
- `q2`: pendulum angle (free joint, `q2 = pi` is downward rest)

## Simulink Models

### Main RR Pendulum Models

| Model | Description |
|-------|-------------|
| `RRpendulum_digtwin.slx` | Base digital twin with simulation & codegen variants |
| `RRpendulum_digtwin_BLDC.slx` | BLDC motor torque control interface |
| `RRpendulum_digtwin_FSFB.slx` | Full state feedback controller (kinematic mode) |
| `RRpendulum_digtwin_FSFB_BLDC.slx` | **Primary model**: UKF observer + LQR + swing-up, codegen-ready |
| `RRpendulum_digtwin_FSFB_BLDC_noUI.slx` | BLDC state feedback without UI (headless codegen) |
| `RRpendulum_digtwin_swingup.slx` | Standalone swing-up control |
| `RRpendulum_digtwin_swingup_buttonctrl.slx` | Button-controlled swing-up (manual trigger via hardware button) |
| `RRpendulum_forcedmovement.slx` | Forced oscillation analysis |

### Hardware Interface

`resources/lib/digtwin_labo_lib.slx` contains the shared hardware interface library. It **must stay synchronized with STM32 MCU firmware** (`invpend_BLDC`). Torque commands are sent in milli-Nm via SPI; the MCU converts to Iq current internally. SPI frames include CRC error checking for robust communication with the latest pendulum firmware.

## Scripts

### Execution Pipeline (order matters)

```
1. RRpendulum_forkin_dyn_noimage.m   → Symbolic EOM → saves data/RRpendulum_EOM.mat
        ↓
2. RRpendulum_Parameters_num_BLDC.m  → Physical constants → saves data/RRpendulum_params_BLDC.mat
        ↓
3. Choose a design script:
   ├── RRpendulum_FSFB_controldesign_torque.m  → LQR gains → data/FSFB_torque_design.mat
   ├── RRpendulum_UKF_design.m                 → UKF observer → data/UKF_design.mat
   ├── RRpendulum_swingup_controldesign.m      → Swing-up + catch → data/swingup_design.mat
   └── RRpendulum_BLDC_drive_params.m          → Motor/drive PI gains → data/BLDC_drive_params.mat
        ↓
4. Open and simulate the corresponding .slx model
        ↓
5. (optional) slbuild() for Embedded Coder deployment
```

### Script Reference

---

#### `RRpendulum_forkin_dyn_noimage.m`

**Purpose:** Derive forward kinematics and Euler-Lagrange EOMs using Symbolic Math Toolbox. Saves all symbolic results to `data/RRpendulum_EOM.mat` for reuse by downstream scripts (avoids re-running slow symbolic computation).

**Outputs:** `RRpendulum_EOM.mat` containing dynamics, Jacobians, and multibody decomposition (M, C, G matrices).

**Dependencies:** Symbolic Math Toolbox, `DH_full.m`, `derive_EOM.m`

> A Live Script variant `RRpendulum_forkin_dyn.mlx` exists with embedded images/formatting. Use the `.m` version for programmatic execution.

---

#### `RRpendulum_Parameters_num_BLDC.m`

**Purpose:** Single source of truth for all physical parameters. Saves a hierarchical struct to `data/RRpendulum_params_BLDC.mat`.

**Key parameters:**

| Variable | Value | Units | Description |
|----------|-------|-------|-------------|
| `m_num` | 0.0126 | kg | Pendulum link mass |
| `l_num` | 0.253 | m | Pendulum length |
| `r_num` | 0.151 | m | Link 1 offset |
| `g_num` | 9.80665 | m/s^2 | Gravitational acceleration |
| `b1_num` | 0.0001 | Nms/rad | Joint 1 viscous damping |
| `b2_num` | 0.00003 | Nms/rad | Joint 2 viscous damping |
| `u_sat` | 0.2 | Nm | Symmetric torque saturation |
| `q1_cpt` | 8192 | counts/turn | Joint 1 encoder (4x quadrature of 2048 PPR) |
| `q2_cpt` | 2400 | counts/turn | Joint 2 encoder |

---

#### `RRpendulum_FSFB_controldesign_torque.m`

**Purpose:** Design discrete LQR controller for inverted-position stabilization. Loads EOM from `.mat` (no symbolic re-derivation needed). Linearizes around `z_eq = [0; 0; pi; 0]`, computes continuous and discrete LQR gains, and feedforward gain Nbar for q1 reference tracking.

**Outputs:** `data/FSFB_torque_design.mat` containing `design` struct with `.par` (plant matrices, LQR gains K/Kd, Nbar, Q, R), `.mlobj` (ss objects), and `.meta` (closed-loop poles, traceability).

**Dependencies:** Control System Toolbox, `RRpendulum_EOM.mat`, `RRpendulum_params_BLDC.mat`

---

#### `RRpendulum_UKF_design.m`

**Purpose:** Design Unscented Kalman Filter (UKF) observer for state estimation from encoder-only measurements.

**Key design choices:**
- Encoder quantization noise: `R = pi^2 / (3 * N^2)` per encoder channel
- Acceleration-noise process model with tunable `qa1`, `qa2`
- Wan-Merwe sigma points with `alpha = 0.5` (avoids extreme weights)
- RK4 propagation for continuous dynamics discretization

**Outputs:** `data/UKF_design.mat` containing `ukf` struct with `.par` (noise covariances, sigma-point weights, initial conditions, measurement model) and `.meta` (dynamics reference, traceability).

---

#### `RRpendulum_swingup_controldesign.m`

**Purpose:** Design Astrom-Furuta energy-based swing-up controller with hybrid catch logic.

**Control modes:**
1. **Mode 0 (Swing-up):** Energy control law pumps pendulum energy toward `E0 = m*g*l`
2. **Mode 1 (Stabilize):** Partial FSFB (q2/v1/v2 only, no q1 feedback) waits for energy to settle below `E_latch` for `N_dwell` samples
3. **Mode 2 (Track):** Full FSFB with linear q1 ramp back to desired position

**Outputs:** `data/swingup_design.mat` containing `swingup` struct with energy controller parameters, catch thresholds, and trajectory settings.

**Reference:** Astrom & Furuta, "Swinging up a pendulum by energy control", *Automatica* 36 (2000) 287-295

---

#### `RRpendulum_BLDC_drive_params.m`

**Purpose:** Parse STM32 Motor Control SDK C header files from the companion firmware project (`invpend_BLDC`) and extract motor, power stage, and PI controller parameters. Converts the MCSDK fixed-point gains to SI units for use in a Simscape DC motor model (commutation ignored).

**Extracted parameter groups** (all under `drive.par.*`):

| Section | Key fields | Description |
|---------|-----------|-------------|
| `drive.par.motor` | `R_ohm`, `L_H`, `Ke_Vpradps`, `Kt_NmpA` | DC-equivalent electrical parameters |
| `drive.par.power` | `V_bus_V`, `R_shunt_ohm`, `G_amp` | Power stage / current sensing |
| `drive.par.timing` | `Ts_foc_s` (62.5 us), `Ts_spd_s` (1 ms) | Control loop sample times |
| `drive.par.torque_pi` | `Kp_ohm`, `Ki_ohm_ps` | Iq current PI in SI (Ohm, Ohm/s) |
| `drive.par.speed_pi` | `Kp_Apradps`, `Ki_Apradps2` | Speed PI in SI (A/(rad/s)) |
| `drive.par.filters` | `iq_lpf`, `motor_vel` | Measurement LPF parameters |
| `drive.par.protection` | OV/UV/OT thresholds | Safety limits |

**Outputs:** `data/BLDC_drive_params.mat`

**External dependency:** Reads C headers from `C:/Users/u0130154/STM32_vscode_projects/invpend_BLDC/Inc/`

---

#### Other Scripts

| Script | Purpose |
|--------|---------|
| `score_competition.m` | Competition scoring tool (see [Competition Scoring](#competition-scoring) below) |
| `RRpendulum_FSFB_controldesign_accel.m` | LQR design with acceleration-command interface (BLDC variant) |
| `RRpendulum_FSFB_sensitivity_analysis.m` | Sensitivity analysis of LQR gains to parameter variations |
| `RRpendulum_decoupledLuenberger.m` | Decoupled Luenberger observer design (superseded by UKF) |
| `RRpendulum_kinctrl_numericalsetup.m` | Legacy parameter setup for kinematic control models |
| `RRpendulum_Parameters_num.m` | Minimal numerical parameters (non-BLDC variant) |
| `analyze_ZOH_performance.m` | Zero-order hold discretization performance analysis |

## Core Functions (`resources/functions/`)

| Function | Purpose |
|----------|---------|
| `derive_EOM.m` | Euler-Lagrange equation of motion derivation |
| `DH_full.m` | Denavit-Hartenberg transformation matrices |
| `RRpendulum_dynamics_ct.m` | Numerical continuous-time dynamics f(z,u) for simulation and UKF |
| `ukf_observer_step.m` | Stateless UKF predict+update step (RK4, diagonal-loading Cholesky repair, Joseph-form update, velocity clamping, NaN firewall) |
| `swingup_energy_controller.m` | Astrom-Furuta energy feedback control law |
| `RRpendulum_totalIz1.m` | Configuration-dependent total inertia at joint 1 |
| `PendulumEnergy.m` | Compute pendulum total mechanical energy |
| `rotationMatrixToZYXEuler.m` | Rotation matrix to ZYX Euler angles |

## Design Data Files (`data/`)

All design `.mat` files follow a **`.par` / `.meta` / `.mlobj`** struct convention:
- **`.par`** — numerical parameters only (doubles, logicals, nested structs of doubles). Codegen-safe; pass `*.par` directly into MATLAB Function blocks as a `Parameter`-scope variable.
- **`.meta`** — traceability: timestamps, script names, descriptions, string annotations.
- **`.mlobj`** — MATLAB objects useful in MATLAB but not codegen-safe (`ss`, `datetime`, etc.).

| File | Variable | Key substructs | Description |
|------|----------|---------------|-------------|
| `RRpendulum_EOM.mat` | `EOM` | — | Symbolic equations of motion, Jacobians, M/C/G matrices |
| `RRpendulum_params_BLDC.mat` | `params` | `.mech`, `.act`, `.sens`, `.ic` | Physical parameters (mechanism, actuation, sensing, ICs) |
| `FSFB_torque_design.mat` | `design` | `.par.lqr`, `.par.plant`, `.par.Nbar`, `.mlobj.plant` | LQR controller: discrete gains, plant matrices, ss objects |
| `UKF_design.mat` | `ukf` | `.par.noise`, `.par.sigma`, `.par.ic`, `.par.dims` | UKF observer: noise covariances, sigma-point weights, P0 |
| `swingup_design.mat` | `swingup` | `.par.energy`, `.par.catch`, `.par.fsfb`, `.par.plant` | Swing-up: energy gains, catch thresholds, FSFB catch gains |
| `BLDC_drive_params.mat` | `drive` | `.par.motor`, `.par.timing`, `.par.torque_pi`, `.par.speed_pi` | Motor/drive: electrical params, PI gains (SI), filters, protection |
| `damping_model.mat` | - | — | Identified damping parameters from model fitting |
| `damping_real.mat` | - | — | Damping coefficients measured from real hardware |

## Hardware Architecture

```
Raspberry Pi (Simulink model)
    │
    │  SPI @ 2 MHz, 1 kHz loop
    │  TX: torque command [mNm]
    │  RX: q1, q2, v1, v2 (encoder counts & velocities)
    │
STM32F401 (invpend_BLDC firmware)
    │  FOC @ 16 kHz  ←── Iq/Id PI controllers
    │  Speed loop @ 1 kHz (when in speed mode)
    │
    └── 3-phase inverter → BLDC motor (8 pole pairs)
            Kt = 0.0234 Nm/A, Rs = 0.23 Ohm, Ls = 150 uH
```

### Target Platforms
- **STM32F401** - Motor control (FOC, current sensing, encoder) via ST MCSDK
- **Raspberry Pi** - High-level control (LQR, UKF, swing-up) via Simulink Embedded Coder
- **STEVAL-EDUKIT01** - STMicroelectronics educational robotics kit

## Competition Scoring

`scripts/score_competition.m` is a MATLAB scoring tool for the digital twin swingup
competition. It processes team submissions (`.mldatx` files from Simulink Data Inspector),
validates successful swingups, computes swingup times and simulation accuracy (SMAPE),
and produces a ranked leaderboard with plots.

### Prerequisites

- MATLAB project open (`openProject('digtwin_labo.prj')` -- the script checks automatically)
- Team `.mldatx` files ready (each file contains one hardware run and one simulation run)

### Usage Workflow

1. **Run the script:**
   ```matlab
   run('scripts/score_competition.m')
   ```

2. **Select q2 display unit** (rev, deg, or rad) when prompted

3. **Session loop** (repeat for each attempt):
   - Select a `.mldatx` file via the file dialog
   - Assign it to a team (5 pre-configured teams)
   - Select the command signal and q2 signal from the file's signal list
   - Preview hw/sim overlay and confirm signal selection
   - Enter time alignment delta (left-shift hardware signal, in seconds)
   - Type `done` to finalize, or press Enter to load the next file

4. **Finalization** (automatic after `done`):
   - Per-team diagnostic summary (swingup times, SMAPE values)
   - Ranked leaderboard table
   - CSV, xlsx, and .mat export to `data/`
   - Score breakdown bar chart

### Configuration

Edit the `cfg` struct at the top of the script to adjust:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cfg.teams` | 5 teams (1 BLDC + 4 stepper) | Team names and motor types |
| `cfg.smape_fixed_duration` | 5 s | SMAPE comparison window duration |
| `cfg.swingup_hold_time` | 1.0 s | Time pendulum must hold upright |
| `cfg.swingup_tolerance_deg` | 2 deg | Tolerance around upright position |
| `cfg.time_points` | [2, 1, 0.5, 0] | Stepper time-ranking points (1st to 4th) |
| `cfg.smape_points` | [2, 1, 0.5, 0.5] | Stepper SMAPE-ranking points (1st to 4th) |
| `cfg.bldc_smape_bands` | [40, 80, 120, 160] | BLDC absolute SMAPE% band edges |
| `cfg.bldc_smape_pts` | [4, 3, 2, 1, 0] | Points per SMAPE band |
| `cfg.export_dir` | `data` | Output directory for results |

### Output Files

After finalization, results are saved to `data/` with timestamped filenames:

| File | Contents |
|------|----------|
| `competition_results_YYYYMMDD_HHMM.csv` | Leaderboard table (CSV) |
| `competition_results_YYYYMMDD_HHMM.xlsx` | Leaderboard table (Excel) |
| `competition_results_YYYYMMDD_HHMM.mat` | Full session struct and table for post-hoc analysis |

### Scoring Rules

- **Stepper teams** (4 teams): Ranked competitively on swingup time and SMAPE. Best result per metric across multiple attempts is used.
- **BLDC team** (1 team): Scored on absolute SMAPE bands (not ranked against stepper teams).
- **Participation point**: 1 point awarded if the pendulum exceeds 90 degrees during any attempt.

## Requirements

### MATLAB/Simulink
- **MATLAB Version**: R2025b or later
- **Required Toolboxes**:
  - Simulink
  - Control System Toolbox
  - Symbolic Math Toolbox
  - Embedded Coder (for code generation)
  - Simulink Coder

### Hardware (optional)
- STEVAL-EDUKIT01 or compatible 2-DOF pendulum
- Raspberry Pi with SPI interface
- STM32F401-based motor controller with ST MCSDK firmware

## License

This project is intended for educational and research purposes.
