# Technology Stack

**Analysis Date:** 2026-04-02

## Languages

**Primary:**
- MATLAB - Core simulation, symbolic math, controller design, parameter scripts
- Simulink - Model-based design, simulation, and embedded code generation
- C/C++ - Generated embedded code (Embedded Coder output for Raspberry Pi and STM32)

**Secondary:**
- C Header Files - STM32 MCSDK firmware parameter definitions (parsed and integrated via `scripts/RRpendulum_BLDC_drive_params.m`)

## Runtime

**Environment:**
- MATLAB R2025b or later

**Package Manager:**
- MATLAB Project system (`digtwin_labo.prj`) - manages code organization and dependencies
- Lockfile: Project file `digtwin_labo.prj` (XML format)

## Frameworks

**Core:**
- Simulink - Version R2025b - Model-based simulation, signal flow, hardware interface library blocks
- Embedded Coder - Version R2025b - C code generation targeting ARM (Raspberry Pi, STM32)
- Simulink Coder (RTW/Real-Time Workshop) - Code generation back-end, ERT target (Embedded Real-Time)

**Mathematical Computation:**
- Symbolic Math Toolbox - Automated derivation of equations of motion (EOM), forward kinematics, Jacobians
- Control System Toolbox - LQR controller design, continuous-to-discrete conversion, system analysis

**Testing:**
- Simulink Design Verifier - Reports in `sdireports/` (design verification, test generation)

**Build/Code Generation:**
- Embedded Coder (codegen) - Target: ERT (Embedded Real-Time), output in `codegen/[modelname]_ert_rtw/`
- Simulink Real-Time Workshop - Back-end for code generation

## Key Dependencies

**Critical:**
- **Symbolic Math Toolbox** - Required for `RRpendulum_forkin_dyn_noimage.m` to derive equations of motion
  - Generates symbolic state-space matrices A, B, Jacobians
  - Output: `data/RRpendulum_EOM.mat` (cached, reused by downstream scripts)
  - Dependencies: `resources/functions/DH_full.m`, `resources/functions/derive_EOM.m`

- **Control System Toolbox** - Required for controller design scripts
  - LQR discrete gain computation in `RRpendulum_FSFB_controldesign_torque.m`
  - Observability/controllability analysis via `obsv()`, `ctrb()`, `rank()`
  - Continuous-to-discrete conversion via `c2d()` with zero-order-hold
  - Discrete LQR via `lqrd()`

- **Embedded Coder** - Required for C code generation
  - Target: ERT (Embedded Real-Time)
  - Command: `slbuild('ModelName')` in scripts or GUI
  - Output: `codegen/[modelname]_ert_rtw/` containing `.c`, `.h`, `.elf`

**Infrastructure:**
- **MATLAB Function blocks** in Simulink - for embedding custom numerical code
  - Examples: UKF observer (`ukf_observer_step.m`), dynamics evaluation (`RRpendulum_dynamics_ct.m`)
  - Must be codegen-safe (no object-oriented code, no external function calls during codegen)

- **Simscape** - Optional, used in some variant models for DC motor electrical simulation
  - Referenced but not required for kinematic/torque-controlled variants

## Configuration

**Environment:**
- MATLAB project configuration: `digtwin_labo.prj`
- MATLAB path: Automatically managed by `openProject()` to include `scripts/`, `resources/functions/`, `models/`, `data/`
- Sample time (hardware interface): `Ts = 1/1000` (1 kHz) for SPI communication with STM32 MCU

**Build:**
- Simulink model configurations (per `.slx` file):
  - Solver: ode45 (Runge-Kutta 4/5) for continuous-time dynamics
  - Discrete-time for control loops (sample time 1 ms = 1 kHz)
  - Optimization: InlineParams ON, RTW code optimization level 3
  - Target: ert_rtw (Embedded Real-Time) for code generation
  - Code generation folder: `codegen/`

**Physical Parameters:**
- Single source of truth: `scripts/RRpendulum_Parameters_num_BLDC.m`
  - Outputs: `data/RRpendulum_params_BLDC.mat` (struct with `.mech`, `.act`, `.sens`, `.ic` subfields)
  - Sample time convention: `Ts = 1/2000` (2 kHz for internal computations), `Ts_ctrl = 1/1000` (1 kHz for SPI)

**Design Data Format:**
- All `.mat` design files follow a **`.par` / `.meta` / `.mlobj`** convention:
  - `.par` — numerical parameters only (doubles, logicals, nested structs). Codegen-safe.
  - `.meta` — metadata: timestamps, script names, descriptions (non-codegen objects like `datetime`, `ss`)
  - `.mlobj` — MATLAB objects (`ss` state-space, `datetime`)
  - Pass only `.par` substructs to MATLAB Function blocks as codegen parameters

## Platform Requirements

**Development:**
- MATLAB R2025b or later
- Windows 11 or compatible (project currently on Windows Enterprise 10.0.22631)
- Toolboxes: Simulink, Symbolic Math Toolbox, Control System Toolbox, Embedded Coder, Simulink Coder
- Compiler: GCC/ARM cross-compiler for STM32/Raspberry Pi (required only for actual deployment)

**Production/Deployment:**
- **Raspberry Pi** - Target for high-level control (LQR, UKF, swing-up) via Simulink Embedded Coder
  - Generated code: C/C++ source + runtime support (Embedded Coder for Raspberry Pi target)
  - Communication: SPI at 2 MHz to STM32 MCU
  - Loop rate: 1 kHz
- **STM32F401** - Motor control (Field-Oriented Control at 16 kHz) running ST MCSDK firmware
  - External dependency: `invpend_BLDC` firmware project (`C:/Users/u0130154/STM32_vscode_projects/invpend_BLDC/Inc/`)
  - Hardware interface abstraction: `resources/lib/digtwin_labo_lib.slx` (must stay synchronized with firmware)
- **STEVAL-EDUKIT01** - STMicroelectronics educational 2-DOF rotary pendulum kit

## External Integrations / Hardware Interfaces

**Motor Control Parameters:**
- STM32 Motor Control SDK (MCSDK) - Parameter extraction via C header parsing
  - Script: `scripts/RRpendulum_BLDC_drive_params.m`
  - Reads: `pmsm_motor_parameters.h`, `power_stage_parameters.h`, `drive_parameters.h`, `parameters_conversion.h`
  - Firmware root: `C:/Users/u0130154/STM32_vscode_projects/invpend_BLDC/Inc/`
  - Converts fixed-point firmware gains to SI units for Simulink models

**Hardware Communication:**
- SPI interface library: `resources/lib/digtwin_labo_lib.slx`
  - Protocol: 2 MHz SPI, 1 kHz command/feedback loop
  - Sends: Torque command [mNm] to STM32 MCU
  - Receives: Encoder positions (q1, q2) and velocities (v1, v2)
  - Encoder resolutions: q1=8192 counts/turn (4x quadrature), q2=2400 counts/turn

---

*Stack analysis: 2026-04-02*
