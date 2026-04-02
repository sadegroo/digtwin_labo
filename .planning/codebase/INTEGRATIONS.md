# External Integrations

**Analysis Date:** 2026-04-02

## APIs & External Services

**Motor Control Firmware:**
- STM32 Motor Control SDK (MCSDK) - Motor and drive parameter extraction
  - Service: STMicroelectronics FOC library (Field-Oriented Control for BLDC)
  - Integration point: `scripts/RRpendulum_BLDC_drive_params.m`
  - Mechanism: Parse C header files from companion firmware project
  - Headers parsed: `pmsm_motor_parameters.h`, `power_stage_parameters.h`, `drive_parameters.h`, `parameters_conversion.h`
  - External path: `C:/Users/u0130154/STM32_vscode_projects/invpend_BLDC/Inc/`
  - Converts firmware fixed-point Q15 gain representations to SI units for Simulink simulation

## Data Storage

**Databases:**
- Not used. All state/design data stored as MATLAB `.mat` files

**File Storage:**
- Local filesystem only
  - Design artifacts: `data/` directory (symbolic EOMs, controller gains, UKF parameters, motor drive params)
  - Simulation results: `example_data/` directory
  - Generated code: `codegen/` directory (git-ignored, RTW output)

**Design Data Files (.mat format):**
- `data/RRpendulum_EOM.mat` — Symbolic equations of motion, Jacobians, M/C/G matrices (output of `RRpendulum_forkin_dyn_noimage.m`)
- `data/RRpendulum_params_BLDC.mat` — Physical parameters (mechanism, actuation, sensing) (output of `RRpendulum_Parameters_num_BLDC.m`)
- `data/FSFB_torque_design.mat` — LQR controller gains, plant state-space matrices, feedforward gain (output of `RRpendulum_FSFB_controldesign_torque.m`)
- `data/UKF_design.mat` — Unscented Kalman Filter observer configuration, noise covariances, sigma-point weights (output of `RRpendulum_UKF_design.m`)
- `data/swingup_design.mat` — Energy-based swing-up controller parameters, catch thresholds, trajectory settings (output of `RRpendulum_swingup_controldesign.m`)
- `data/BLDC_drive_params.mat` — Motor electrical parameters, PI controller gains (SI units), power stage settings, filters, protection thresholds (output of `RRpendulum_BLDC_drive_params.m`)
- `data/damping_model.mat` — Identified damping parameters from system identification
- `data/damping_real.mat` — Measured damping coefficients from hardware

**Caching:**
- EOM caching: Symbolic derivation results (`RRpendulum_EOM.mat`) cached to avoid expensive re-computation
  - Generated via `RRpendulum_forkin_dyn_noimage.m` (uses Symbolic Math Toolbox)
  - Reused by all design scripts to extract linearized A/B matrices

## Authentication & Identity

**Auth Provider:**
- Not applicable (no user authentication or cloud services)

## Monitoring & Observability

**Error Tracking:**
- None (no cloud-based error tracking)
- Runtime errors logged to MATLAB console

**Logs:**
- MATLAB console output from design scripts
- Simulink diagnostic viewer during simulation
- SDI (Simulink Data Inspector) for logging signals during simulation
- Generated code runtime output to STM32 UART serial interface (if enabled in firmware)

## CI/CD & Deployment

**Hosting:**
- Target platforms:
  - **Raspberry Pi** — High-level control loop (LQR, UKF, swing-up) deployed via Simulink Embedded Coder
  - **STM32F401** — Motor control (FOC) running ST MCSDK firmware (separate project: `invpend_BLDC`)
  - **STEVAL-EDUKIT01** — Hardware testbed (combines Raspberry Pi + STM32 + BLDC motor + pendulum mechanics)

**Code Generation Pipeline:**
- MATLAB/Simulink → Embedded Coder (ERT target) → C code (`codegen/[modelname]_ert_rtw/`) → ARM cross-compiler → `.elf` binary
- Invoked via: `slbuild('RRpendulum_digtwin_FSFB_BLDC')` in MATLAB or GUI
- Output: Executable `.elf` for Raspberry Pi target

**CI Pipeline:**
- Not implemented. Manual build workflow:
  1. Run `scripts/RRpendulum_forkin_dyn_noimage.m` (symbolic derivation)
  2. Run `scripts/RRpendulum_Parameters_num_BLDC.m` (parameters)
  3. Run design script (e.g., `RRpendulum_FSFB_controldesign_torque.m`, `RRpendulum_UKF_design.m`)
  4. Open and simulate model (e.g., `models/RRpendulum_digtwin_FSFB_BLDC.slx`)
  5. (optional) Run `slbuild()` for code generation
  6. Deploy `.elf` to Raspberry Pi via SCP or similar

## Environment Configuration

**Required env vars:**
- None (project uses MATLAB project system for path management)

**Secrets location:**
- Not applicable. No API keys or credentials used.
- MATLAB project path is absolute on local machine: `C:/Users/u0130154/MATLAB/projects/digtwin_labo/`

**Key Configuration Parameters:**
- Hardware communication: SPI speed = 2 MHz, loop rate = 1 kHz
- Sample times: `Ts_ctrl = 1/1000` (1 kHz for control loop), `Ts_foc = 1/16000` (16 kHz for FOC on STM32)
- Motor torque saturation: ±0.2 Nm
- Encoder resolutions: q1 = 8192 counts/turn, q2 = 2400 counts/turn

## Webhooks & Callbacks

**Incoming:**
- None

**Outgoing:**
- None

## Hardware Communication Interface

**Primary Interface:**
- SPI (Serial Peripheral Interface)
  - Speed: 2 MHz
  - Master: Raspberry Pi (running Embedded Coder-generated control loop)
  - Slave: STM32F401 (running ST MCSDK firmware, `invpend_BLDC` project)
  - Loop rate: 1 kHz
  - Message format:
    - **TX (Raspberry Pi → STM32):** Torque command [milli-Newton-meters]
    - **RX (STM32 → Raspberry Pi):** Encoder readings (q1, q2 positions), velocity estimates (v1, v2)

**Hardware Library:**
- `resources/lib/digtwin_labo_lib.slx` — Shared hardware interface library
  - Contains: SPI blocks, encoder decoding, torque command formatting
  - **Critical:** Must stay synchronized with `invpend_BLDC` firmware
  - Used by: All simulation models that target hardware deployment

**Sensor Interfaces:**
- **Joint 1 Encoder (q1):** Absolute position, quadrature output → 4x decoding → 8192 counts/turn
- **Joint 2 Encoder (q2):** Absolute position, quadrature output → 2x decoding → 2400 counts/turn
- **Motor Current Feedback (Iq):** Sensed via shunt resistor + amplifier (ADC input to STM32)
- **Motor Velocity:** Estimated via encoder differentiation + low-pass filtering (EMA, alpha=0.1)

**Actuator Interface:**
- **BLDC Motor Command:** Torque reference [Nm] → STM32 converts to Iq current reference
  - Current loop (FOC): 16 kHz sample rate (Q15 fixed-point PI controller)
  - Speed loop: 1 kHz sample rate (optional, for speed control mode)
  - Physical limits:
    - Max torque saturation: ±0.2 Nm (in pendulum control)
    - Max phase current (Iq): Limited in firmware (typically 10-15 A for STEVAL-EDUKIT01)
    - Bus voltage: 24 V (nominal)

**Parameter Extraction from Firmware:**
- Script: `scripts/RRpendulum_BLDC_drive_params.m`
- Reads C header files:
  - `pmsm_motor_parameters.h` — Pole pairs, resistance, inductance, voltage constant, encoder PPR
  - `power_stage_parameters.h` — Shunt resistance, amplifier gain, dead-time
  - `drive_parameters.h` — PWM frequency, FOC ISR rate, PI controller gains (Q15 fixed-point)
  - `parameters_conversion.h` — Scaling factors, filter constants
- Converts firmware Q15 gains to SI units for Simulink models

## Simulink Model Hierarchy

**Top-level Models:**
- `models/RRpendulum_digtwin_FSFB_BLDC.slx` — **Primary model**
  - Contains: UKF observer, LQR feedback, swing-up control, hardware interface library
  - Codegen-ready for Raspberry Pi deployment

**Variant Models:**
- `models/RRpendulum_digtwin_BLDC.slx` — BLDC torque control interface only
- `models/RRpendulum_digtwin_swingup.slx` — Standalone swing-up control
- `models/stepper/RRpendulum_digtwin_FSFB.slx` — Kinematic control (non-BLDC)
- `models/stepper/RRpendulum_digtwin.slx` — Base simulation model

**Shared Library:**
- `resources/lib/digtwin_labo_lib.slx` — Hardware interface (SPI, encoders, torque command)
  - Referenced by top-level models via Simulink Model Reference blocks

## Design Artifact Dependencies

**Execution Order (must be followed):**

1. `scripts/RRpendulum_forkin_dyn_noimage.m` → `data/RRpendulum_EOM.mat`
   - Symbolic derivation of kinematics and Euler-Lagrange equations of motion
   - Uses: Symbolic Math Toolbox, `DH_full.m`, `derive_EOM.m`
   - Output: Symbolic A/B matrices for linearization

2. `scripts/RRpendulum_Parameters_num_BLDC.m` → `data/RRpendulum_params_BLDC.mat`
   - Physical parameters (mass, length, damping, encoder resolutions)
   - Independent (no dependencies on EOM)

3. **Choose design script (depends on step 1 & 2):**
   - `scripts/RRpendulum_FSFB_controldesign_torque.m` → `data/FSFB_torque_design.mat`
     - LQR discrete controller gains
     - Uses: Control System Toolbox, EOM, params
   - `scripts/RRpendulum_UKF_design.m` → `data/UKF_design.mat`
     - Unscented Kalman Filter observer
     - Uses: EOM, params
   - `scripts/RRpendulum_swingup_controldesign.m` → `data/swingup_design.mat`
     - Energy-based swing-up controller
     - Uses: params
   - `scripts/RRpendulum_BLDC_drive_params.m` → `data/BLDC_drive_params.mat`
     - Motor/drive parameters from STM32 firmware
     - External dependency: `C:/Users/u0130154/STM32_vscode_projects/invpend_BLDC/Inc/`

4. **Simulate:** Load design `.mat` file into Simulink model and run `sim()`
   - Model loads design parameters as Parameter-scope variables
   - Models reference `resources/lib/digtwin_labo_lib.slx` for hardware interface

5. **(optional) Code Generation:** `slbuild('ModelName')`
   - Embedded Coder generates C code → `codegen/[modelname]_ert_rtw/`
   - ARM cross-compiler produces `.elf` binary
   - Deploy to Raspberry Pi

---

*Integration audit: 2026-04-02*
