# Digital Twin Laboratory - RR Pendulum Control System

A MATLAB/Simulink project for rotary RR (2-DOF) pendulum control simulation and real-time hardware deployment. This project implements control algorithms for both virtual simulations and physical embedded systems (STM32, Raspberry Pi).

## Project Overview

This digital twin framework enables:
- **Controller Design**: Develop and validate control algorithms in simulation
- **Hardware Integration**: Deploy generated code to embedded targets
- **System Identification**: Estimate damping and dynamic parameters from real data
- **Real-time Control**: Interface with educational robotics hardware

## Directory Structure

```
digtwin_labo/
├── models/                    # Simulink simulation and control models
├── scripts/                   # MATLAB analysis and parameter setup scripts
├── resources/
│   ├── functions/            # Core utility functions (dynamics, kinematics)
│   ├── images/               # UI/visualization resources
│   └── lib/                  # Support libraries
├── data/                      # Calibration and experimental data
├── example_data/             # Simulation results and example recordings
├── codegen/                  # Generated embedded C code for hardware
├── UI/                       # Real-time control workbench interface
└── sdireports/               # Simulink Design Verifier reports
```

## Simulink Models

### Main RR Pendulum Models

| Model | Description |
|-------|-------------|
| `RRpendulum_digtwin.slx` | Base digital twin with simulation & codegen variants |
| `RRpendulum_digtwin_BLDC.slx` | BLDC motor torque control interface |
| `RRpendulum_digtwin_FSFB.slx` | Full State Feedback controller for inverted position stabilization |
| `RRpendulum_digtwin_swingup.slx` | Swing-up control algorithm |
| `RRpendulum_digtwin_swingup_buttonctrl.slx` | Swing-up with manual button control |
| `RRpendulum_forcedmovement.slx` | Forced oscillation analysis |

### Embedded System Test Models

| Model | Description |
|-------|-------------|
| `rpitest_blink.slx` | Raspberry Pi LED blink test |
| `rpitest_SPI.slx` | Raspberry Pi SPI communication test with dashboard |

## Scripts

### Control Design & Analysis (`scripts/`)

---

#### `RRpendulum_forkin_dyn.mlx`

**Purpose:** Derive forward kinematics and equations of motion for the 2-DOF RR pendulum using symbolic computation.

**Sections:**
- Symbolic variable definitions
- DH parameter setup and forward kinematics
- Lagrangian mechanics derivation
- Equation of motion generation

**Inputs:** None (standalone symbolic derivation)

**Outputs:**
- Symbolic variables: `q1`, `q2`, `v1`, `v2`, `t` (generalized coordinates and time)
- Dynamics expressions for use in numerical scripts
- Forward kinematics transformations

**Dependencies:**
- Toolboxes: Symbolic Math Toolbox
- Functions: `DH_full.m`, `derive_EOM.m`

**Execution Notes:**
- Run this script first before `RRpendulum_kinctrl_numericalsetup.m`
- Creates symbolic workspace variables used by downstream scripts
- Expected runtime: Fast (symbolic computation)

---

#### `RRpendulum_kinctrl_numericalsetup.m` / `.mlx`

**Purpose:** Define numerical parameter values for pendulum simulation and set up initial conditions for forced motion analysis.

**Sections:**
- Clear workspace and run `RRpendulum_forkin_dyn.mlx`
- Define physical parameters (mass, lengths, gravity, damping)
- Define saturation limits
- Set up forced q1 motion (sinusoidal velocity profile)
- Define initial conditions

**Inputs:**
- Requires `RRpendulum_forkin_dyn.mlx` to have been run (symbolic variables)

**Outputs:**
| Variable | Value | Units | Description |
|----------|-------|-------|-------------|
| `m_num` | 0.0126 | kg | Pendulum link mass |
| `l_num` | 0.253 | m | Link length |
| `r_num` | 0.151 | m | Radius parameter |
| `g_num` | 9.80665 | m/s² | Gravitational acceleration |
| `b1_num` | 0.0001 | Nms/rad | Joint 1 damping coefficient |
| `b2_num` | 0.00003 | Nms/rad | Joint 2 damping coefficient |
| `satlimits` | [64.34, 19.63] | [rad/s², rad/s] | Acceleration and velocity saturation |
| `v1ampl` | 3 | rad/s | Forced velocity amplitude |
| `v1freq` | 1 | Hz | Forced velocity frequency |
| `z0` | [q1_0; v1_0; q2_0; v2_0] | - | Initial state vector |

**Dependencies:**
- Simulink model: `RRpendulum_digtwin.slx` (uses these parameters)
- Prerequisite: `RRpendulum_forkin_dyn.mlx`

**Execution Notes:**
- Run before any simulation to populate workspace with parameters
- Overwrites workspace variables
- Expected runtime: Fast

**Usage:**
```matlab
run('scripts/RRpendulum_kinctrl_numericalsetup.m')
% Then open and simulate RRpendulum_digtwin.slx
```

---

#### `RRpendulum_kinctrl_simulation.mlx`

**Purpose:** Run kinematic control simulation and visualize pendulum trajectory tracking.

**Sections:**
- Simulation setup
- Run Simulink model
- Extract and plot results

**Inputs:**
- Workspace variables from `RRpendulum_kinctrl_numericalsetup.m`
- Simulink model: `RRpendulum_digtwin.slx`

**Outputs:**
- Figures: Trajectory plots, state evolution
- Logged simulation data

**Dependencies:**
- Simulink model: `RRpendulum_digtwin.slx`
- Prerequisite: `RRpendulum_kinctrl_numericalsetup.m`

**Execution Notes:**
- Run after parameter setup script
- Expected runtime: Moderate (depends on simulation length)

---

#### `RRpendulum_FSFB_controldesign.mlx`

**Purpose:** Design full state feedback (LQR-type) controller for stabilizing the pendulum at the inverted position.

**Sections:**
- Linearization around equilibrium point
- State-space model formulation
- LQR gain computation
- Controller validation

**Inputs:**
- System parameters from `RRpendulum_kinctrl_numericalsetup.m`
- Linearized dynamics matrices

**Outputs:**
- `K`: State feedback gain matrix
- Controller parameters for Simulink model

**Dependencies:**
- Toolboxes: Control System Toolbox
- Simulink model: `RRpendulum_digtwin_FSFB.slx`

**Execution Notes:**
- Run after establishing numerical parameters
- Computed gains are used in the FSFB Simulink model
- Expected runtime: Fast

---

#### `RRpendulum_torquectrl.mlx`

**Purpose:** Set up torque control parameters for direct motor torque/current control interface.

**Sections:**
- Torque control parameter definitions
- Motor model parameters

**Inputs:**
- Base system parameters

**Outputs:**
- Torque control configuration variables

**Dependencies:**
- Simulink model: `RRpendulum_digtwin_BLDC.slx`

**Execution Notes:**
- Short script for torque control setup
- Expected runtime: Fast

---

#### `ident_damping.mlx`

**Purpose:** Identify damping coefficients by comparing experimental data with simulated model outputs.

**Sections:**
- Load experimental data
- Run parameter identification
- Compare measured vs simulated trajectories
- Save identified parameters

**Inputs:**
- Experimental data files from `data/` folder
- `damping_real.mat`, `video.mldatx`

**Outputs:**
- Identified damping coefficients
- `damping_model.mat`: Saved identification results
- Figures: Comparison plots

**Dependencies:**
- Simulink model: `RRpendulum_digtwin.slx` or `RRpendulum_forcedmovement.slx`
- Data files in `data/` folder

**Execution Notes:**
- Requires experimental data to be present
- May run simulations for parameter fitting
- Expected runtime: Moderate to long (depending on optimization iterations)

### Script Execution Order

For a typical workflow, run scripts in this order:

```
1. RRpendulum_forkin_dyn.mlx          (symbolic derivation - run once)
        ↓
2. RRpendulum_kinctrl_numericalsetup.m (parameter setup - run before each simulation)
        ↓
3. Choose one:
   ├── RRpendulum_kinctrl_simulation.mlx   (kinematic control)
   ├── RRpendulum_FSFB_controldesign.mlx   (state feedback design)
   ├── RRpendulum_torquectrl.mlx           (torque control)
   └── ident_damping.mlx                   (parameter identification)
```

## Core Functions (`resources/functions/`)

| Function | Purpose |
|----------|---------|
| `derive_EOM.m` | Lagrangian-based equation of motion derivation |
| `DH_full.m` | Denavit-Hartenberg transformation matrices for forward kinematics |
| `rotationMatrixToZYXEuler.m` | Rotation matrix to ZYX Euler angles conversion |

## Control Strategies

1. **Kinematic Control** - Trajectory tracking with feedforward
2. **Swing-up Control** - Energy-based control to bring pendulum upright
3. **Full State Feedback (FSFB)** - LQR-type stabilization at inverted position
4. **Torque/Current Control** - Direct motor control interfaces

## Hardware Support

### Target Platforms
- **STM32 Microcontroller** - Primary embedded target with generated C code
- **Raspberry Pi** - GPIO and SPI communication support
- **STEVAL-EDUKIT01** - STMicroelectronics educational robotics kit

### Real-Time Interface
The `UI/Edukit_Real_Time_Control_System_Workbench_Matlab.m` provides:
- Serial communication at 23400 baud
- Rotor and pendulum impulse control
- Load/noise disturbance rejection testing
- Sinusoidal reference tracking
- Real-time data logging

## Requirements

### MATLAB/Simulink
- **MATLAB Version**: R2025b or later
- **Required Toolboxes**:
  - Simulink
  - Control System Toolbox
  - Symbolic Math Toolbox
  - Embedded Coder (for code generation)
  - Simulink Design Verifier (optional)

### Hardware (optional)
- STM32-based controller board
- Raspberry Pi (for alternative deployment)
- STEVAL-EDUKIT01 or compatible 2-DOF pendulum setup

## Getting Started

1. **Open the project** in MATLAB:
   ```matlab
   openProject('digtwin_labo.prj')
   ```

2. **Set up parameters** by running:
   ```matlab
   run('scripts/RRpendulum_kinctrl_numericalsetup.m')
   ```

3. **Open a model** to simulate:
   ```matlab
   open_system('models/RRpendulum_digtwin.slx')
   ```

4. **Run simulation** using the Simulink Run button or:
   ```matlab
   sim('RRpendulum_digtwin')
   ```

## Data Files

### Calibration Data (`data/`)

| File | Description |
|------|-------------|
| `damping_model.mat` | Identified damping parameters from model fitting |
| `damping_real.mat` | Damping coefficients measured from real hardware |
| `video.mldatx` | Experimental video/data recording for validation |

### Example Data (`example_data/`)

| File | Description |
|------|-------------|
| `swingup_sim.mldatx` | Swing-up simulation recording |
| `swingup_video.mldatx` | Swing-up visualization data |

## License

This project is intended for educational and research purposes.

## Contributing

Contributions are welcome. Please ensure any changes are tested with the existing models before submitting.
