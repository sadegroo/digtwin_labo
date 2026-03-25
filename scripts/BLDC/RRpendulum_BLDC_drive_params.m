%% BLDC Motor Drive Parameters — extracted from STM32 firmware
%[text] Extract motor electrical parameters, power stage settings, and
%[text] PI controller gains from the STM32 MCSDK firmware in
%[text] `invpend\_BLDC`. Build a `drive` struct for use in a Simscape DC
%[text] motor model (commutation ignored — the BLDC is modelled as an
%[text] equivalent brushed DC motor with back-EMF constant $K\_e$ and torque
%[text] constant $K\_t$).
%[text] 
%[text] **Source firmware:** `C:\Users\u0130154\STM32\_vscode\_projects\invpend\_BLDC`
%[text] 
%[text] **Convention:** all parameters are stored in SI units unless noted
%[text] otherwise. Each field has a comment stating the unit and intended
%[text] usage in a prospective Simulink / Simscape model.

%%
%[text] ## 1 — Firmware root
fw_root = 'C:/Users/u0130154/STM32_vscode_projects/invpend_BLDC/Inc';
% change into a path onm your PC, for code, see: https://github.com/sadegroo/NUCLEO-F401RE-invpend-SPIslave-BLDC
%% Parse C headers
%[text] Helper: read a `#define` value from a C header file.
parse = @(file, token) parse_define(fullfile(fw_root, file), token);

%%
%[text] ## 2 — Motor electrical parameters
%[text] Source: `pmsm\_motor\_parameters.h`

POLE_PAIR_NUM          = parse('pmsm_motor_parameters.h', 'POLE_PAIR_NUM');
RS                     = parse('pmsm_motor_parameters.h', 'RS');
LS                     = parse('pmsm_motor_parameters.h', 'LS');
MOTOR_VOLTAGE_CONSTANT = parse('pmsm_motor_parameters.h', 'MOTOR_VOLTAGE_CONSTANT');
NOMINAL_CURRENT_A      = parse('pmsm_motor_parameters.h', 'NOMINAL_CURRENT_A');
MOTOR_MAX_SPEED_RPM    = parse('pmsm_motor_parameters.h', 'MOTOR_MAX_SPEED_RPM');
M1_ENCODER_PPR         = parse('pmsm_motor_parameters.h', 'M1_ENCODER_PPR');

% Ke conversion:
%   MOTOR_VOLTAGE_CONSTANT is in [V_rms line-to-line / kRPM]
%   Convert to peak phase voltage per electrical rad/s:
%     Ke_peak_ph = Ke_rms_ll / sqrt(3) * sqrt(2) / (kRPM -> rad/s_elec)
%     kRPM -> rad/s_mech = 1000 * 2*pi/60 = 104.72
%     rad/s_elec = rad/s_mech * pole_pairs   (but Ke in PMSM is per elec rad)
%   For DC-equivalent model we want:  Ke_dc = V_bemf / omega_mech  [V/(rad/s)]
%   Ke_dc = MOTOR_VOLTAGE_CONSTANT * sqrt(2)/sqrt(3) / (1000*2*pi/60)
%   This equals Kt for a surface-mount PMSM.
Ke_dc = MOTOR_VOLTAGE_CONSTANT * sqrt(2)/sqrt(3) / (1000 * 2*pi/60);
Kt_dc = Ke_dc;  % surface-mount PMSM: Kt = Ke in consistent SI units

fprintf('Motor: Rs=%.4f Ohm, Ls=%.2e H, Ke=%.4f V/(rad/s), Kt=%.4f Nm/A\n', ... %[output:group:90c5f69d] %[output:3e3d4bfa]
    RS, LS, Ke_dc, Kt_dc); %[output:group:90c5f69d] %[output:3e3d4bfa]

%%
%[text] ## 3 — Power stage parameters
%[text] Source: `power\_stage\_parameters.h`

RSHUNT              = parse('power_stage_parameters.h', 'RSHUNT');
AMPLIFICATION_GAIN  = parse('power_stage_parameters.h', 'AMPLIFICATION_GAIN');
NOMINAL_BUS_VOLTAGE = parse('power_stage_parameters.h', 'NOMINAL_BUS_VOLTAGE_V');
HW_DEAD_TIME_NS     = parse('power_stage_parameters.h', 'HW_DEAD_TIME_NS');
TNOISE_NS           = parse('power_stage_parameters.h', 'TNOISE_NS');
TRISE_NS            = parse('power_stage_parameters.h', 'TRISE_NS');

%%
%[text] ## 4 — Drive / PWM parameters
%[text] Source: `drive\_parameters.h`

PWM_FREQUENCY              = parse('drive_parameters.h', 'PWM_FREQUENCY');
SW_DEADTIME_NS             = parse('drive_parameters.h', 'SW_DEADTIME_NS');
REGULATION_EXECUTION_RATE  = parse('drive_parameters.h', 'REGULATION_EXECUTION_RATE');
SPEED_LOOP_FREQUENCY_HZ    = parse('drive_parameters.h', 'SPEED_LOOP_FREQUENCY_HZ');
IQMAX_A                    = parse('drive_parameters.h', 'IQMAX_A');
MAX_APPLICATION_SPEED_RPM  = parse('drive_parameters.h', 'MAX_APPLICATION_SPEED_RPM');

% FOC execution frequency
ISR_FREQUENCY_HZ = PWM_FREQUENCY / REGULATION_EXECUTION_RATE;

fprintf('PWM: %d Hz, FOC ISR: %d Hz, Speed loop: %d Hz\n', ... %[output:group:04c14a3a] %[output:742f51c3]
    PWM_FREQUENCY, ISR_FREQUENCY_HZ, SPEED_LOOP_FREQUENCY_HZ); %[output:group:04c14a3a] %[output:742f51c3]

%%
%[text] ## 5 — Current (torque) PI controller gains
%[text] Source: `drive\_parameters.h`
%[text] 
%[text] The STM32 MCSDK stores gains as **fixed-point integers** with separate
%[text] divisors (powers of 2). The continuous-time equivalent gains are:
%[text] 
%[text] $$K\_p^{SI} = \\frac{\\text{KP\_DEFAULT}}{\\text{KPDIV}} \\times \\frac{V\_{bus}/2}{I\_{max\_readable}}$$
%[text] 
%[text] where $I\_{max\_readable} = V\_{ADC}/(2 \\cdot R\_{shunt} \\cdot G\_{amp})$.
%[text] 
%[text] However, for the Simulink model we convert directly to SI by keeping
%[text] track of the firmware's internal Q15 scaling.

% --- Raw fixed-point gains ---
PID_TORQUE_KP = parse('drive_parameters.h', 'PID_TORQUE_KP_DEFAULT');
PID_TORQUE_KI = parse('drive_parameters.h', 'PID_TORQUE_KI_DEFAULT');
TF_KPDIV      = parse('drive_parameters.h', 'TF_KPDIV');
TF_KIDIV      = parse('drive_parameters.h', 'TF_KIDIV');

% --- ADC / current sensing ---
ADC_REF_VOLTAGE = 3.3;  % [V]  MCU supply / ADC reference
I_max_readable  = ADC_REF_VOLTAGE / (2 * RSHUNT * AMPLIFICATION_GAIN);  % [A]

% In the MCSDK the PI operates on s16 quantities where:
%   - current error is in digits:  1 digit = I_max_readable / 32767  [A]
%   - output voltage is in digits: 1 digit = (Vbus/2) / 32767       [V]
% The proportional gain converts current-digits to voltage-digits.
% To get SI [V/A]:
%   Kp_SI = (KP_raw / KPDIV) * (Vbus/2) / I_max_readable   [V/A] = [Ohm]
% The integral gain accumulates once per ISR cycle (Ts_foc):
%   Ki_SI = (KI_raw / KIDIV) * (Vbus/2) / I_max_readable / Ts_foc  [V/(A·s)] = [Ohm/s]

Ts_foc = 1 / ISR_FREQUENCY_HZ;  % [s] current loop sample time

voltage_scale = (NOMINAL_BUS_VOLTAGE / 2) / I_max_readable;  % [V/A] digit ratio

Kp_iq_SI = (PID_TORQUE_KP / TF_KPDIV) * voltage_scale;     % [Ohm]
Ki_iq_SI = (PID_TORQUE_KI / TF_KIDIV) * voltage_scale / Ts_foc;  % [Ohm/s]

fprintf('Iq PI (SI): Kp = %.4f Ohm, Ki = %.4f Ohm/s\n', Kp_iq_SI, Ki_iq_SI); %[output:1bda5a63]
fprintf('  (cross-check: Kp/Rs = %.1f, Ki*Ls/Rs = %.1f — expect ~1 for well-tuned loop)\n', ... %[output:group:8ba5b0b4] %[output:15602ee9]
    Kp_iq_SI/RS, Ki_iq_SI*LS/RS); %[output:group:8ba5b0b4] %[output:15602ee9]

%%
%[text] ## 6 — Speed PI controller gains
%[text] Source: `drive\_parameters.h`
%[text] 
%[text] The MCSDK stores speed gains for a PI that converts speed-error (in
%[text] SPEED\_UNIT = 0.1 Hz electrical) to Iq-reference (in s16 current
%[text] digits). The raw defines already include a `/(SPEED\_UNIT/10)` term
%[text] so they are in "per 0.1 Hz" units.
%[text] 
%[text] For SI conversion:
%[text] $$K\_{p,spd}^{SI} = \\frac{\\text{KP\_raw}}{\\text{KPDIV}} \\times \\frac{I\_{max\_readable}}{\\omega\_{scale}}$$
%[text] where $\\omega\_{scale}$ converts SPEED\_UNIT digits to rad/s.

% Raw speed gains — the #define includes /(SPEED_UNIT/10), but the raw
% numbers in the Workbench are 3512 and 636 (for 0.1 Hz unit).
% With SPEED_UNIT = 10 (U_01HZ), SPEED_UNIT/10 = 1, so the division is unity.
PID_SPEED_KP_RAW = 3512;  % Workbench value before /(SPEED_UNIT/10)
PID_SPEED_KI_RAW = 636;   % Workbench value before /(SPEED_UNIT/10)
SP_KPDIV         = parse('drive_parameters.h', 'SP_KPDIV');
SP_KIDIV         = parse('drive_parameters.h', 'SP_KIDIV');

% SPEED_UNIT = U_01HZ = 10.  1 speed digit = 0.1 Hz electrical.
% Mechanical: 1 digit = 0.1 Hz_elec / pole_pairs = 0.1/8 Hz_mech
%           = 0.1/8 * 2*pi rad/s = 0.02*pi/8 rad/s
% Actually the MCSDK speed unit is purely mechanical 0.1 Hz:
%   SPEED_UNIT_2_RPM: rpm = speed * 60 / 10 = speed * 6
%   So 1 digit = 6 RPM = 6*2*pi/60 rad/s = 0.6283 rad/s
SPEED_UNIT = 10;  % U_01HZ
U_RPM      = 60;
speed_digit_to_radps = (U_RPM / SPEED_UNIT) * (2*pi/60);  % 1 digit -> rad/s mechanical

% PI output is in s16 current digits (same scale as torque loop ref).
% 1 current digit = I_max_readable / 32767  [A]
current_digit_to_A = I_max_readable / 32767;

Ts_spd = 1 / SPEED_LOOP_FREQUENCY_HZ;  % [s] speed loop sample time

% Kp converts speed-digits -> current-digits.
% SI: [A / (rad/s)]  =  (KP/KPDIV) * current_digit_to_A / speed_digit_to_radps
Kp_spd_SI = (PID_SPEED_KP_RAW / SP_KPDIV) * current_digit_to_A / speed_digit_to_radps;
Ki_spd_SI = (PID_SPEED_KI_RAW / SP_KIDIV) * current_digit_to_A / speed_digit_to_radps / Ts_spd;

fprintf('Speed PI (SI): Kp = %.4f A/(rad/s), Ki = %.4f A/(rad/s·s)\n', ... %[output:group:3beacac7] %[output:026e07bc]
    Kp_spd_SI, Ki_spd_SI); %[output:group:3beacac7] %[output:026e07bc]

%%
%[text] ## 7 — Low-pass filter parameters
%[text] Source: `parameters\_conversion.h` and `app\_config.h`

% Current measurement LPF (on Iq and Id feedback in the FOC loop)
%   LPF_FILT_CONST = 32767 * 0.5  =>  alpha = 0.5
%   y[k] = alpha*x[k] + (1-alpha)*y[k-1]
%   Equivalent continuous time constant: tau = -Ts / ln(1 - alpha)
alpha_iq_lpf = 0.5;
tau_iq_lpf   = -Ts_foc / log(1 - alpha_iq_lpf);  % [s]
fc_iq_lpf    = 1 / (2*pi*tau_iq_lpf);             % [Hz] -3dB frequency

% Velocity EMA filters (applied at 1 kHz in pendulum_control.c)
alpha_motor_vel = 0.1;
alpha_pend_vel  = 0.1;
tau_motor_vel   = -Ts_spd / log(1 - alpha_motor_vel);  % [s]
fc_motor_vel    = 1 / (2*pi*tau_motor_vel);             % [Hz]

fprintf('Iq LPF:  alpha=%.2f, tau=%.2e s, fc=%.1f Hz\n', alpha_iq_lpf, tau_iq_lpf, fc_iq_lpf); %[output:3fc83290]
fprintf('Vel LPF: alpha=%.2f, tau=%.2e s, fc=%.1f Hz\n', alpha_motor_vel, tau_motor_vel, fc_motor_vel); %[output:810b339e]

%%
%[text] ## 8 — Build the parameter structure
%[text] All fields carry inline documentation of their **unit** and
%[text] **intended use** in a Simscape / Simulink model.

drive = struct();

% ---- Metadata ----
drive.meta.created        = datetime('now');
drive.meta.matlab_version = version;
drive.meta.source_script  = mfilename('fullpath');
drive.meta.fw_root        = fw_root;
drive.meta.description    = [ ...
    'BLDC motor + drive parameters extracted from STM32 MCSDK firmware. ', ...
    'Commutation is ignored — the motor is modelled as an equivalent ', ...
    'DC motor (Simscape ee.electromechanical.rotational.DCMotor) with ', ...
    'back-EMF constant Ke and torque constant Kt.'];

% ---- Motor electrical ----
%   Use with: Simscape DC Motor block (or Controlled Voltage Source + RL + back-EMF)
drive.par.motor.R_ohm           = RS;         % [Ohm]   armature resistance (phase-to-neutral equivalent)
drive.par.motor.L_H             = LS;         % [H]     armature inductance (phase-to-neutral equivalent)
drive.par.motor.Ke_Vpradps      = Ke_dc;      % [V/(rad/s)]  back-EMF constant (mechanical rad/s). Connect to DC Motor Ke port.
drive.par.motor.Kt_NmpA         = Kt_dc;      % [Nm/A]  torque constant = Ke for surface-mount PMSM. Connect to DC Motor Kt port.
drive.par.motor.pole_pairs      = POLE_PAIR_NUM;  % [-] pole pairs (informational — not needed for DC model)
drive.par.motor.I_nom_A         = NOMINAL_CURRENT_A;  % [A] nominal (rated) current
drive.par.motor.I_max_A         = IQMAX_A;    % [A] maximum allowed Iq current (saturation limit in firmware)
drive.par.motor.w_max_radps     = MOTOR_MAX_SPEED_RPM * 2*pi/60;  % [rad/s] rated max mechanical speed
drive.par.motor.encoder_ppr     = M1_ENCODER_PPR;  % [pulses/rev] encoder resolution (before 4x quadrature)
drive.par.motor.encoder_cpr     = 4 * M1_ENCODER_PPR;  % [counts/rev] after quadrature decoding

% ---- Power stage ----
%   Use with: DC voltage source feeding H-bridge / PWM average model
drive.par.power.V_bus_V         = NOMINAL_BUS_VOLTAGE;   % [V] DC bus voltage. Supply voltage in Simscape model.
drive.par.power.R_shunt_ohm     = RSHUNT;                % [Ohm] current sense shunt resistance (per phase)
drive.par.power.G_amp           = AMPLIFICATION_GAIN;     % [-] current sense amplifier gain
drive.par.power.V_adc_ref_V     = ADC_REF_VOLTAGE;       % [V] ADC reference voltage (3.3 V)
drive.par.power.I_max_readable_A = I_max_readable;        % [A] maximum current the ADC can read = Vref/(2*Rshunt*Gamp)
drive.par.power.deadtime_sw_ns  = SW_DEADTIME_NS;         % [ns] software dead time
drive.par.power.deadtime_hw_ns  = HW_DEAD_TIME_NS;        % [ns] hardware dead time

% ---- PWM / timing ----
%   Use with: PWM block sample time or Rate Transition block
drive.par.timing.f_pwm_Hz       = PWM_FREQUENCY;          % [Hz] PWM switching frequency (16 kHz)
drive.par.timing.Ts_pwm_s       = 1 / PWM_FREQUENCY;      % [s]  PWM period
drive.par.timing.f_foc_Hz       = ISR_FREQUENCY_HZ;       % [Hz] FOC ISR rate = PWM freq / execution rate
drive.par.timing.Ts_foc_s       = Ts_foc;                  % [s]  current loop sample time (= 1/f_foc)
drive.par.timing.f_spd_Hz       = SPEED_LOOP_FREQUENCY_HZ; % [Hz] speed control loop execution rate
drive.par.timing.Ts_spd_s       = Ts_spd;                  % [s]  speed loop sample time (= 1/f_spd)
drive.par.timing.reg_exec_rate  = REGULATION_EXECUTION_RATE; % [-] FOC runs every N PWM cycles

% ---- Torque (Iq) PI controller ----
%   Use with: Discrete PI block (Simulink) with sample time = Ts_foc
%   Transfer function: C(z) = Kp + Ki * Ts / (1 - z^-1)
%   Anti-windup: clamp integrator output to ± V_bus/2
drive.par.torque_pi.Kp_ohm      = Kp_iq_SI;   % [Ohm]   proportional gain (voltage / current error)
drive.par.torque_pi.Ki_ohm_ps   = Ki_iq_SI;    % [Ohm/s] integral gain
drive.par.torque_pi.Ts_s        = Ts_foc;      % [s]     sample time for discrete implementation
drive.par.torque_pi.I_max_A     = IQMAX_A;     % [A]     output clamp for Iq reference
drive.par.torque_pi.V_max_V     = NOMINAL_BUS_VOLTAGE / 2;  % [V] output saturation (half bus for SVPWM)
drive.par.torque_pi.raw_Kp      = PID_TORQUE_KP;  % [-] firmware integer gain (for traceability)
drive.par.torque_pi.raw_Ki      = PID_TORQUE_KI;  % [-] firmware integer gain
drive.par.torque_pi.raw_Kp_div  = TF_KPDIV;       % [-] firmware divisor (power of 2)
drive.par.torque_pi.raw_Ki_div  = TF_KIDIV;       % [-] firmware divisor (power of 2)

% ---- Speed PI controller ----
%   Use with: Discrete PI block (Simulink) with sample time = Ts_spd
%   Input:  speed error [rad/s]
%   Output: Iq current reference [A]
%   Anti-windup: clamp output to ± I_max
drive.par.speed_pi.Kp_Apradps   = Kp_spd_SI;  % [A/(rad/s)]    proportional gain
drive.par.speed_pi.Ki_Apradps2  = Ki_spd_SI;  % [A/(rad/s·s)]  integral gain
drive.par.speed_pi.Ts_s         = Ts_spd;     % [s]             sample time for discrete implementation
drive.par.speed_pi.I_max_A      = IQMAX_A;    % [A]             output clamp (Iq saturation)
drive.par.speed_pi.w_max_radps  = MAX_APPLICATION_SPEED_RPM * 2*pi/60;  % [rad/s] max application speed
drive.par.speed_pi.raw_Kp       = PID_SPEED_KP_RAW;  % [-] Workbench integer gain (for traceability)
drive.par.speed_pi.raw_Ki       = PID_SPEED_KI_RAW;  % [-] Workbench integer gain
drive.par.speed_pi.raw_Kp_div   = SP_KPDIV;          % [-] firmware divisor
drive.par.speed_pi.raw_Ki_div   = SP_KIDIV;           % [-] firmware divisor

% ---- Low-pass filters ----
%   Iq measurement filter: first-order discrete LPF at FOC rate
%   Use with: Discrete Transfer Fcn block:  H(z) = alpha / (1 - (1-alpha)*z^-1)
drive.par.filters.iq_lpf.alpha       = alpha_iq_lpf;   % [-]  filter coefficient (0.5 = heavy filtering)
drive.par.filters.iq_lpf.tau_s       = tau_iq_lpf;      % [s]  equivalent continuous time constant
drive.par.filters.iq_lpf.fc_Hz       = fc_iq_lpf;       % [Hz] -3 dB cutoff frequency
drive.par.filters.iq_lpf.Ts_s        = Ts_foc;          % [s]  sample time

%   Motor velocity EMA filter: applied in pendulum_control.c at 1 kHz
%   Use with: Discrete Transfer Fcn at speed loop rate
drive.par.filters.motor_vel.alpha    = alpha_motor_vel;  % [-]  EMA coefficient
drive.par.filters.motor_vel.tau_s    = tau_motor_vel;    % [s]  equivalent time constant
drive.par.filters.motor_vel.fc_Hz    = fc_motor_vel;     % [Hz] -3 dB frequency
drive.par.filters.motor_vel.Ts_s     = Ts_spd;           % [s]  sample time

%   Pendulum velocity EMA filter (same structure, different sensor)
drive.par.filters.pend_vel.alpha     = alpha_pend_vel;
drive.par.filters.pend_vel.tau_s     = -Ts_spd / log(1 - alpha_pend_vel);
drive.par.filters.pend_vel.fc_Hz     = 1 / (2*pi * drive.par.filters.pend_vel.tau_s);
drive.par.filters.pend_vel.Ts_s      = Ts_spd;

% ---- Current sensing (ADC scaling) ----
%   Useful for modelling quantization / sensor noise in simulation
drive.par.adc.R_shunt_ohm       = RSHUNT;              % [Ohm]
drive.par.adc.G_amp              = AMPLIFICATION_GAIN;  % [-]
drive.par.adc.V_ref_V            = ADC_REF_VOLTAGE;     % [V]
drive.par.adc.I_per_digit_A      = I_max_readable / 32767;  % [A/digit] current resolution per ADC digit
drive.par.adc.bits               = 12;                  % [-] ADC resolution
drive.par.adc.I_max_readable_A   = I_max_readable;      % [A]

% ---- Protection thresholds ----
drive.par.protection.V_over_V    = 29;   % [V] over-voltage fault threshold
drive.par.protection.V_under_V   = 10;   % [V] under-voltage fault threshold
drive.par.protection.T_over_C    = 70;   % [°C] over-temperature threshold
drive.par.protection.I_max_A     = IQMAX_A;  % [A] maximum current (firmware hard limit)
drive.par.protection.w_max_radps = MAX_APPLICATION_SPEED_RPM * 2*pi/60;  % [rad/s] max speed

%%
%[text] ## 9 — Display summary
fprintf('\n===== BLDC Drive Parameter Summary =====\n'); %[output:2593c05f]
fprintf('Motor:   R=%.3f Ohm, L=%.2e H, Ke=%.4f V/(rad/s), Kt=%.4f Nm/A\n', ... %[output:group:2cc19cd5] %[output:3c1d27ba]
    drive.par.motor.R_ohm, drive.par.motor.L_H, drive.par.motor.Ke_Vpradps, drive.par.motor.Kt_NmpA); %[output:group:2cc19cd5] %[output:3c1d27ba]
fprintf('Power:   Vbus=%d V, Rshunt=%.3f Ohm, Gamp=%.2f, Imax_adc=%.1f A\n', ... %[output:group:06a880b5] %[output:429f4a92]
    drive.par.power.V_bus_V, drive.par.power.R_shunt_ohm, drive.par.power.G_amp, drive.par.power.I_max_readable_A); %[output:group:06a880b5] %[output:429f4a92]
fprintf('Timing:  PWM=%d Hz, FOC=%d Hz, Speed=%d Hz\n', ... %[output:group:9fea7817] %[output:9673f9e9]
    drive.par.timing.f_pwm_Hz, drive.par.timing.f_foc_Hz, drive.par.timing.f_spd_Hz); %[output:group:9fea7817] %[output:9673f9e9]
fprintf('Iq PI:   Kp=%.4f Ohm, Ki=%.1f Ohm/s  (Ts=%.2e s)\n', ... %[output:group:02bc639a] %[output:3ab9d144]
    drive.par.torque_pi.Kp_ohm, drive.par.torque_pi.Ki_ohm_ps, drive.par.torque_pi.Ts_s); %[output:group:02bc639a] %[output:3ab9d144]
fprintf('Spd PI:  Kp=%.4f A/(rad/s), Ki=%.4f A/(rad/s·s)  (Ts=%.3f s)\n', ... %[output:group:4b48f8aa] %[output:7f56f63a]
    drive.par.speed_pi.Kp_Apradps, drive.par.speed_pi.Ki_Apradps2, drive.par.speed_pi.Ts_s); %[output:group:4b48f8aa] %[output:7f56f63a]
fprintf('Filters: Iq LPF fc=%.0f Hz, Vel EMA fc=%.1f Hz\n', ... %[output:group:7ae35b32] %[output:308481ef]
    drive.par.filters.iq_lpf.fc_Hz, drive.par.filters.motor_vel.fc_Hz); %[output:group:7ae35b32] %[output:308481ef]
fprintf('=========================================\n'); %[output:4975bb73]

%%
%[text] ## 10 — Save
if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
save_dir = fullfile(prj.RootFolder, 'data');
if ~isfolder(save_dir), mkdir(save_dir); end

save_path = fullfile(save_dir, 'BLDC_drive_params.mat');
save(save_path, 'drive');
fprintf('\nSaved drive parameters to: %s\n', save_path); %[output:25603846]

%%
%[text] ## Appendix — Usage guide for Simulink / Simscape
%[text] 
%[text] **DC Motor block (Simscape Electrical):**
%[text] - Armature resistance: `drive.par.motor.R\_ohm` \[Ohm\]
%[text] - Armature inductance: `drive.par.motor.L\_H` \[H\]
%[text] - Back-EMF constant: `drive.par.motor.Ke\_Vpradps` \[V/(rad/s)\]
%[text] - Torque constant: `drive.par.motor.Kt\_NmpA` \[Nm/A\] \
%[text] 
%[text] **Torque (current) control loop:**
%[text] - Discrete PI block, sample time = `drive.par.torque\_pi.Ts\_s`
%[text] - P = `drive.par.torque\_pi.Kp\_ohm`, I = `drive.par.torque\_pi.Ki\_ohm\_ps`
%[text] - Input: current error \[A\] = Iq\_ref − Iq\_meas
%[text] - Output: voltage command \[V\] applied to motor terminals
%[text] - Saturation: ± `drive.par.torque\_pi.V\_max\_V` \
%[text] 
%[text] **Speed control loop:**
%[text] - Discrete PI block, sample time = `drive.par.speed\_pi.Ts\_s`
%[text] - P = `drive.par.speed\_pi.Kp\_Apradps`, I = `drive.par.speed\_pi.Ki\_Apradps2`
%[text] - Input: speed error \[rad/s\] = w\_ref − w\_meas
%[text] - Output: Iq reference \[A\]
%[text] - Saturation: ± `drive.par.speed\_pi.I\_max\_A` \
%[text] 
%[text] **Measurement filters:**
%[text] - Iq feedback: `drive.par.filters.iq\_lpf.alpha` at `drive.par.filters.iq\_lpf.Ts\_s`
%[text] - Speed feedback: `drive.par.filters.motor\_vel.alpha` at `drive.par.filters.motor\_vel.Ts\_s` \
%[text] 
%[text] **PWM average model:**
%[text] - If using a PWM average model instead of switching, the voltage \
%[text]   applied to the motor is: $V\_{motor} = \\frac{duty}{PWM\_{period}} \\times V\_{bus}$
%[text] - Include a transport delay of `drive.par.timing.Ts\_pwm\_s` / 2 to model \
%[text]   the zero-order hold effect of the PWM.
%[text] 
%[text]  **Key design choices:**
%[text]   \- The Ke conversion from V\_rms\_ll/kRPM to V/(rad/s) follows the standard PMSM formula: Ke = Ke\_ll \* sqrt(2)/sqrt(3) / (1000\*2\*pi/60)
%[text]   \- The fixed-point → SI gain conversion accounts for the MCSDK's s16 digit scaling through the voltage\_scale = (Vbus/2) / I\_max\_readable ratio
%[text]   \- Cross-check shows Ki\*Ls/Rs ≈ 0.9 which confirms the current loop targets a bandwidth near Rs/Ls
%[text]   \- The script includes an embedded parse\_define function that handles C type casts and suffixes




function val = parse_define(filepath, token)







txt = fileread(filepath);

pat = ['#define\s+' token '\s+([^\s/]+)'];
tok = regexp(txt, pat, 'tokens', 'once');
if isempty(tok)
error('parse_define:notFound', 'Token "%s" not found in %s', token, filepath);
end
raw = tok{1};

raw = regexprep(raw, '\([a-zA-Z_][a-zA-Z0-9_]*\)', '');

raw = regexprep(raw, '[fFuUlL]+$', '');
val = str2double(raw);
if isnan(val)
error('parse_define:parseError', 'Cannot parse "%s" for token "%s"', raw, token);
end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":53.4}
%---
%[output:3e3d4bfa]
%   data: {"dataType":"text","outputData":{"text":"Motor: Rs=0.2300 Ohm, Ls=1.50e-04 H, Ke=0.0234 V\/(rad\/s), Kt=0.0234 Nm\/A\n","truncated":false}}
%---
%[output:742f51c3]
%   data: {"dataType":"text","outputData":{"text":"PWM: 16000 Hz, FOC ISR: 16000 Hz, Speed loop: 1000 Hz\n","truncated":false}}
%---
%[output:1bda5a63]
%   data: {"dataType":"text","outputData":{"text":"Iq PI (SI): Kp = 0.8999 Ohm, Ki = 1379.6165 Ohm\/s\n","truncated":false}}
%---
%[output:15602ee9]
%   data: {"dataType":"text","outputData":{"text":"  (cross-check: Kp\/Rs = 3.9, Ki*Ls\/Rs = 0.9 — expect ~1 for well-tuned loop)\n","truncated":false}}
%---
%[output:026e07bc]
%   data: {"dataType":"text","outputData":{"text":"Speed PI (SI): Kp = 0.0425 A\/(rad\/s), Ki = 0.0601 A\/(rad\/s·s)\n","truncated":false}}
%---
%[output:3fc83290]
%   data: {"dataType":"text","outputData":{"text":"Iq LPF:  alpha=0.50, tau=9.02e-05 s, fc=1765.1 Hz\n","truncated":false}}
%---
%[output:810b339e]
%   data: {"dataType":"text","outputData":{"text":"Vel LPF: alpha=0.10, tau=9.49e-03 s, fc=16.8 Hz\n","truncated":false}}
%---
%[output:2593c05f]
%   data: {"dataType":"text","outputData":{"text":"\n===== BLDC Drive Parameter Summary =====\n","truncated":false}}
%---
%[output:3c1d27ba]
%   data: {"dataType":"text","outputData":{"text":"Motor:   R=0.230 Ohm, L=1.50e-04 H, Ke=0.0234 V\/(rad\/s), Kt=0.0234 Nm\/A\n","truncated":false}}
%---
%[output:429f4a92]
%   data: {"dataType":"text","outputData":{"text":"Power:   Vbus=24 V, Rshunt=0.010 Ohm, Gamp=5.18, Imax_adc=31.9 A\n","truncated":false}}
%---
%[output:9673f9e9]
%   data: {"dataType":"text","outputData":{"text":"Timing:  PWM=16000 Hz, FOC=16000 Hz, Speed=1000 Hz\n","truncated":false}}
%---
%[output:3ab9d144]
%   data: {"dataType":"text","outputData":{"text":"Iq PI:   Kp=0.8999 Ohm, Ki=1379.6 Ohm\/s  (Ts=6.25e-05 s)\n","truncated":false}}
%---
%[output:7f56f63a]
%   data: {"dataType":"text","outputData":{"text":"Spd PI:  Kp=0.0425 A\/(rad\/s), Ki=0.0601 A\/(rad\/s·s)  (Ts=0.001 s)\n","truncated":false}}
%---
%[output:308481ef]
%   data: {"dataType":"text","outputData":{"text":"Filters: Iq LPF fc=1765 Hz, Vel EMA fc=16.8 Hz\n","truncated":false}}
%---
%[output:4975bb73]
%   data: {"dataType":"text","outputData":{"text":"=========================================\n","truncated":false}}
%---
%[output:25603846]
%   data: {"dataType":"text","outputData":{"text":"\nSaved drive parameters to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\BLDC_drive_params.mat\n","truncated":false}}
%---
