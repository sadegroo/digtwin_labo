%% Swing-Up + Catch Controller Design
%[text] **Energy-based swing-up** (Åström & Furuta, Automatica 2000)
%[text] combined with the existing FSFB/LQR catch controller.
%[text] 
%[text] The swing-up law pumps energy into the pendulum until it matches
%[text] the upright equilibrium energy ($E=0$). Once the pendulum is within
%[text] a catch region, the FSFB controller takes over for stabilization.
clear
clc

if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
data_dir = fullfile(prj.RootFolder, 'data');

%% Load parameters and existing FSFB design
load(fullfile(data_dir, 'RRpendulum_params_BLDC.mat'), 'params');
load(fullfile(data_dir, 'FSFB_torque_design.mat'), 'design');

m   = params.mechanism.m;       % 0.0126 kg
l   = params.mechanism.l;       % 0.253 m
r   = params.mechanism.r;       % 0.151 m
g   = params.mechanism.g;       % 9.80665 m/s²
Iz_1 = params.mechanism.Iz_1;   % 3e-5 kgm²
u_sat = params.actuation.u_sat; % 0.2 Nm

%%
%[text] **Maximum pivot acceleration and swing-up ratio** $n$
%[text] 
%[text] The paper characterizes swing-up behavior by:
%[text] $n = u\_\\mathrm{max} / g$
%[text] 
%[text] For our rotary system, the maximum *linear* acceleration at the
%[text] pendulum pivot (distance $r$ from motor axis) is $\\alpha\_\\mathrm{max} \\cdot r$.
%[text] We compute $\\alpha\_\\mathrm{max}$ from the torque saturation and the
%[text] minimum total inertia (at $q\_2 = \\pi$, pendulum folded).

% Minimum Iz1_total occurs when pendulum is inverted (q2 = pi)
Iz1_min = RRpendulum_totalIz1([0; pi], Iz_1, m, r, l);
Iz1_max = RRpendulum_totalIz1([0; pi/2], Iz_1, m, r, l);

alpha_max = u_sat / Iz1_max;
fprintf('alpha_max = %.1f rad/s²  (at min inertia %.2e kgm²)\n', alpha_max, Iz1_max); %[output:157bd663]

% Equivalent linear acceleration at pendulum pivot
u_lin_max = alpha_max * r;
n_ratio = u_lin_max / g;
fprintf('n = u_lin_max/g = %.3f\n', n_ratio); %[output:2dbf9198]

%%
%[text] **Predicted number of swings**
%[text] 
%[text] From the paper (Section 4.4, Eq. 13), the required number of swings
%[text] $k$ satisfies $2\\sin\\theta\_0 \\geq 1 + \\cos(2k-1)\\theta\_0$ where
%[text] $n = \\tan\\theta\_0$.
%[text] 
%[text] | $n$   | swings |
%[text] |-------|--------|
%[text] | \>1.33 | 1      |
%[text] | \>0.58 | 2      |
%[text] | \>0.39 | 3      |
%[text] | \>0.30 | 4      |
%[text] | \>0.24 | 5      |

if n_ratio > 4/3
    k_swings = 1;
elseif n_ratio > 0.577
    k_swings = 2;
elseif n_ratio > 0.388
    k_swings = 3;
elseif n_ratio > 0.296
    k_swings = 4;
elseif n_ratio > 0.241
    k_swings = 5;
else
    k_swings = ceil(pi / (2*n_ratio));  % approximate
end
fprintf('Predicted swings needed: %d  (n = %.3f)\n', k_swings, n_ratio); %[output:4b87d9bd]

%%
%[text] **Energy controller gain** $k\_e$
%[text] 
%[text] From Eq. (8): $u = \\mathrm{sat}\_{n g}(k (E - E\_0), \\mathrm{sign}(\\dot\\theta\\cos\\theta))$
%[text] 
%[text] $k$ must be large enough for near-bang-bang behavior but small enough
%[text] to avoid chattering from measurement noise. A good starting point is
%[text] $k\_e = \\alpha\_\\mathrm{max}, /, (m g l)$ so that the controller saturates
%[text] when $|E - E\_0| \\geq m g l$ (half the total energy swing).

% k_e = alpha_max / (m * g * l);
k_e = 15000;
fprintf('k_e = %.1f  (saturates at |E-E0| = mgl = %.4f J)\n', k_e, m*g*l); %[output:7755719c]

E0 = 0;  % target energy: upright position

%%
%[text] **Catch region thresholds**
%[text] 
%[text] The FSFB controller is designed for the linearized system around
%[text] $q\_2 = \\pi$. It can tolerate deviations up to roughly 20-30 degrees.
%[text] We also require the energy to be close to the target before switching.

q2_catch_threshold = deg2rad(15);  % ±15° from upright (min residual oscillation ~13°)
E_catch_threshold = 0.3 * m * g * l;  % 30% of mgl

fprintf('Catch region: |q2 - pi| < %.1f° and |E| < %.4f J\n', ... %[output:group:1b57a55c] %[output:9f62a694]
    rad2deg(q2_catch_threshold), E_catch_threshold); %[output:group:1b57a55c] %[output:9f62a694]

%%
%[text] **Save swing-up design**

swingup = struct();

% Metadata
swingup.meta.created     = datetime('now');
swingup.meta.script      = mfilename('fullpath');
swingup.meta.description = 'Åström-Furuta energy-based swing-up + FSFB catch';
swingup.meta.reference   = 'Åström & Furuta, "Swinging up a pendulum by energy control", Automatica 36 (2000) 287-295';

% Energy controller parameters
swingup.energy.k_e       = k_e;         % energy feedback gain
swingup.energy.alpha_max = alpha_max;    % max angular acceleration [rad/s²]
swingup.energy.E0        = E0;           % target energy [J]
swingup.energy.n_ratio   = n_ratio;      % u_max/g ratio
swingup.energy.k_swings  = k_swings;     % predicted number of swings

% Catch/switching thresholds
swingup.catch.q2_threshold = q2_catch_threshold;  % [rad]
swingup.catch.E_threshold  = E_catch_threshold;   % [J]
swingup.catch.E_latch      = 0.1 * m * g * l;    % [J] energy threshold for q1 latch
swingup.catch.N_dwell      = 200;                 % samples |E|<E_latch before latching (0.2s at 1kHz)

% Physical parameters (copied for convenience)
swingup.params = params;

% FSFB controller (copied from existing design)
swingup.fsfb.Kd   = design.controller.lqr.Kd;
swingup.fsfb.Nbar = design.controller.Nbar;

% q1 return trajectory after catch
swingup.trajectory.max_rate    = 1.0;    % [rad/s] linear ramp rate
% swingup.trajectory.q1_desired  = 0.0;    % [rad] final q1 setpoint (input in simulink)
swingup.trajectory.Ts          = 1/1000; % [s] sample time

% Save
save_path = fullfile(data_dir, 'swingup_design.mat');
save(save_path, 'swingup');
fprintf('Swing-up design saved to: %s\n', save_path); %[output:3da55eca]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":6.4}
%---
%[output:157bd663]
%   data: {"dataType":"text","outputData":{"text":"alpha_max = 178.0 rad\/s²  (at min inertia 1.12e-03 kgm²)\n","truncated":false}}
%---
%[output:2dbf9198]
%   data: {"dataType":"text","outputData":{"text":"n = u_lin_max\/g = 2.740\n","truncated":false}}
%---
%[output:4b87d9bd]
%   data: {"dataType":"text","outputData":{"text":"Predicted swings needed: 1  (n = 2.740)\n","truncated":false}}
%---
%[output:7755719c]
%   data: {"dataType":"text","outputData":{"text":"k_e = 15000.0  (saturates at |E-E0| = mgl = 0.0313 J)\n","truncated":false}}
%---
%[output:9f62a694]
%   data: {"dataType":"text","outputData":{"text":"Catch region: |q2 - pi| < 15.0° and |E| < 0.0094 J\n","truncated":false}}
%---
%[output:3da55eca]
%   data: {"dataType":"text","outputData":{"text":"Swing-up design saved to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\swingup_design.mat\n","truncated":false}}
%---
