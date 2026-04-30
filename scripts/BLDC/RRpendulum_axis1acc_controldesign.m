%% Axis-1 acceleration controller design
%[text] Design parameters for **ctrl\_axis1\_acc**: model-based acceleration
%[text] tracking with integral action and back-calculation anti-windup.
%[text] 
%[text] Gains to tune: `Ki` (integral), `tau\_f\_aw` (anti-windup time constant).
clear
clc

if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
data_dir = fullfile(prj.RootFolder, 'data');

load(fullfile(data_dir, 'RRpendulum_params_BLDC.mat'), 'params');

Ts = 1/1000;

%%
%[text] **Controller gains** (initial values — tune on hardware)
Ki       = 31;
tau_f_aw = 0.016;
tau_min  = -params.act.u_sat;
tau_max  =  params.act.u_sat;

%%
%[text] **Bandwidth and pole analysis**
%[text] 
%[text] Unsaturated closed-loop error dynamics: $\\dot{e}\_v = -K\_i , e\_v$,
%[text] where $e\_v = v\_\\mathrm{ref} - v\_1$.
%[text] The inner loop appears to the outer FSFB as a first-order lag
%[text] $G(s) = K\_i / (s + K\_i)$ for disturbance rejection.

% Continuous pole and bandwidth
pole_ct = -Ki;
bw = Ki;                         % [rad/s]
tau_cl = 1/Ki;                   % [s] time constant

% Discrete pole (ZOH)
pole_dt = exp(pole_ct * Ts);

fprintf('--- Inner loop dynamics ---\n'); %[output:3bc95242]
fprintf('Pole:           s = %.1f  (z = %.6f)\n', pole_ct, pole_dt); %[output:9572a368]
fprintf('Bandwidth:      %.1f rad/s  (%.2f Hz)\n', bw, bw/(2*pi)); %[output:5d6e2f06]
fprintf('Time constant:  %.1f ms\n', tau_cl*1000); %[output:0beb9e73]

%%
%[text] **Interaction with FSFB outer loop**
%[text] 
%[text] The pendulum unstable pole at $\\omega\_p = \\sqrt{g/l}$ sets the minimum
%[text] bandwidth the inner loop must provide. At $\\omega\_p$, the inner loop
%[text] introduces gain attenuation and phase lag into the outer loop.
%[text] Rule of thumb: $K\_i / \\omega\_p \> 5$ for adequate separation.

w_pend = sqrt(params.mech.g / params.mech.l);
gain_at_wpend = Ki / sqrt(w_pend^2 + Ki^2);
phase_at_wpend = -atan2d(w_pend, Ki);

fprintf('\n--- Outer loop interaction ---\n'); %[output:212a748b]
fprintf('Pendulum unstable pole:     %.1f rad/s (%.1f Hz)\n', w_pend, w_pend/(2*pi)); %[output:89d5a2c0]
fprintf('Inner loop gain at w_pend:  %.3f (%.1f dB)\n', gain_at_wpend, 20*log10(gain_at_wpend)); %[output:1dfa9d43]
fprintf('Inner loop phase at w_pend: %.1f°\n', phase_at_wpend); %[output:94bef24b]
fprintf('Bandwidth ratio Ki/w_pend:  %.1f  (want > 5)\n', Ki/w_pend); %[output:31ef5413]

%%
%[text] **Effective inertia range**
%[text] 
%[text] $M\_\\mathrm{eff}$ varies with pose. The controller adapts via online
%[text] computation, but the range determines peak torque requirements.

Meff_min = compute_Meff([0; pi], params.mech.Iz_1, params.mech.l, params.mech.m, params.mech.r);
Meff_max = compute_Meff([0; pi/2], params.mech.Iz_1, params.mech.l, params.mech.m, params.mech.r);

fprintf('\n--- Effective inertia ---\n'); %[output:25109582]
fprintf('Meff at inverted (q2=pi):     %.2e kgm²\n', Meff_min); %[output:311046a5]
fprintf('Meff at horizontal (q2=pi/2): %.2e kgm²\n', Meff_max); %[output:3c06e225]
fprintf('Ratio Meff_max/Meff_min:      %.0fx\n', Meff_max/Meff_min); %[output:0268e87d]
fprintf('Nyquist freq:                 %.0f rad/s  (Ki = %.2f%%)\n', pi/Ts, Ki/(pi/Ts)*100); %[output:175acef1]

%%
%[text] **Save** using `.par` / `.meta` convention
axis1acc = struct();

axis1acc.meta.created     = datetime('now');
axis1acc.meta.script      = mfilename('fullpath');
axis1acc.meta.description = 'Axis-1 acceleration controller with integral action and anti-windup';
axis1acc.meta.analysis.pole_ct         = pole_ct;
axis1acc.meta.analysis.pole_dt         = pole_dt;
axis1acc.meta.analysis.bandwidth_rads  = bw;
axis1acc.meta.analysis.time_constant_s = tau_cl;
axis1acc.meta.analysis.w_pend          = w_pend;
axis1acc.meta.analysis.gain_at_wpend   = gain_at_wpend;
axis1acc.meta.analysis.phase_at_wpend  = phase_at_wpend;
axis1acc.meta.analysis.Meff_range      = [Meff_min, Meff_max];

axis1acc.par.Ki       = Ki;
axis1acc.par.tau_f_aw = tau_f_aw;
axis1acc.par.tau_min  = tau_min;
axis1acc.par.tau_max  = tau_max;
axis1acc.par.Ts       = Ts;

save_dir = fullfile(prj.RootFolder, 'data');
if ~isfolder(save_dir), mkdir(save_dir); end
save(fullfile(save_dir, 'axis1acc_design.mat'), 'axis1acc');
fprintf('Saved to: %s\n', fullfile(save_dir, 'axis1acc_design.mat')); %[output:77b96ce5]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40.3}
%---
%[output:3bc95242]
%   data: {"dataType":"text","outputData":{"text":"--- Inner loop dynamics ---\n","truncated":false}}
%---
%[output:9572a368]
%   data: {"dataType":"text","outputData":{"text":"Pole:           s = -31.0  (z = 0.969476)\n","truncated":false}}
%---
%[output:5d6e2f06]
%   data: {"dataType":"text","outputData":{"text":"Bandwidth:      31.0 rad\/s  (4.93 Hz)\n","truncated":false}}
%---
%[output:0beb9e73]
%   data: {"dataType":"text","outputData":{"text":"Time constant:  32.3 ms\n","truncated":false}}
%---
%[output:212a748b]
%   data: {"dataType":"text","outputData":{"text":"\n--- Outer loop interaction ---\n","truncated":false}}
%---
%[output:89d5a2c0]
%   data: {"dataType":"text","outputData":{"text":"Pendulum unstable pole:     6.2 rad\/s (1.0 Hz)\n","truncated":false}}
%---
%[output:1dfa9d43]
%   data: {"dataType":"text","outputData":{"text":"Inner loop gain at w_pend:  0.980 (-0.2 dB)\n","truncated":false}}
%---
%[output:94bef24b]
%   data: {"dataType":"text","outputData":{"text":"Inner loop phase at w_pend: -11.4°\n","truncated":false}}
%---
%[output:31ef5413]
%   data: {"dataType":"text","outputData":{"text":"Bandwidth ratio Ki\/w_pend:  5.0  (want > 5)\n","truncated":false}}
%---
%[output:25109582]
%   data: {"dataType":"text","outputData":{"text":"\n--- Effective inertia ---\n","truncated":false}}
%---
%[output:311046a5]
%   data: {"dataType":"text","outputData":{"text":"Meff at inverted (q2=pi):     3.00e-05 kgm²\n","truncated":false}}
%---
%[output:3c06e225]
%   data: {"dataType":"text","outputData":{"text":"Meff at horizontal (q2=pi\/2): 1.12e-03 kgm²\n","truncated":false}}
%---
%[output:0268e87d]
%   data: {"dataType":"text","outputData":{"text":"Ratio Meff_max\/Meff_min:      37x\n","truncated":false}}
%---
%[output:175acef1]
%   data: {"dataType":"text","outputData":{"text":"Nyquist freq:                 3142 rad\/s  (Ki = 0.99%)\n","truncated":false}}
%---
%[output:77b96ce5]
%   data: {"dataType":"text","outputData":{"text":"Saved to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\axis1acc_design.mat\n","truncated":false}}
%---
