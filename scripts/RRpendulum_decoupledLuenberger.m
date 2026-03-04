%This script is not being used anymore since we use UKF now

clear
clc
run RRpendulum_forkin_dyn_noimage.m
%%
%[text] **Decoupled observer models**
%[text] Explicit accelerations: $\\ddot{q} = M^{-1}(q)\\big(B\_\\tau \\tau\_1 - C(q,\\dot{q})\\dot{q} - B\_{damp}\\dot{q} - G(q)\\big)$

M_inv = simplify(inv(M_mat));
vdot_explicit = simplify(M_inv * (B_tau*tau_1 - C_mat*v - B_damp*v - G_vec));

%[text] Split each joint into self-dynamics $a\_i(q\_i, \\dot{q}\_i, \\tau\_i)$ + coupling $d\_i(q, \\dot{q})$
%[text] Self-dynamics: terms that depend only on joint i's own state and input
%[text] Coupling: everything else (cross-joint interactions)

% --- Joint 1: a_1(q_1, v_1, tau_1) + d_1(q, v) ---
% Set q_2, v_2 = 0 to isolate self-dynamics of joint 1
a1 = simplify(subs(vdot_explicit(1), [q(2), v(2)], [0, 0])) %[output:6442e995]
d1 = simplify(vdot_explicit(1) - a1) %[output:4ffe9591]

% --- Joint 2: a_2(q_2, v_2) + d_2(q, v) ---
% Set q_1, v_1 = 0 and tau_1 = 0 to isolate self-dynamics of joint 2
a2 = simplify(subs(vdot_explicit(2), [q(1), v(1), tau_1], [0, 0, 0])) %[output:34da046c]
d2 = simplify(vdot_explicit(2) - a2) %[output:2ca1d2b6]

% Verify: a_i + d_i should reconstruct the explicit acceleration
verify_dec1 = simplify(a1 + d1 - vdot_explicit(1)) %[output:35bdde86]
verify_dec2 = simplify(a2 + d2 - vdot_explicit(2)) %[output:36895545]

%[text] **Local observer models** for each joint $i$:
%[text] $\\dot{x}\_i = A\_i x\_i + B\_i u\_i + E\_i d\_i$
%[text] with $x\_i = \[q\_i;; \\dot{q}\_i\]$, $u\_i = \\tau\_i$ (joint 1) or $u\_i = 0$ (joint 2), $d\_i$ = lumped coupling/disturbance

% --- Joint 1 observer: x1 = [q_1; v_1] ---
A1_obs = [0, 1; 0, 0];
% b1_coeff: coefficient of tau_1 in a1 (input gain for local model)
b1_coeff = simplify(diff(a1, tau_1)) %[output:2f784117]
B1_obs = [0; b1_coeff];
E_obs = [0; 1];  % coupling enters as additive acceleration

disp('Joint 1 observer: x1_dot = A1*x1 + B1*tau_1 + E*d1') %[output:4bbb0f4c]
disp('A1 ='); disp(A1_obs) %[output:6b877126]
disp('B1 ='); disp(B1_obs) %[output:60bd29c2] %[output:5382b095]
disp('E  ='); disp(E_obs) %[output:7a87ae2b]

% --- Joint 2 observer: x2 = [q_2; v_2] ---
A2_obs = [0, 1; 0, 0];
B2_obs = [0; 0];  % no direct input on joint 2
disp('Joint 2 observer: x2_dot = A2*x2 + E*d2') %[output:2286ad9d]
disp('A2 ='); disp(A2_obs) %[output:350e2682]
disp('E  ='); disp(E_obs) %[output:28ab2dda]

%%
%[text] **Quantization-limited observer bandwidth**
%[text] The effective quantisation frequency sets the maximum useful observer bandwidth.
%[text] $\\omega\_\\text{quant} \\approx \\frac{2\\pi}{T\_s} \\cdot \\frac{1}{N\_\\text{counts}}$
%[text] Rule of thumb: $\\omega\_\\text{obs} \\le \\frac{1}{5},\\omega\_\\text{quant}$

Ts = 1/1000;            % sample rate [s]
N_counts_1 = 8192;      % encoder 1 CPR (joint 1, stepper)
N_counts_2 = 2400;      % encoder 2 CPR (joint 2, pendulum)

% Quantization frequency for each encoder
omega_quant_1 = (2*pi / Ts) * (1 / N_counts_1) %[output:57531275]
omega_quant_2 = (2*pi / Ts) * (1 / N_counts_2) %[output:3de47e84]

% Maximum recommended observer bandwidth (1/5 rule)
omega_obs_max_1 = omega_quant_1 / 5 %[output:4dfaf80d]
omega_obs_max_2 = omega_quant_2 / 5 %[output:5d440165]

%[text] **Verify the rule of thumb**: the quantization step in position is $\\Delta q = 2\\pi / N$.
%[text] At sample rate $1/T\_s$, a 1-count change per sample implies a velocity of $\\Delta q / T\_s$.
%[text] The observer differentiates position to estimate velocity. The noise PSD of quantization
%[text] is approximately $S\_{qq} \\approx (\\Delta q)^2 / (12 \\cdot f\_s)$ (uniform quantization noise).
%[text] A 2nd-order observer with bandwidth $\\omega\_\\text{obs}$ amplifies position noise into velocity
%[text] with gain $\\sim \\omega\_\\text{obs}^2$ at low frequencies. The velocity noise RMS scales as:
%[text] $\\sigma\_v \\approx \\omega\_\\text{obs}^2 \\cdot \\frac{\\Delta q}{\\sqrt{12 \\cdot f\_s \\cdot \\omega\_\\text{obs}}} = \\omega\_\\text{obs}^{3/2} \\cdot \\frac{\\Delta q}{\\sqrt{12 \\cdot f\_s}}$
%[text] The "signal" for 1 count/sample velocity is $\\Delta q / T\_s$. The SNR = 1 condition gives:
%[text] $\\omega\_\\text{obs}^{3/2} \\cdot \\frac{\\Delta q}{\\sqrt{12 f\_s}} \\approx \\Delta q / T\_s$
%[text] $\\Rightarrow \\omega\_\\text{obs} \\approx (12 f\_s^3)^{1/3} \\cdot f\_s^{-2/3}$
%[text] A simpler way: the quantization noise bandwidth is $f\_s / N$. The observer should
%[text] not try to resolve features faster than the encoder can distinguish, hence $\\omega\_\\text{obs} \\lesssim \\omega\_\\text{quant}/5$.

fprintf('\n--- Quantization bandwidth analysis ---\n') %[output:0616cab3]
fprintf('Joint 1: omega_quant = %.3f rad/s, max omega_obs = %.3f rad/s\n', omega_quant_1, omega_obs_max_1) %[output:3ce988c5]
fprintf('Joint 2: omega_quant = %.3f rad/s, max omega_obs = %.3f rad/s\n', omega_quant_2, omega_obs_max_2) %[output:9415cae6]

% Chosen observer bandwidths
omega_obs_1 = 0.15;  % rad/s (joint 1)
omega_obs_2 = 0.5;  % rad/s (joint 2)

fprintf('\nChosen bandwidths:\n') %[output:3a1de760]
fprintf('Joint 1: omega_obs = %.2f rad/s  (limit = %.3f) — %s\n', ... %[output:group:43fcc423] %[output:4f1e93e1]
    omega_obs_1, omega_obs_max_1, ... %[output:4f1e93e1]
    ternary(omega_obs_1 <= omega_obs_max_1, 'OK', 'EXCEEDS LIMIT')) %[output:group:43fcc423] %[output:4f1e93e1]
fprintf('Joint 2: omega_obs = %.2f rad/s  (limit = %.3f) — %s\n', ... %[output:group:66e7df09] %[output:26e1f284]
    omega_obs_2, omega_obs_max_2, ... %[output:26e1f284]
    ternary(omega_obs_2 <= omega_obs_max_2, 'OK', 'EXCEEDS LIMIT')) %[output:group:66e7df09] %[output:26e1f284]

%%
%[text] **Discrete-time nonlinear Luenberger observer design** ($f\_s = 1$ kHz)
%[text] Local model per joint (ZOH discretization of double integrator):
%[text] $A\_d = \[1,; T\_s;; 0,; 1\], \\quad C = \[1,; 0\]$
%[text] Observer: $x\_i(k+1) = A\_d, x\_i(k) + B\_i, u\_i(k) + E, d\_i(k) + L\_i,(y\_i(k) - C, x\_i(k))$
%[text] Error dynamics placed at double pole $z\_{obs} = e^{-\\omega\_{obs}, T\_s}$:
%[text] $L\_i = \[2(1 - z\_{obs});; (1 - z\_{obs})^2 / T\_s\]$

C_obs = [1, 0];  % position measurement for both joints

% Discrete local model (ZOH of double integrator)
A_d = [1, Ts; 0, 1];

% --- Joint 1 observer gains ---
z_obs_1 = exp(-omega_obs_1 * Ts) %[output:5b4e9ab8]
L1 = [2*(1 - z_obs_1); (1 - z_obs_1)^2 / Ts] %[output:2bbe2d49]

% Verify error eigenvalues: eig(A_d - L*C) should be [z_obs, z_obs]
eig_obs1 = eig(A_d - L1*C_obs) %[output:5324e165]

% --- Joint 2 observer gains ---
z_obs_2 = exp(-omega_obs_2 * Ts) %[output:6a86b175]
L2 = [2*(1 - z_obs_2); (1 - z_obs_2)^2 / Ts] %[output:9603fb9b]

eig_obs2 = eig(A_d - L2*C_obs) %[output:89ad81a9]

fprintf('\nObserver gains (discrete, Ts = %.4f s):\n', Ts) %[output:0cc169e0]
fprintf('Joint 1: z_obs = %.6f,  L1 = [%.6f; %.6f],  eig = [%.6f, %.6f]\n', z_obs_1, L1, eig_obs1) %[output:1a01f339]
fprintf('Joint 2: z_obs = %.6f,  L2 = [%.6f; %.6f],  eig = [%.6f, %.6f]\n', z_obs_2, L2, eig_obs2) %[output:70d0405f]
fprintf('Equiv. s-poles: Joint 1 = %.3f rad/s, Joint 2 = %.3f rad/s\n', ... %[output:group:53351641] %[output:4479c254]
    log(z_obs_1)/Ts, log(z_obs_2)/Ts) %[output:group:53351641] %[output:4479c254]

%[text] **Full nonlinear observer equations** (discrete-time, per joint):
%[text] $q\_i(k+1) = q\_i(k) + T\_s, v\_i(k) + L\_{i,1}, (q\_{i,meas}(k) - q\_i(k))$
%[text] $v\_i(k+1) = v\_i(k) + T\_s, a\_i(q\_i, v\_i, u\_i) + L\_{i,2}, (q\_{i,meas}(k) - q\_i(k))$
%[text] The coupling $d\_i$ can be computed from the full state estimate if available,
%[text] or treated as an unknown disturbance (set $d\_i = 0$ for a minimal observer).

%--- helper ---
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":44.3}
%---
%[output:6442e995]
%   data: {"dataType":"symbolic","outputData":{"name":"a1","value":"\\frac{\\tau_1 -b_1 \\,v_1 }{{\\textrm{Iz}}_1 }"}}
%---
%[output:4ffe9591]
%   data: {"dataType":"symbolic","outputData":{"name":"d1","value":"\\frac{l\\,\\tau_1 -b_1 \\,l\\,v_1 +b_2 \\,r\\,v_2 \\,\\cos \\left(q_2 \\right)+\\frac{g\\,l\\,m\\,r\\,\\sin \\left(2\\,q_2 \\right)}{2}+l^2 \\,m\\,r\\,{v_2 }^2 \\,\\sin \\left(q_2 \\right)-l^2 \\,m\\,r\\,{v_1 }^2 \\,{\\left(\\sin \\left(q_2 \\right)-{\\sin \\left(q_2 \\right)}^3 \\right)}-l^3 \\,m\\,v_1 \\,v_2 \\,\\sin \\left(2\\,q_2 \\right)}{l\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}-\\frac{\\tau_1 -b_1 \\,v_1 }{{\\textrm{Iz}}_1 }"}}
%---
%[output:34da046c]
%   data: {"dataType":"symbolic","outputData":{"name":"a2","value":"-\\frac{{\\left(b_2 \\,v_2 +g\\,l\\,m\\,\\sin \\left(q_2 \\right)\\right)}\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}{l^2 \\,m\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}-\\frac{m\\,r^2 \\,{v_2 }^2 \\,\\cos \\left(q_2 \\right)\\,\\sin \\left(q_2 \\right)}{-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 }"}}
%---
%[output:2ca1d2b6]
%   data: {"dataType":"symbolic","outputData":{"name":"d2","value":"\\frac{\\cos \\left(q_2 \\right)\\,{\\left(b_1 \\,r\\,v_1 -r\\,\\tau_1 +\\frac{3\\,l^3 \\,m\\,{v_1 }^2 \\,\\sin \\left(q_2 \\right)}{4}-\\frac{l^3 \\,m\\,{v_1 }^2 \\,\\sin \\left(3\\,q_2 \\right)}{4}+{\\textrm{Iz}}_1 \\,l\\,{v_1 }^2 \\,\\sin \\left(q_2 \\right)+l\\,m\\,r^2 \\,{v_1 }^2 \\,\\sin \\left(q_2 \\right)+l^2 \\,m\\,r\\,v_1 \\,v_2 \\,\\sin \\left(2\\,q_2 \\right)\\right)}}{l\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}"}}
%---
%[output:35bdde86]
%   data: {"dataType":"symbolic","outputData":{"name":"verify_dec1","value":"0"}}
%---
%[output:36895545]
%   data: {"dataType":"symbolic","outputData":{"name":"verify_dec2","value":"0"}}
%---
%[output:2f784117]
%   data: {"dataType":"symbolic","outputData":{"name":"b1_coeff","value":"\\frac{1}{{\\textrm{Iz}}_1 }"}}
%---
%[output:4bbb0f4c]
%   data: {"dataType":"text","outputData":{"text":"Joint 1 observer: x1_dot = A1*x1 + B1*tau_1 + E*d1\n","truncated":false}}
%---
%[output:6b877126]
%   data: {"dataType":"text","outputData":{"text":"A1 =\n     0     1\n     0     0\n\n","truncated":false}}
%---
%[output:60bd29c2]
%   data: {"dataType":"text","outputData":{"text":"B1 =\n","truncated":false}}
%---
%[output:5382b095]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{c}\n0\\\\\n\\frac{1}{{\\textrm{Iz}}_1 }\n\\end{array}\\right)"}}
%---
%[output:7a87ae2b]
%   data: {"dataType":"text","outputData":{"text":"E  =\n     0\n     1\n\n","truncated":false}}
%---
%[output:2286ad9d]
%   data: {"dataType":"text","outputData":{"text":"Joint 2 observer: x2_dot = A2*x2 + E*d2\n","truncated":false}}
%---
%[output:350e2682]
%   data: {"dataType":"text","outputData":{"text":"A2 =\n     0     1\n     0     0\n\n","truncated":false}}
%---
%[output:28ab2dda]
%   data: {"dataType":"text","outputData":{"text":"E  =\n     0\n     1\n\n","truncated":false}}
%---
%[output:57531275]
%   data: {"dataType":"textualVariable","outputData":{"name":"omega_quant_1","value":"0.7670"}}
%---
%[output:3de47e84]
%   data: {"dataType":"textualVariable","outputData":{"name":"omega_quant_2","value":"2.6180"}}
%---
%[output:4dfaf80d]
%   data: {"dataType":"textualVariable","outputData":{"name":"omega_obs_max_1","value":"0.1534"}}
%---
%[output:5d440165]
%   data: {"dataType":"textualVariable","outputData":{"name":"omega_obs_max_2","value":"0.5236"}}
%---
%[output:0616cab3]
%   data: {"dataType":"text","outputData":{"text":"\n--- Quantization bandwidth analysis ---\n","truncated":false}}
%---
%[output:3ce988c5]
%   data: {"dataType":"text","outputData":{"text":"Joint 1: omega_quant = 0.767 rad\/s, max omega_obs = 0.153 rad\/s\n","truncated":false}}
%---
%[output:9415cae6]
%   data: {"dataType":"text","outputData":{"text":"Joint 2: omega_quant = 2.618 rad\/s, max omega_obs = 0.524 rad\/s\n","truncated":false}}
%---
%[output:3a1de760]
%   data: {"dataType":"text","outputData":{"text":"\nChosen bandwidths:\n","truncated":false}}
%---
%[output:4f1e93e1]
%   data: {"dataType":"text","outputData":{"text":"Joint 1: omega_obs = 0.15 rad\/s  (limit = 0.153) — OK\n","truncated":false}}
%---
%[output:26e1f284]
%   data: {"dataType":"text","outputData":{"text":"Joint 2: omega_obs = 0.50 rad\/s  (limit = 0.524) — OK\n","truncated":false}}
%---
%[output:5b4e9ab8]
%   data: {"dataType":"textualVariable","outputData":{"name":"z_obs_1","value":"0.9999"}}
%---
%[output:2bbe2d49]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"-3","name":"L1","rows":2,"type":"double","value":[["0.3000"],["0.0225"]]}}
%---
%[output:5324e165]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"eig_obs1","rows":2,"type":"double","value":[["0.9999"],["0.9999"]]}}
%---
%[output:6a86b175]
%   data: {"dataType":"textualVariable","outputData":{"name":"z_obs_2","value":"0.9995"}}
%---
%[output:9603fb9b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"-3","name":"L2","rows":2,"type":"double","value":[["0.9998"],["0.2499"]]}}
%---
%[output:89ad81a9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"eig_obs2","rows":2,"type":"double","value":[["0.9995"],["0.9995"]]}}
%---
%[output:0cc169e0]
%   data: {"dataType":"text","outputData":{"text":"\nObserver gains (discrete, Ts = 0.0010 s):\n","truncated":false}}
%---
%[output:1a01f339]
%   data: {"dataType":"text","outputData":{"text":"Joint 1: z_obs = 0.999850,  L1 = [0.000300; 0.000022],  eig = [0.999850, 0.999850]\n","truncated":false}}
%---
%[output:70d0405f]
%   data: {"dataType":"text","outputData":{"text":"Joint 2: z_obs = 0.999500,  L2 = [0.001000; 0.000250],  eig = [0.999500, 0.999500]\n","truncated":false}}
%---
%[output:4479c254]
%   data: {"dataType":"text","outputData":{"text":"Equiv. s-poles: Joint 1 = -0.150 rad\/s, Joint 2 = -0.500 rad\/s\n","truncated":false}}
%---
