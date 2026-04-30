%% FSFB controller design — acceleration input
%[text] Parallel to `RRpendulum\_FSFB\_controldesign\_torque.m`, but assumes
%[text] perfect acceleration control on axis 1 via `ctrl\_axis1\_acc`.
%[text] Input $u = \\ddot{q}\_1$ \[rad/s$^2$\] instead of $\\tau\_1$ \[Nm\].
%[text] 
%[text] Axis 1 becomes a pure double integrator ($\\dot{v}\_1 = u$);
%[text] pendulum coupling on axis 2 is preserved through $M\_{21}$.
clear
clc

if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
data_dir = fullfile(prj.RootFolder, 'data');

load(fullfile(data_dir, 'RRpendulum_EOM.mat'), 'EOM');
load(fullfile(data_dir, 'RRpendulum_params_BLDC.mat'), 'params');

% Unpack symbolic variables for linearization and parameter substitution
f    = EOM.nlss.f;
x    = EOM.sym.x;
u    = EOM.sym.u;
q    = EOM.sym.q;
v    = EOM.sym.v;
sym_params = EOM.sym.params;  % [g l r m b_1 b_2 Iz_1]
g    = sym_params(1); l   = sym_params(2); r    = sym_params(3);
m    = sym_params(4); b_1 = sym_params(5); b_2  = sym_params(6);
Iz_1 = sym_params(7);

% Unpack numerical variables
% Mechanism
m_num   = params.mech.m;
l_num   = params.mech.l;
r_num   = params.mech.r;
g_num  = params.mech.g;
b1_num  = params.mech.b1;
b2_num  = params.mech.b2;
Iz_1_num = params.mech.Iz_1;

% Sensing
q1_cpt = params.sens.q1_cpt;
q2_cpt = params.sens.q2_cpt;

% Actuation
u_sat = params.act.u_sat;

% Sample time
Ts = 1/1000;  % [s] (1 kHz, matches hardware interface)

%%
%[text] Linearize the torque-input system around the inverted position
%[text] (same as torque design — reused as starting point for transformation).
A_tau = EOM.nlss.A_sym;  % Jacobian df/dx (precomputed)
B_tau = EOM.nlss.B_sym;  % Jacobian df/du (precomputed)

A_tau_lin = subs(A_tau, {q(2), v(2), v(1)}, {pi, 0, 0});
B_tau_lin = subs(B_tau, {q(2), v(2), v(1)}, {pi, 0, 0});
A_tau_lin = simplify(A_tau_lin);
B_tau_lin = simplify(B_tau_lin);

%%
%[text] **Transform to acceleration input.**
%[text] With perfect $\\dot{v}\_1 = u$, the torque satisfying row 2 is
%[text] $\\tau\_1 = (u - A\_\\tau(2,:),x);/;B\_\\tau(2)$.
%[text] Substituting back into the full linearized system:
%[text] $$A\_\\alpha = A\_\\tau - B\_\\tau,A\_\\tau(2,:);/;B\_\\tau(2)$$
%[text] $$B\_\\alpha = B\_\\tau;/;B\_\\tau(2)$$
A_lin = simplify(A_tau_lin - B_tau_lin * A_tau_lin(2,:) / B_tau_lin(2)) %[output:0e35b9b1]
B_lin = simplify(B_tau_lin / B_tau_lin(2)) %[output:0b979bf8]

%%
%[text] Set up numerical state space model for linearized inverted pendulum, and design full state feedback
% numerical state-space matrices
A_lin_num = double(subs(A_lin, [g l r m b_1 b_2 Iz_1], [g_num, l_num, r_num, m_num, b1_num, b2_num, Iz_1_num])) %[output:7adfd58e]
B_lin_num = double(subs(B_lin, [r l Iz_1], [r_num, l_num, Iz_1_num])) %[output:0178c2bd]
C_num = [1 0 0 0; 0 0 1 0] %[output:073a34b6]

rank(obsv(A_lin_num, C_num)) %[output:684949af]
rank(ctrb(A_lin_num, B_lin_num)) %[output:8d10868c]

sys = ss(A_lin_num, B_lin_num, C_num, 0) %[output:1424cc9b]

Ts = 1/1000; % 1000Hz sample rate
sysd = c2d(sys, Ts, 'zoh');
A_lin_d_num = sysd.A;
B_lin_d_num = sysd.B;
C_lin_d_num = sysd.C;

% LQR — same Q, R scaled for acceleration input
% Equivalence: R_accel * alpha^2 = R_torque * tau^2, tau ≈ Iz_1 * alpha at equilibrium
Q = diag([100, 1, 10, 1]);
R = 50000 * Iz_1_num^2;
K = lqr(sys, Q, R) %[output:92b362e5]
Kd = lqrd(A_lin_num, B_lin_num, Q, R, Ts) %[output:48e9605d]

ctrpoles = eig(A_lin_num - B_lin_num*K) %[output:0e9e2029]

% Feedforward gain for DC tracking of q1 reference
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C;
C_ref = [1 0 0 0];  % track q1 only
Nbar = -1 / (C_ref * ((Ad - Bd*Kd - eye(4)) \ Bd)) %[output:10e459be]
%%
%[text] 
%[text] Save controller design data to struct and .mat file

design_accel = struct();

% --- Metadata: traceability (strings, timestamps, non-codegen) ---
design_accel.meta.created        = datetime('now');
design_accel.meta.matlab_version = version;
design_accel.meta.script         = mfilename('fullpath');
design_accel.meta.description    = 'FSFB acceleration control (assumes perfect axis-1 accel tracking via ctrl_axis1_acc)';
design_accel.meta.controller.poles.ct = eig(A_lin_num - B_lin_num*K);
design_accel.meta.controller.poles.dt = eig(Ad - Bd*Kd);

% --- MATLAB objects (useful in MATLAB, not codegen-safe) ---
design_accel.mlobj.plant.ct = sys;
design_accel.mlobj.plant.dt = sysd;

% --- Numerical parameters (codegen-safe: doubles only) ---
design_accel.par.plant.m_kg       = m_num;
design_accel.par.plant.l_m        = l_num;
design_accel.par.plant.r_m        = r_num;
design_accel.par.plant.g_mps2     = g_num;
design_accel.par.plant.b1_Nmsrad  = b1_num;
design_accel.par.plant.b2_Nmsrad  = b2_num;
design_accel.par.plant.Iz1_kgm2   = Iz_1_num;
design_accel.par.plant.q1_cpt     = q1_cpt;
design_accel.par.plant.q2_cpt     = q2_cpt;
design_accel.par.plant.u_sat      = u_sat / Iz_1_num;  % [rad/s^2] accel saturation at equilibrium
design_accel.par.plant.z_eq       = [0; 0; pi; 0];
design_accel.par.plant.Ad         = A_lin_d_num;
design_accel.par.plant.Bd         = B_lin_d_num;
design_accel.par.plant.Cd         = C_lin_d_num;

design_accel.par.Ts = Ts;

design_accel.par.lqr.Q  = Q;
design_accel.par.lqr.R  = R;
design_accel.par.lqr.K  = K;
design_accel.par.lqr.Kd = Kd;
design_accel.par.Nbar    = Nbar;
%%
% Save
if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
save_dir = fullfile(prj.RootFolder, 'data');
if ~isfolder(save_dir), mkdir(save_dir); end
save(fullfile(save_dir, 'FSFB_accel_design.mat'), 'design_accel');
fprintf('Parameters saved to: %s\n', fullfile(save_dir, 'FSFB_accel_design.mat')); %[output:4b89a238]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":31}
%---
%[output:0e35b9b1]
%   data: {"dataType":"symbolic","outputData":{"name":"A_lin","value":"\\left(\\begin{array}{cccc}\n0 & 1 & 0 & 0\\\\\n0 & 0 & 0 & 0\\\\\n0 & 0 & 0 & 1\\\\\n0 & 0 & \\frac{g}{l} & -\\frac{b_2 }{l^2 \\,m}\n\\end{array}\\right)"}}
%---
%[output:0b979bf8]
%   data: {"dataType":"symbolic","outputData":{"name":"B_lin","value":"\\left(\\begin{array}{c}\n0\\\\\n1\\\\\n0\\\\\n\\frac{r}{l}\n\\end{array}\\right)"}}
%---
%[output:7adfd58e]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"A_lin_num","rows":4,"type":"double","value":[["0","1.0000","0","0"],["0","0","0","0"],["0","0","0","1.0000"],["0","0","38.7615","-0.0372"]]}}
%---
%[output:0178c2bd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"B_lin_num","rows":4,"type":"double","value":[["0"],["1.0000"],["0"],["0.5968"]]}}
%---
%[output:073a34b6]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"C_num","rows":2,"type":"double","value":[["1","0","0","0"],["0","0","1","0"]]}}
%---
%[output:684949af]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:8d10868c]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:1424cc9b]
%   data: {"dataType":"text","outputData":{"text":"\nsys =\n \n  A = \n            x1       x2       x3       x4\n   x1        0        1        0        0\n   x2        0        0        0        0\n   x3        0        0        0        1\n   x4        0        0    38.76  -0.0372\n \n  B = \n           u1\n   x1       0\n   x2       1\n   x3       0\n   x4  0.5968\n \n  C = \n       x1  x2  x3  x4\n   y1   1   0   0   0\n   y2   0   0   1   0\n \n  D = \n       u1\n   y1   0\n   y2   0\n \nContinuous-time state-space model.\n<a href=\"matlab:disp(char([10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 65 58 32 91 52 215 52 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 66 58 32 91 52 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 67 58 32 91 50 215 52 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 68 58 32 91 50 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 69 58 32 91 93 10 32 32 32 32 32 32 32 32 32 32 79 102 102 115 101 116 115 58 32 91 93 10 32 32 32 32 32 32 32 32 32 32 32 83 99 97 108 101 100 58 32 48 10 32 32 32 32 32 32 32 32 83 116 97 116 101 78 97 109 101 58 32 123 52 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 32 32 83 116 97 116 101 80 97 116 104 58 32 123 52 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 32 32 83 116 97 116 101 85 110 105 116 58 32 123 52 215 49 32 99 101 108 108 125 10 32 32 32 32 73 110 116 101 114 110 97 108 68 101 108 97 121 58 32 91 48 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 73 110 112 117 116 68 101 108 97 121 58 32 48 10 32 32 32 32 32 32 79 117 116 112 117 116 68 101 108 97 121 58 32 91 50 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 73 110 112 117 116 78 97 109 101 58 32 123 39 39 125 10 32 32 32 32 32 32 32 32 73 110 112 117 116 85 110 105 116 58 32 123 39 39 125 10 32 32 32 32 32 32 32 73 110 112 117 116 71 114 111 117 112 58 32 91 49 215 49 32 115 116 114 117 99 116 93 10 32 32 32 32 32 32 32 79 117 116 112 117 116 78 97 109 101 58 32 123 50 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 32 79 117 116 112 117 116 85 110 105 116 58 32 123 50 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 79 117 116 112 117 116 71 114 111 117 112 58 32 91 49 215 49 32 115 116 114 117 99 116 93 10 32 32 32 32 32 32 32 32 32 32 32 32 78 111 116 101 115 58 32 91 48 215 49 32 115 116 114 105 110 103 93 10 32 32 32 32 32 32 32 32 32 85 115 101 114 68 97 116 97 58 32 91 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 78 97 109 101 58 32 39 39 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 84 115 58 32 48 10 32 32 32 32 32 32 32 32 32 84 105 109 101 85 110 105 116 58 32 39 115 101 99 111 110 100 115 39 10 32 32 32 32 32 83 97 109 112 108 105 110 103 71 114 105 100 58 32 91 49 215 49 32 115 116 114 117 99 116 93 10]))\">Model Properties<\/a>\n","truncated":false}}
%---
%[output:92b362e5]
%   data: {"dataType":"matrix","outputData":{"columns":4,"exponent":"3","name":"K","rows":1,"type":"double","value":[["-1.4907","-0.6505","8.7975","1.4147"]]}}
%---
%[output:48e9605d]
%   data: {"dataType":"matrix","outputData":{"columns":4,"exponent":"3","name":"Kd","rows":1,"type":"double","value":[["-1.3562","-0.5925","8.0293","1.2912"]]}}
%---
%[output:0e9e2029]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"ctrpoles","rows":4,"type":"complex","value":[["-1.7344 + 0.0000i"],["-0.0753 + 0.0216i"],["-0.0753 - 0.0216i"],["-0.0543 + 0.0000i"]]}}
%---
%[output:10e459be]
%   data: {"dataType":"textualVariable","outputData":{"name":"Nbar","value":"-1.3562e+03"}}
%---
%[output:4b89a238]
%   data: {"dataType":"text","outputData":{"text":"Parameters saved to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\FSFB_accel_design.mat\n","truncated":false}}
%---
