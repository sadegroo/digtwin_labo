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
m_num   = params.mechanism.m;
l_num   = params.mechanism.l;
r_num   = params.mechanism.r;
g_num  = params.mechanism.g;
b1_num  = params.mechanism.b1;
b2_num  = params.mechanism.b2;
Iz_1_num = params.mechanism.Iz_1;

% Sensing
q1_cpt = params.sensing.q1_cpt;
q2_cpt = params.sensing.q2_cpt;

% Actuation
u_sat = params.actuation.u_sat;

% Sample time
Ts = 1/1000;  % [s] (1 kHz, matches hardware interface)

%%
%[text] Linearize the system around the inverted position.
%[text] Use both ODE's and torque as input
% Compute the Jacobians
A = EOM.nlss.A_sym;  % Jacobian df/dx (precomputed)
B = EOM.nlss.B_sym;  % Jacobian df/du (precomputed)

% Linearize around q2 = pi, v2 = 0, v1 = 0, dot(v1) = 0 (inverted pendulum
% position)
A_lin = subs(A, {q(2), v(2), v(1)}, {pi, 0, 0}) %[output:7c01c9ee]
B_lin = subs(B, {q(2), v(2), v(1)}, {pi, 0, 0}) %[output:37ed93f0]

% Simplify the results
A_lin = simplify(A_lin);
B_lin = simplify(B_lin);

% Display the linearized system matrices
A_lin %[output:2e3a9561]
B_lin %[output:90888d91]

%%
%[text] Set up numerical state space model for linearized inverted pendulum, and design full state feedback
% numerical state-space matrices
A_lin_num = double(subs(A_lin, [g l r m b_1 b_2 Iz_1], [g_num, l_num,r_num, m_num, b1_num, b2_num, Iz_1_num])) %[output:756fa914]
B_lin_num = double(subs(B_lin, [r l Iz_1], [r_num, l_num, Iz_1_num])) %[output:4b7e4224]
C_num = [1 0 0 0; 0 0 1 0] % we can observe positions (q1 with stepper count, q2 with encoder==> observable system) %[output:73bf36a7]

rank(obsv(A_lin_num, C_num)) % must be 4 to be observable %[output:05f06767]
rank(ctrb(A_lin_num, B_lin_num)) % % must be 4 to be fully controlable %[output:2db56f0b]
    
sys = ss(A_lin_num, B_lin_num, C_num, 0) %[output:951fb1bf]

Ts = 1/1000; % 1000Hz sample rate
sysd = c2d(sys,Ts, 'zoh');
A_lin_d_num = sysd.A;
B_lin_d_num = sysd.B;
C_lin_d_num = sysd.C;

% set up state feedback
Q = diag([100, 1, 10, 1]);  % State cost matrix (Q(1,1) penalizes q1 tracking error)
R = 50000;             % Control effort cost
K = lqr(sys, Q, R)  % LQR feedback gain %[output:657ac2b7]
Kd = lqrd(A_lin_num, B_lin_num, Q, R, Ts)  % LQR feedback gain discrete %[output:1b53e105]

ctrpoles = eig(A_lin_num - B_lin_num*K) %[output:2c7562df]

% Feedforward gain for DC tracking of q1 reference
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C;
C_ref = [1 0 0 0];  % track q1 only
Nbar = -1 / (C_ref * ((Ad - Bd*Kd - eye(4)) \ Bd)) %[output:953bf282]
%%
%[text] 
%[text] Save controller design data to struct and .mat file

design = struct();

% --- Metadata: traceability ---
design.meta.created        = datetime('now');
design.meta.matlab_version = version;
design.meta.script         = mfilename('fullpath');
design.meta.description    = 'FSFB torque control with decoupled nonlinear observers for RR inverted pendulum';

% --- Plant ---
design.plant.params.m_kg    = m_num;
design.plant.params.l_m     = l_num;
design.plant.params.r_m     = r_num;
design.plant.params.g_mps2  = g_num;
design.plant.params.b1_Nmsrad = b1_num;
design.plant.params.b2_Nmsrad = b2_num;
design.plant.params.Iz1_kgm2 = Iz_1_num;
design.plant.params.q1_cpt = q1_cpt;
design.plant.params.q2_cpt = q2_cpt;

design.plant.u_sat      = u_sat;   % [rad/s², rad/s]
design.plant.linearization.z_eq = [0; 0; pi; 0]; % equilibrium [q1;v1;q2;v2]

design.plant.ct = sys;                     % continuous-time ss
design.plant.dt = sysd;                    % discrete-time ss (ZOH)
design.plant.Ts = Ts;

% --- Controller ---
design.controller.lqr.Q  = Q;
design.controller.lqr.R  = R;
design.controller.lqr.K  = K;             % continuous LQR gain
design.controller.lqr.Kd = Kd;            % discrete LQR gain (lqrd)
design.controller.Nbar    = Nbar;          % feedforward gain for q1 tracking

design.controller.poles.ct = eig(A_lin_num - B_lin_num*K);   % continuous CL
design.controller.poles.dt = eig(Ad - Bd*Kd);                % discrete CL
%%

% --- Save ---
save_path = fullfile('C:\Users\u0130154\MATLAB\projects\digtwin_labo\data', 'FSFB_torque_design.mat');
save(save_path, 'design');
fprintf('Design saved to: %s\n', save_path); %[output:7839b6ac]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":13}
%---
%[output:7c01c9ee]
%   data: {"dataType":"symbolic","outputData":{"name":"A_lin","value":"\\left(\\begin{array}{cccc}\n0 & 1 & 0 & 0\\\\\n0 & -\\frac{b_1 }{{\\textrm{Iz}}_1 } & \\frac{g\\,m\\,r}{{\\textrm{Iz}}_1 } & -\\frac{b_2 \\,r}{{\\textrm{Iz}}_1 \\,l}\\\\\n0 & 0 & 0 & 1\\\\\n0 & -\\frac{b_1 \\,r}{{\\textrm{Iz}}_1 \\,l} & \\frac{g\\,l\\,m^2 \\,r^2 +{\\textrm{Iz}}_1 \\,g\\,l\\,m}{{\\textrm{Iz}}_1 \\,l^2 \\,m} & -\\frac{b_2 \\,m\\,r^2 +{\\textrm{Iz}}_1 \\,b_2 }{{\\textrm{Iz}}_1 \\,l^2 \\,m}\n\\end{array}\\right)"}}
%---
%[output:37ed93f0]
%   data: {"dataType":"symbolic","outputData":{"name":"B_lin","value":"\\left(\\begin{array}{c}\n0\\\\\n\\frac{1}{{\\textrm{Iz}}_1 }\\\\\n0\\\\\n\\frac{r}{{\\textrm{Iz}}_1 \\,l}\n\\end{array}\\right)"}}
%---
%[output:2e3a9561]
%   data: {"dataType":"symbolic","outputData":{"name":"A_lin","value":"\\left(\\begin{array}{cccc}\n0 & 1 & 0 & 0\\\\\n0 & -\\frac{b_1 }{{\\textrm{Iz}}_1 } & \\frac{g\\,m\\,r}{{\\textrm{Iz}}_1 } & -\\frac{b_2 \\,r}{{\\textrm{Iz}}_1 \\,l}\\\\\n0 & 0 & 0 & 1\\\\\n0 & -\\frac{b_1 \\,r}{{\\textrm{Iz}}_1 \\,l} & \\frac{g\\,{\\left(m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}{{\\textrm{Iz}}_1 \\,l} & -\\frac{b_2 \\,{\\left(m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}{{\\textrm{Iz}}_1 \\,l^2 \\,m}\n\\end{array}\\right)"}}
%---
%[output:90888d91]
%   data: {"dataType":"symbolic","outputData":{"name":"B_lin","value":"\\left(\\begin{array}{c}\n0\\\\\n\\frac{1}{{\\textrm{Iz}}_1 }\\\\\n0\\\\\n\\frac{r}{{\\textrm{Iz}}_1 \\,l}\n\\end{array}\\right)"}}
%---
%[output:756fa914]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"A_lin_num","rows":4,"type":"double","value":[["0","1.0000","0","0"],["0","-3.3333","621.9377","-0.5968"],["0","0","0","1.0000"],["0","-1.9895","409.9575","-0.3934"]]}}
%---
%[output:4b7e4224]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"4","name":"B_lin_num","rows":4,"type":"double","value":[["0"],["3.3333"],["0"],["1.9895"]]}}
%---
%[output:73bf36a7]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"C_num","rows":2,"type":"double","value":[["1","0","0","0"],["0","0","1","0"]]}}
%---
%[output:05f06767]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:2db56f0b]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:951fb1bf]
%   data: {"dataType":"text","outputData":{"text":"\nsys =\n \n  A = \n            x1       x2       x3       x4\n   x1        0        1        0        0\n   x2        0   -3.333    621.9  -0.5968\n   x3        0        0        0        1\n   x4        0   -1.989      410  -0.3934\n \n  B = \n              u1\n   x1          0\n   x2  3.333e+04\n   x3          0\n   x4  1.989e+04\n \n  C = \n       x1  x2  x3  x4\n   y1   1   0   0   0\n   y2   0   0   1   0\n \n  D = \n       u1\n   y1   0\n   y2   0\n \nContinuous-time state-space model.\n<a href=\"matlab:disp(char([10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 65 58 32 91 52 215 52 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 66 58 32 91 52 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 67 58 32 91 50 215 52 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 68 58 32 91 50 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 69 58 32 91 93 10 32 32 32 32 32 32 32 32 32 32 79 102 102 115 101 116 115 58 32 91 93 10 32 32 32 32 32 32 32 32 32 32 32 83 99 97 108 101 100 58 32 48 10 32 32 32 32 32 32 32 32 83 116 97 116 101 78 97 109 101 58 32 123 52 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 32 32 83 116 97 116 101 80 97 116 104 58 32 123 52 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 32 32 83 116 97 116 101 85 110 105 116 58 32 123 52 215 49 32 99 101 108 108 125 10 32 32 32 32 73 110 116 101 114 110 97 108 68 101 108 97 121 58 32 91 48 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 73 110 112 117 116 68 101 108 97 121 58 32 48 10 32 32 32 32 32 32 79 117 116 112 117 116 68 101 108 97 121 58 32 91 50 215 49 32 100 111 117 98 108 101 93 10 32 32 32 32 32 32 32 32 73 110 112 117 116 78 97 109 101 58 32 123 39 39 125 10 32 32 32 32 32 32 32 32 73 110 112 117 116 85 110 105 116 58 32 123 39 39 125 10 32 32 32 32 32 32 32 73 110 112 117 116 71 114 111 117 112 58 32 91 49 215 49 32 115 116 114 117 99 116 93 10 32 32 32 32 32 32 32 79 117 116 112 117 116 78 97 109 101 58 32 123 50 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 32 79 117 116 112 117 116 85 110 105 116 58 32 123 50 215 49 32 99 101 108 108 125 10 32 32 32 32 32 32 79 117 116 112 117 116 71 114 111 117 112 58 32 91 49 215 49 32 115 116 114 117 99 116 93 10 32 32 32 32 32 32 32 32 32 32 32 32 78 111 116 101 115 58 32 91 48 215 49 32 115 116 114 105 110 103 93 10 32 32 32 32 32 32 32 32 32 85 115 101 114 68 97 116 97 58 32 91 93 10 32 32 32 32 32 32 32 32 32 32 32 32 32 78 97 109 101 58 32 39 39 10 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 84 115 58 32 48 10 32 32 32 32 32 32 32 32 32 84 105 109 101 85 110 105 116 58 32 39 115 101 99 111 110 100 115 39 10 32 32 32 32 32 83 97 109 112 108 105 110 103 71 114 105 100 58 32 91 49 215 49 32 115 116 114 117 99 116 93 10]))\">Model Properties<\/a>\n","truncated":false}}
%---
%[output:657ac2b7]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"K","rows":1,"type":"double","value":[["-0.0447","-0.0197","0.2847","0.0427"]]}}
%---
%[output:1b53e105]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"Kd","rows":1,"type":"double","value":[["-0.0407","-0.0180","0.2617","0.0390"]]}}
%---
%[output:2c7562df]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"ctrpoles","rows":4,"type":"complex","value":[["-1.7560 + 0.0000i"],["-0.0736 + 0.0171i"],["-0.0736 - 0.0171i"],["-0.0576 + 0.0000i"]]}}
%---
%[output:953bf282]
%   data: {"dataType":"textualVariable","outputData":{"name":"Nbar","value":"-0.0407"}}
%---
%[output:7839b6ac]
%   data: {"dataType":"text","outputData":{"text":"Design saved to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\FSFB_torque_design.mat\n","truncated":false}}
%---
