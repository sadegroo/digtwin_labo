clear
clc
run RRpendulum_forkin_dyn_noimage.m %[output:6c011544] %[output:65ac0709] %[output:2a386ea2] %[output:0db2a197] %[output:61f18245] %[output:5a82e310] %[output:0d891464] %[output:759f949b] %[output:2c8c7188] %[output:52f465bd] %[output:09fe1bdf] %[output:0fdf442e] %[output:18b521a1] %[output:3f444538]
run RRpendulum_Parameters_num.m

% define numerical values for all parameters (if override wanted)
% m_num = 0.05; % kg
% l_num = 0.235; % m
% r_num = 0.14; % m
% g_num = 9.80665; % m/s²
% b1_num = 0.001; %Nms/rad
% b2_num = 0.001; %Nms/rad
% satlimits = [64.34 19.63]; % [rad/s², rad/s]

% Define initial conditions
q1_0 = 0;
v1_0 = 0;
q2_0 = pi+0.1; % Initial condition for q2
v2_0 = 0;   % Initial velocity for q2
z0 = [q1_0; v1_0; q2_0; v2_0];  % Initial state vector
%%
%[text] Linearize the system around the inverted position
%[text] Uso only second ODE, because v\_dot1 (accel axis 1) is input (tau1 is an output).
x = [q(1); v(1); q(2); v(2)];   % State variables [q_2, v_2]
u = v_dot(1); % Inputs [dot(v_1)]

% First-order system
f1 = v(1) % q1_dot = v1 %[output:54181e51]
f2 = v_dot(1)   %[output:8ccd146c]
f3 = v(2)  % q2_dot = v2 %[output:9b292870]
f4 = solve(solve_EOM(2), v_dot(2)); %[output:217983a8]
f = [f1;f2;f3;f4];  % Vector field

% Compute the Jacobians
A = jacobian(f, x)  % Jacobian of f with respect to states %[output:47a9e483]
B = jacobian(f, u)  % Jacobian of f with respect to inputs %[output:1d6c7a42]

% Linearize around q2 = pi, v2 = 0, v1 = 0, dot(v1) = 0 (inverted pendulum
% position)
A_lin = subs(A, {q(2), v(2), v(1), v_dot(1)}, {pi, 0, 0, 0}) %[output:077e48c8]
B_lin = subs(B, {q(2), v(2), v(1), v_dot(1)}, {pi, 0, 0, 0}) %[output:8e8ef508]

% Simplify the results
A_lin = simplify(A_lin);
B_lin = simplify(B_lin);

% Display the linearized system matrices
A_lin %[output:2e7d0c07]
B_lin %[output:1c1414ad]

%%
%[text] Set up numerical state space model for linearized inverted pendulum, and design full state feedback
% numerical state-space matrices
A_lin_num = double(subs(A_lin, [g l m b_2], [g_num, l_num, m_num, b2_num])) %[output:3addf324]
B_lin_num = double(subs(B_lin, [r l], [r_num, l_num])) %[output:6f1b0f87]
C_num = [1 0 0 0; 0 0 1 0] % we can observe positions (q1 with stepper count, q2 with encoder==> observable system) %[output:5aa6684b]
rank(obsv(A_lin_num, C_num)) % must be 4 to be observable %[output:99c41761]
rank(ctrb(A_lin_num, B_lin_num)) % % must be 4 to be fully controlable %[output:76e250c0]
    
sys = ss(A_lin_num, B_lin_num, C_num, 0) %[output:9cb45fec]

Ts = 1/2000; % 2000Hz sample rate
sysd = c2d(sys,Ts, 'zoh') %[output:90dc9494]

% set up state feedback
Q = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];  % State cost matrix
R = 5;             % Control effort cost
K = lqr(sys, Q, R)  % LQR feedback gain %[output:6598d570]
Kd = lqrd(A_lin_num, B_lin_num, Q, R, Ts)  % LQR feedback gain discrete %[output:1ed7ac47]

ctrpoles = eig(A_lin_num - B_lin_num*K) %[output:0161382f]
%%
%[text] Luenberger observer (does not seem to work...)
obspoles = 10*ctrpoles % rule of thumb, make observer 10x faster %[output:2f9d2848]
L =  transpose(place(A_lin_num', C_num', obspoles)) %[output:0cc0d2eb]
eig(A_lin_num - L*C_num) %[output:4eec6499]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":32.8}
%---
%[output:6c011544]
%   data: {"dataType":"symbolic","outputData":{"name":"DH_table","value":"\\left(\\begin{array}{cccc}\n-\\frac{\\pi }{2} & 0 & 0 & q_1 \\left(t\\right)\\\\\n\\frac{\\pi }{2} & 0 & r & q_2 \\left(t\\right)\n\\end{array}\\right)"}}
%---
%[output:65ac0709]
%   data: {"dataType":"symbolic","outputData":{"name":"o2Pm","value":"\\left(\\begin{array}{c}\n0\\\\\n0\\\\\n-l\\\\\n1\n\\end{array}\\right)"}}
%---
%[output:2a386ea2]
%   data: {"dataType":"symbolic","outputData":{"name":"o0Pm","value":"\\left(\\begin{array}{c}\n-r\\,\\sin \\left(q_1 \\left(t\\right)\\right)-l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\\\\nr\\,\\cos \\left(q_1 \\left(t\\right)\\right)-l\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\\\\n-l\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\\\\n1\n\\end{array}\\right)"}}
%---
%[output:0db2a197]
%   data: {"dataType":"symbolic","outputData":{"name":"jac_m","value":"\\left(\\begin{array}{cc}\nl\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)-r\\,\\cos \\left(q_1 \\left(t\\right)\\right) & -l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\\\\n-r\\,\\sin \\left(q_1 \\left(t\\right)\\right)-l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right) & -l\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\\\\n0 & l\\,\\sin \\left(q_2 \\left(t\\right)\\right)\n\\end{array}\\right)"}}
%---
%[output:61f18245]
%   data: {"dataType":"symbolic","outputData":{"name":"Ekin","value":"\\frac{{\\textrm{Iz}}_1 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\right)}}^2 }{2}+\\frac{m\\,r^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}-\\frac{l^2 \\,m\\,{\\cos \\left(q_2 \\left(t\\right)\\right)}^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+l\\,m\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)"}}
%---
%[output:5a82e310]
%   data: {"dataType":"symbolic","outputData":{"name":"Epot","value":"-g\\,l\\,m\\,\\cos \\left(q_2 \\left(t\\right)\\right)"}}
%---
%[output:0d891464]
%   data: {"dataType":"symbolic","outputData":{"name":"L","value":"\\frac{{\\textrm{Iz}}_1 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\right)}}^2 }{2}+\\frac{m\\,r^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}-\\frac{l^2 \\,m\\,{\\cos \\left(q_2 \\left(t\\right)\\right)}^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+g\\,l\\,m\\,\\cos \\left(q_2 \\left(t\\right)\\right)+l\\,m\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)"}}
%---
%[output:759f949b]
%   data: {"dataType":"symbolic","outputData":{"name":"Q","value":"\\left(\\begin{array}{c}\n\\tau_1 -b_1 \\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\\\\n-b_2 \\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\n\\end{array}\\right)"}}
%---
%[output:2c8c7188]
%   data: {"dataType":"symbolic","outputData":{"name":"tau","value":"\\left(\\begin{array}{c}\n\\tau_1 \\\\\n0\n\\end{array}\\right)"}}
%---
%[output:52f465bd]
%   data: {"dataType":"symbolic","outputData":{"name":"EOM","value":"\\left(\\begin{array}{c}\n-m\\,l^2 \\,{\\cos \\left(q_2 \\left(t\\right)\\right)}^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)+m\\,l^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)+m\\,l\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_2 \\left(t\\right)+m\\,r^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)+{\\textrm{Iz}}_1 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)-m\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\,l\\,r\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\right)}}^2 +m\\,\\sin \\left(2\\,q_2 \\left(t\\right)\\right)\\,l^2 \\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)+b_1 \\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)-\\tau_1 \\\\\nm\\,l^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_2 \\left(t\\right)+m\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,l\\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)-m\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\,l^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 +b_2 \\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)+g\\,m\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\,l\n\\end{array}\\right)"}}
%---
%[output:09fe1bdf]
%   data: {"dataType":"symbolic","outputData":{"name":"solve_EOM","value":"\\left(\\begin{array}{c}\nm\\,v_1 \\,\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,v_2 -m\\,{\\dot{v} }_1 \\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,{\\dot{v} }_1 \\,l^2 -m\\,\\sin \\left(q_2 \\right)\\,l\\,r\\,{v_2 }^2 +m\\,{\\dot{v} }_2 \\,l\\,r\\,\\cos \\left(q_2 \\right)+m\\,{\\dot{v} }_1 \\,r^2 -\\tau_1 +{\\textrm{Iz}}_1 \\,{\\dot{v} }_1 +b_1 \\,v_1 =0\\\\\nb_2 \\,v_2 +l^2 \\,m\\,{\\dot{v} }_2 +g\\,l\\,m\\,\\sin \\left(q_2 \\right)-l^2 \\,m\\,{v_1 }^2 \\,\\cos \\left(q_2 \\right)\\,\\sin \\left(q_2 \\right)+l\\,m\\,r\\,{\\dot{v} }_1 \\,\\cos \\left(q_2 \\right)=0\n\\end{array}\\right)"}}
%---
%[output:0fdf442e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Solutions are only valid under certain conditions. To include parameters and conditions in the solution, specify the 'ReturnConditions' value as 'true'."}}
%---
%[output:18b521a1]
%   data: {"dataType":"symbolic","outputData":{"name":"eqn_vdot1","value":"\\frac{l\\,\\tau_1 -b_1 \\,l\\,v_1 +b_2 \\,r\\,v_2 \\,\\cos \\left(q_2 \\right)+\\frac{g\\,l\\,m\\,r\\,\\sin \\left(2\\,q_2 \\right)}{2}+l^2 \\,m\\,r\\,{v_2 }^2 \\,\\sin \\left(q_2 \\right)-l^2 \\,m\\,r\\,{v_1 }^2 \\,{\\left(\\sin \\left(q_2 \\right)-{\\sin \\left(q_2 \\right)}^3 \\right)}-l^3 \\,m\\,v_1 \\,v_2 \\,\\sin \\left(2\\,q_2 \\right)}{l\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}"}}
%---
%[output:3f444538]
%   data: {"dataType":"symbolic","outputData":{"name":"eqn_vdot2","value":"-\\frac{l^4 \\,m^2 \\,{v_1 }^2 \\,{\\cos \\left(q_2 \\right)}^3 \\,\\sin \\left(q_2 \\right)-\\frac{\\sin \\left(2\\,q_2 \\right)\\,l^4 \\,m^2 \\,{v_1 }^2 }{2}+2\\,l^3 \\,m^2 \\,r\\,v_1 \\,v_2 \\,{\\sin \\left(q_2 \\right)}^3 -2\\,l^3 \\,m^2 \\,r\\,v_1 \\,v_2 \\,\\sin \\left(q_2 \\right)+g\\,l^3 \\,m^2 \\,{\\sin \\left(q_2 \\right)}^3 -\\frac{\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,m^2 \\,r^2 \\,{v_1 }^2 }{2}+\\frac{\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,m^2 \\,r^2 \\,{v_2 }^2 }{2}-\\frac{{\\textrm{Iz}}_1 \\,\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,m\\,{v_1 }^2 }{2}-b_2 \\,l^2 \\,m\\,v_2 \\,{\\cos \\left(q_2 \\right)}^2 +b_2 \\,l^2 \\,m\\,v_2 +g\\,l\\,m^2 \\,r^2 \\,\\sin \\left(q_2 \\right)-b_1 \\,l\\,m\\,r\\,v_1 \\,\\cos \\left(q_2 \\right)+\\tau_1 \\,l\\,m\\,r\\,\\cos \\left(q_2 \\right)+{\\textrm{Iz}}_1 \\,g\\,l\\,m\\,\\sin \\left(q_2 \\right)+b_2 \\,m\\,r^2 \\,v_2 +{\\textrm{Iz}}_1 \\,b_2 \\,v_2 }{l^2 \\,m\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}"}}
%---
%[output:54181e51]
%   data: {"dataType":"symbolic","outputData":{"name":"f1","value":"v_1"}}
%---
%[output:8ccd146c]
%   data: {"dataType":"symbolic","outputData":{"name":"f2","value":"{\\dot{v} }_1"}}
%---
%[output:9b292870]
%   data: {"dataType":"symbolic","outputData":{"name":"f3","value":"v_2"}}
%---
%[output:217983a8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Solutions are only valid under certain conditions. To include parameters and conditions in the solution, specify the 'ReturnConditions' value as 'true'."}}
%---
%[output:47a9e483]
%   data: {"dataType":"symbolic","outputData":{"name":"A","value":"\\left(\\begin{array}{cccc}\n0 & 1 & 0 & 0\\\\\n0 & 0 & 0 & 0\\\\\n0 & 0 & 0 & 1\\\\\n0 & 2\\,v_1 \\,\\cos \\left(q_2 \\right)\\,\\sin \\left(q_2 \\right) & -\\frac{-m\\,l^2 \\,{v_1 }^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 \\,{v_1 }^2 \\,{\\sin \\left(q_2 \\right)}^2 +g\\,m\\,l\\,\\cos \\left(q_2 \\right)-m\\,r\\,{\\dot{v} }_1 \\,l\\,\\sin \\left(q_2 \\right)}{l^2 \\,m} & -\\frac{b_2 }{l^2 \\,m}\n\\end{array}\\right)"}}
%---
%[output:1d6c7a42]
%   data: {"dataType":"symbolic","outputData":{"name":"B","value":"\\left(\\begin{array}{c}\n0\\\\\n1\\\\\n0\\\\\n-\\frac{r\\,\\cos \\left(q_2 \\right)}{l}\n\\end{array}\\right)"}}
%---
%[output:077e48c8]
%   data: {"dataType":"symbolic","outputData":{"name":"A_lin","value":"\\left(\\begin{array}{cccc}\n0 & 1 & 0 & 0\\\\\n0 & 0 & 0 & 0\\\\\n0 & 0 & 0 & 1\\\\\n0 & 0 & \\frac{g}{l} & -\\frac{b_2 }{l^2 \\,m}\n\\end{array}\\right)"}}
%---
%[output:8e8ef508]
%   data: {"dataType":"symbolic","outputData":{"name":"B_lin","value":"\\left(\\begin{array}{c}\n0\\\\\n1\\\\\n0\\\\\n\\frac{r}{l}\n\\end{array}\\right)"}}
%---
%[output:2e7d0c07]
%   data: {"dataType":"symbolic","outputData":{"name":"A_lin","value":"\\left(\\begin{array}{cccc}\n0 & 1 & 0 & 0\\\\\n0 & 0 & 0 & 0\\\\\n0 & 0 & 0 & 1\\\\\n0 & 0 & \\frac{g}{l} & -\\frac{b_2 }{l^2 \\,m}\n\\end{array}\\right)"}}
%---
%[output:1c1414ad]
%   data: {"dataType":"symbolic","outputData":{"name":"B_lin","value":"\\left(\\begin{array}{c}\n0\\\\\n1\\\\\n0\\\\\n\\frac{r}{l}\n\\end{array}\\right)"}}
%---
%[output:3addf324]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"A_lin_num","rows":4,"type":"double","value":[["0","1.0000","0","0"],["0","0","0","0"],["0","0","0","1.0000"],["0","0","38.7615","-0.0372"]]}}
%---
%[output:6f1b0f87]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"B_lin_num","rows":4,"type":"double","value":[["0"],["1.0000"],["0"],["0.5968"]]}}
%---
%[output:5aa6684b]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"C_num","rows":2,"type":"double","value":[["1","0","0","0"],["0","0","1","0"]]}}
%---
%[output:99c41761]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:76e250c0]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"4"}}
%---
%[output:9cb45fec]
%   data: {"dataType":"text","outputData":{"text":"\nsys =\n \n  A = \n            x1       x2       x3       x4\n   x1        0        1        0        0\n   x2        0        0        0        0\n   x3        0        0        0        1\n   x4        0        0    38.76  -0.0372\n \n  B = \n           u1\n   x1       0\n   x2       1\n   x3       0\n   x4  0.5968\n \n  C = \n       x1  x2  x3  x4\n   y1   1   0   0   0\n   y2   0   0   1   0\n \n  D = \n       u1\n   y1   0\n   y2   0\n \nContinuous-time state-space model.\n","truncated":false}}
%---
%[output:90dc9494]
%   data: {"dataType":"text","outputData":{"text":"\nsysd =\n \n  A = \n            x1       x2       x3       x4\n   x1        1   0.0005        0        0\n   x2        0        1        0        0\n   x3        0        0        1   0.0005\n   x4        0        0  0.01938        1\n \n  B = \n              u1\n   x1   1.25e-07\n   x2     0.0005\n   x3   7.46e-08\n   x4  0.0002984\n \n  C = \n       x1  x2  x3  x4\n   y1   1   0   0   0\n   y2   0   0   1   0\n \n  D = \n       u1\n   y1   0\n   y2   0\n \nSample time: 0.0005 seconds\nDiscrete-time state-space model.\n","truncated":false}}
%---
%[output:6598d570]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"K","rows":1,"type":"double","value":[["-0.4472","-1.1903","153.2940","24.5526"]]}}
%---
%[output:1ed7ac47]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"Kd","rows":1,"type":"double","value":[["-0.4457","-1.1864","153.0164","24.5081"]]}}
%---
%[output:0161382f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"ctrpoles","rows":4,"type":"complex","value":[["-6.3607 + 0.0000i"],["-6.0940 + 0.0000i"],["-0.5231 + 0.4167i"],["-0.5231 - 0.4167i"]]}}
%---
%[output:2f9d2848]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"obspoles","rows":4,"type":"complex","value":[["-63.6072 + 0.0000i"],["-60.9400 + 0.0000i"],["-5.2307 + 4.1666i"],["-5.2307 - 4.1666i"]]}}
%---
%[output:0cc0d2eb]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"L","rows":4,"type":"double","value":[["67.9104","0.4865"],["296.8887","73.0082"],["-15.3500","67.0609"],["-898.2208","399.1215"]]}}
%---
%[output:4eec6499]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"ans","rows":4,"type":"complex","value":[["-5.2307 + 4.1666i"],["-5.2307 - 4.1666i"],["-63.6072 + 0.0000i"],["-60.9400 + 0.0000i"]]}}
%---
