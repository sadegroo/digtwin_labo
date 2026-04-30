clear
clc
%%

%Define symbolic variables
syms m g t b_1 b_2 Iz_1 real positive
syms r l tau_1 real
q_t = arrayfun(@(i) str2sym(['q_' num2str(i) '(t)']), 1:2);
q = arrayfun(@(i) str2sym(['q_' num2str(i)]), 1:2);
v = arrayfun(@(i) str2sym(['v_' num2str(i)]), 1:2);
v_t = arrayfun(@(i) str2sym(['v_' num2str(i) '(t)']), 1:2);
v_dot = arrayfun(@(i) str2sym(['v_dot_' num2str(i)]), 1:2);
assume(q_t, 'real');
assume(q, 'real');
assume(v, 'real');
assume(v_t, 'real');
assume(v_dot, 'real');

q_t=transpose(q_t);
q=transpose(q);
v=transpose(v);
v_dot=transpose(v_dot);
v_t =transpose(v_t);

q_t_dot=diff(q_t,t);
q_t_ddot=diff(q_t_dot,t);

v_t_dot = diff(v_t,t);

assume(q_t_dot, 'real');
assume(q_t_ddot, 'real');
assume(v_t_dot, 'real');
%%
%forward kinematics
DH_table= [-pi/2, 0,0,q_t(1);... %[output:group:8900f75c] %[output:8a914992]
            pi/2, 0, r, q_t(2)] %[output:group:8900f75c] %[output:8a914992]

[Tfull,Tparts,Tcumul ] = DH_full(DH_table);

o2Pm = [0;0;-l;1] %[output:4a618d3d]

o0Pm = Tfull*o2Pm %[output:5765cd89]
jac_m=jacobian(o0Pm(1:3),q_t) %[output:3bc22a4d]

%forward kinematics (no (t))
DH_table_= [-pi/2, 0,0,q(1);... %[output:group:3aa7b076] %[output:5f1c323f]
            pi/2, 0, r, q(2)] %[output:group:3aa7b076] %[output:5f1c323f]

[Tfull_,Tparts_,Tcumul_ ] = DH_full(DH_table_);

o2Pm_ = [0;0;-l;1] %[output:19e27f24]

o0Pm_ = Tfull_*o2Pm_ %[output:60b512d1]
jac_m_=jacobian(o0Pm_(1:3),q) %[output:9c433d75]

%[text] Helper function to find the moment of inertia of mass auround the motor axis 1
totalIz1 = m*(o0Pm(1)^2+o0Pm(2)^2)+Iz_1 %[output:69bf69b0]
totalIz1 = subs(totalIz1,q_t,q);
prj = matlab.project.rootProject;
func_dir = fullfile(prj.RootFolder, 'resources', 'functions');
f_path = fullfile(func_dir, 'RRpendulum_totalIz1.m');
matlabFunction(totalIz1, ...
    'File', f_path, ...
    'Vars', {q,Iz_1,m,r,l}, ...
    'Outputs', {'Iz1tot'}, ...
    'Comments', 'Calculate total inertia around Iz1 axis for certain pose (auto-generated).');

%[text] 
Ekin = simplify(0.5*m*transpose(q_t_dot)*(transpose(jac_m)*jac_m)*q_t_dot) ... % point mass kinetic energy %[output:group:8611da8a] %[output:92505d1a]
    + 0.5*q_t_dot(1)^2*Iz_1    % rotational kinetic energy of first axis %[output:group:8611da8a] %[output:92505d1a]
%[text] Since we modeled a point mass, there is nog contribution of rotational kinetic energy:
Epot = m*g*o0Pm(3,1) %[output:1c4acb6a]

L=Ekin-Epot %Lagrangian %[output:7944a89e]
%[text] Euler-lagrange with friction, but without holonomic constraints. Right hand side = Qi (generalized force i)
% generalized forces "Q", only viscous friction.
Q = [tau_1 - b_1*q_t_dot(1);-b_2*q_t_dot(2)] %[output:141e9be7]
tau=[tau_1;0] %[output:995e8ba5]

% derive equations of motion
[EOM, ~] = derive_EOM(L, q_t, Q) %[output:90a76c85]
%%
%Substitute q1_ddot and q2_ddot into the equations
subs_eqns = subs(EOM, q_t_ddot, v_dot);
subs_eqns2 = subs(subs_eqns, q_t_dot, v);
subs_eqns3 = subs(subs_eqns2, q_t, q);
solve_EOM = subs_eqns3 == 0 %[output:73eb1618]

% Solve for q1_ddot and q2_ddot (i.e., dv1/dt and dv2/dt)
solution = solve(solve_EOM, v_dot); %[output:7347ac3a]

% eqn = v==q_t_dot
eqn_vdot1=simplify(solution.v_dot_1) %[output:0f4bfe99]
eqn_vdot2=simplify(solution.v_dot_2) %[output:273a60cc]
%%
%[text] Formulate as a nonlinear system x\_dot = f(x,u), y=g(x,u)
x = [q(1); v(1); q(2); v(2)];   % State variables [q_2, v_2]
u = tau_1; % Input torque on axis 1

% First-order system
f1 = v(1) % q1_dot = v1 %[output:3f4ba4e1]
f2 = eqn_vdot1;  %q1_ddot = v1_dot
f3 = v(2)  % q2_dot = v2 %[output:3656c39c]
f4 = eqn_vdot2; %q2_ddot = v2_dot
f = [f1;f2;f3;f4];  % Vector field

%%
%[text] Decompose into standard multibody form: M(q)\*q\_ddot + C(q,qdot)*qdot + G(q) = B*tau
%[text] EOM convention: d/dt(dL/dqdot) - dL/dq - Q = 0, with Q = tau - B\_damp\*qdot
%[text] So subs\_eqns3 = M*vdot + coriolis\_terms + G - tau + B\_damp*v

% --- M(q): mass/inertia matrix (coefficients of v_dot in EOM) ---
% equationsToMatrix convention: expr = A*vars - b, so subs_eqns3 = M_mat*v_dot - b
[M_mat, b_rem] = equationsToMatrix(subs_eqns3, v_dot);
M_mat = simplify(M_mat) %[output:5c3dfe22]

% b_rem = M*v_dot - subs_eqns3 = tau - C*v - B_damp*v - G (negated non-inertial terms)

% --- C(q,qdot): Coriolis/centrifugal matrix via Christoffel symbols ---
% c_ijk = 0.5*(dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
% C_ij = sum_k c_ijk * qdot_k
n_dof = length(q);
C_mat = sym(zeros(n_dof));
for i = 1:n_dof
    for j = 1:n_dof
        for k = 1:n_dof
            c_ijk = 0.5*( diff(M_mat(i,j), q(k)) + diff(M_mat(i,k), q(j)) - diff(M_mat(j,k), q(i)) );
            C_mat(i,j) = C_mat(i,j) + c_ijk * v(k);
        end
    end
end
C_mat = simplify(C_mat) %[output:7358dc4e]

% --- Viscous damping matrix ---
B_damp = diag([b_1, b_2]);

% --- G(q): gravity vector ---
% From b_rem at v=0, tau_1=0: b_rem = -G, so G = -b_rem(v=0, tau=0)
G_vec = simplify(-subs(subs(b_rem, v, zeros(size(v))), tau_1, 0)) %[output:9eed4343]

% --- Input matrix B_tau ---
B_tau = [1; 0] %[output:50dbdd01]

% --- Full decomposition: M*vdot + C*v + B_damp*v + G = B_tau*tau_1 ---
disp('M(q):'); disp(M_mat) %[output:042c0fc5] %[output:3ace2f4a]
disp('C(q,qdot):'); disp(C_mat) %[output:0a70b61d] %[output:4ba6c06e]
disp('B_damp:'); disp(B_damp) %[output:6534c84a] %[output:2bcae8db]
disp('G(q):'); disp(G_vec) %[output:9dcbb90c] %[output:828ff114]

% --- Verify: M*vdot + C*v + B_damp*v + G - B_tau*tau_1 should equal subs_eqns3 ---
verify = simplify(M_mat*v_dot + C_mat*v + B_damp*v + G_vec - B_tau*tau_1 - subs_eqns3) %[output:52619c1a]
disp('Verification (should be all zeros):') %[output:51270805]
disp(verify) %[output:71c6d8a1]
%%
%% Save symbolic EOM to .mat file
% Precompute Jacobians of f (expensive — only done once here)
A_sym = jacobian(f, x);   % df/dx (4x4)
B_sym = jacobian(f, u);   % df/du (4x1)

EOM = struct();

% --- Metadata ---
EOM.meta.created          = datetime('now');
EOM.meta.matlab_version   = version;
EOM.meta.source_script    = 'RRpendulum_forkin_dyn_noimage.m';
EOM.meta.description      = 'Symbolic EOM for RR inverted pendulum (Euler-Lagrange)';
EOM.meta.state_convention = 'z = [q1; v1; q2; v2], u = tau_1';
EOM.meta.n_dof            = 2;

% --- Symbolic variables (for subs() in downstream scripts) ---
EOM.sym.x      = x;                           % state vector [q_1; v_1; q_2; v_2]
EOM.sym.u      = u;                           % input tau_1
EOM.sym.q      = q;                           % generalized coordinates [q_1; q_2]
EOM.sym.v      = v;                           % generalized velocities [v_1; v_2]
EOM.sym.params = [g l r m b_1 b_2 Iz_1];     % symbolic parameter vector

% --- Nonlinear state-space: dx/dt = f(x, u) ---
EOM.nlss.f     = f;                           % [v1; vdot1; v2; vdot2]
EOM.nlss.A_sym = A_sym;                       % df/dx
EOM.nlss.B_sym = B_sym;                       % df/du

% --- Multibody form: M(q)*vdot + C(q,v)*v + B_damp*v + G(q) = B_tau*tau ---
EOM.multibody.M      = M_mat;
EOM.multibody.C      = C_mat;
EOM.multibody.G      = G_vec;
EOM.multibody.B_damp = B_damp;
EOM.multibody.B_tau  = B_tau;

% --- split dynamics into actuated and unactuated using Shur complement
a = 1; b = 2;
S    = [eye(numel(a)), -M_mat(a,b)/M_mat(b,b)];
Meff = simplify(S * M_mat(:,a));
h    = simplify(S * ((C_mat + B_damp)*v + G_vec));

matlabFunction(Meff, 'File', fullfile(func_dir, 'compute_Meff.m'), ...
    'Vars', {q, Iz_1,l,m,r}, 'Outputs', {'Meff'});
matlabFunction(h, 'File', fullfile(func_dir, 'compute_h.m'), ...
    'Vars', {q, v, Iz_1,l,m,r,b_1,b_2,g}, 'Outputs', {'h'});

% --- Save ---
if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
save_dir = fullfile(prj.RootFolder, 'data');
if ~isfolder(save_dir), mkdir(save_dir); end
save(fullfile(save_dir, 'RRpendulum_EOM.mat'), 'EOM');
fprintf('Symbolic EOM saved to: %s\n', fullfile(save_dir, 'RRpendulum_EOM.mat')); %[output:0a9895d8]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.5}
%---
%[output:8a914992]
%   data: {"dataType":"symbolic","outputData":{"name":"DH_table","value":"\\left(\\begin{array}{cccc}\n-\\frac{\\pi }{2} & 0 & 0 & q_1 \\left(t\\right)\\\\\n\\frac{\\pi }{2} & 0 & r & q_2 \\left(t\\right)\n\\end{array}\\right)"}}
%---
%[output:4a618d3d]
%   data: {"dataType":"symbolic","outputData":{"name":"o2Pm","value":"\\left(\\begin{array}{c}\n0\\\\\n0\\\\\n-l\\\\\n1\n\\end{array}\\right)"}}
%---
%[output:5765cd89]
%   data: {"dataType":"symbolic","outputData":{"name":"o0Pm","value":"\\left(\\begin{array}{c}\n-r\\,\\sin \\left(q_1 \\left(t\\right)\\right)-l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\\\\nr\\,\\cos \\left(q_1 \\left(t\\right)\\right)-l\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\\\\n-l\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\\\\n1\n\\end{array}\\right)"}}
%---
%[output:3bc22a4d]
%   data: {"dataType":"symbolic","outputData":{"name":"jac_m","value":"\\left(\\begin{array}{cc}\nl\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)-r\\,\\cos \\left(q_1 \\left(t\\right)\\right) & -l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\\\\n-r\\,\\sin \\left(q_1 \\left(t\\right)\\right)-l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right) & -l\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\\\\n0 & l\\,\\sin \\left(q_2 \\left(t\\right)\\right)\n\\end{array}\\right)"}}
%---
%[output:5f1c323f]
%   data: {"dataType":"symbolic","outputData":{"name":"DH_table_","value":"\\left(\\begin{array}{cccc}\n-\\frac{\\pi }{2} & 0 & 0 & q_1 \\\\\n\\frac{\\pi }{2} & 0 & r & q_2 \n\\end{array}\\right)"}}
%---
%[output:19e27f24]
%   data: {"dataType":"symbolic","outputData":{"name":"o2Pm_","value":"\\left(\\begin{array}{c}\n0\\\\\n0\\\\\n-l\\\\\n1\n\\end{array}\\right)"}}
%---
%[output:60b512d1]
%   data: {"dataType":"symbolic","outputData":{"name":"o0Pm_","value":"\\left(\\begin{array}{c}\n-r\\,\\sin \\left(q_1 \\right)-l\\,\\cos \\left(q_1 \\right)\\,\\sin \\left(q_2 \\right)\\\\\nr\\,\\cos \\left(q_1 \\right)-l\\,\\sin \\left(q_1 \\right)\\,\\sin \\left(q_2 \\right)\\\\\n-l\\,\\cos \\left(q_2 \\right)\\\\\n1\n\\end{array}\\right)"}}
%---
%[output:9c433d75]
%   data: {"dataType":"symbolic","outputData":{"name":"jac_m_","value":"\\left(\\begin{array}{cc}\nl\\,\\sin \\left(q_1 \\right)\\,\\sin \\left(q_2 \\right)-r\\,\\cos \\left(q_1 \\right) & -l\\,\\cos \\left(q_1 \\right)\\,\\cos \\left(q_2 \\right)\\\\\n-r\\,\\sin \\left(q_1 \\right)-l\\,\\cos \\left(q_1 \\right)\\,\\sin \\left(q_2 \\right) & -l\\,\\cos \\left(q_2 \\right)\\,\\sin \\left(q_1 \\right)\\\\\n0 & l\\,\\sin \\left(q_2 \\right)\n\\end{array}\\right)"}}
%---
%[output:69bf69b0]
%   data: {"dataType":"symbolic","outputData":{"name":"totalIz1","value":"{\\textrm{Iz}}_1 +m\\,{\\left({{\\left(r\\,\\sin \\left(q_1 \\left(t\\right)\\right)+l\\,\\cos \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\right)}}^2 +{{\\left(r\\,\\cos \\left(q_1 \\left(t\\right)\\right)-l\\,\\sin \\left(q_1 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\right)}}^2 \\right)}"}}
%---
%[output:92505d1a]
%   data: {"dataType":"symbolic","outputData":{"name":"Ekin","value":"\\frac{{\\textrm{Iz}}_1 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\right)}}^2 }{2}+\\frac{m\\,r^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}-\\frac{l^2 \\,m\\,{\\cos \\left(q_2 \\left(t\\right)\\right)}^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+l\\,m\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)"}}
%---
%[output:1c4acb6a]
%   data: {"dataType":"symbolic","outputData":{"name":"Epot","value":"-g\\,l\\,m\\,\\cos \\left(q_2 \\left(t\\right)\\right)"}}
%---
%[output:7944a89e]
%   data: {"dataType":"symbolic","outputData":{"name":"L","value":"\\frac{{\\textrm{Iz}}_1 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+\\frac{l^2 \\,m\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\right)}}^2 }{2}+\\frac{m\\,r^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}-\\frac{l^2 \\,m\\,{\\cos \\left(q_2 \\left(t\\right)\\right)}^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 }{2}+g\\,l\\,m\\,\\cos \\left(q_2 \\left(t\\right)\\right)+l\\,m\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)"}}
%---
%[output:141e9be7]
%   data: {"dataType":"symbolic","outputData":{"name":"Q","value":"\\left(\\begin{array}{c}\n\\tau_1 -b_1 \\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\\\\n-b_2 \\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\n\\end{array}\\right)"}}
%---
%[output:995e8ba5]
%   data: {"dataType":"symbolic","outputData":{"name":"tau","value":"\\left(\\begin{array}{c}\n\\tau_1 \\\\\n0\n\\end{array}\\right)"}}
%---
%[output:90a76c85]
%   data: {"dataType":"symbolic","outputData":{"name":"EOM","value":"\\left(\\begin{array}{c}\n-m\\,l^2 \\,{\\cos \\left(q_2 \\left(t\\right)\\right)}^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)+m\\,l^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)+m\\,l\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_2 \\left(t\\right)+m\\,r^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)+{\\textrm{Iz}}_1 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)-m\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\,l\\,r\\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\right)}}^2 +m\\,\\sin \\left(2\\,q_2 \\left(t\\right)\\right)\\,l^2 \\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)\\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)+b_1 \\,\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)-\\tau_1 \\\\\nm\\,l^2 \\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_2 \\left(t\\right)+m\\,r\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,l\\,\\frac{\\partial^2 }{\\partial t^2 }\\;q_1 \\left(t\\right)-m\\,\\cos \\left(q_2 \\left(t\\right)\\right)\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\,l^2 \\,{{\\left(\\frac{\\partial }{\\partial t}\\;q_1 \\left(t\\right)\\right)}}^2 +b_2 \\,\\frac{\\partial }{\\partial t}\\;q_2 \\left(t\\right)+g\\,m\\,\\sin \\left(q_2 \\left(t\\right)\\right)\\,l\n\\end{array}\\right)"}}
%---
%[output:73eb1618]
%   data: {"dataType":"symbolic","outputData":{"name":"solve_EOM","value":"\\left(\\begin{array}{c}\nm\\,v_1 \\,\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,v_2 -m\\,{\\dot{v} }_1 \\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,{\\dot{v} }_1 \\,l^2 -m\\,\\sin \\left(q_2 \\right)\\,l\\,r\\,{v_2 }^2 +m\\,{\\dot{v} }_2 \\,l\\,r\\,\\cos \\left(q_2 \\right)+m\\,{\\dot{v} }_1 \\,r^2 -\\tau_1 +{\\textrm{Iz}}_1 \\,{\\dot{v} }_1 +b_1 \\,v_1 =0\\\\\nb_2 \\,v_2 +l^2 \\,m\\,{\\dot{v} }_2 +g\\,l\\,m\\,\\sin \\left(q_2 \\right)-l^2 \\,m\\,{v_1 }^2 \\,\\cos \\left(q_2 \\right)\\,\\sin \\left(q_2 \\right)+l\\,m\\,r\\,{\\dot{v} }_1 \\,\\cos \\left(q_2 \\right)=0\n\\end{array}\\right)"}}
%---
%[output:7347ac3a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Solutions are only valid under certain conditions. To include parameters and conditions in the solution, specify the 'ReturnConditions' value as 'true'."}}
%---
%[output:0f4bfe99]
%   data: {"dataType":"symbolic","outputData":{"name":"eqn_vdot1","value":"\\frac{l\\,\\tau_1 -b_1 \\,l\\,v_1 +b_2 \\,r\\,v_2 \\,\\cos \\left(q_2 \\right)+\\frac{g\\,l\\,m\\,r\\,\\sin \\left(2\\,q_2 \\right)}{2}+l^2 \\,m\\,r\\,{v_2 }^2 \\,\\sin \\left(q_2 \\right)-l^2 \\,m\\,r\\,{v_1 }^2 \\,{\\left(\\sin \\left(q_2 \\right)-{\\sin \\left(q_2 \\right)}^3 \\right)}-l^3 \\,m\\,v_1 \\,v_2 \\,\\sin \\left(2\\,q_2 \\right)}{l\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}"}}
%---
%[output:273a60cc]
%   data: {"dataType":"symbolic","outputData":{"name":"eqn_vdot2","value":"-\\frac{l^4 \\,m^2 \\,{v_1 }^2 \\,{\\cos \\left(q_2 \\right)}^3 \\,\\sin \\left(q_2 \\right)-\\frac{\\sin \\left(2\\,q_2 \\right)\\,l^4 \\,m^2 \\,{v_1 }^2 }{2}+2\\,l^3 \\,m^2 \\,r\\,v_1 \\,v_2 \\,{\\sin \\left(q_2 \\right)}^3 -2\\,l^3 \\,m^2 \\,r\\,v_1 \\,v_2 \\,\\sin \\left(q_2 \\right)+g\\,l^3 \\,m^2 \\,{\\sin \\left(q_2 \\right)}^3 -\\frac{\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,m^2 \\,r^2 \\,{v_1 }^2 }{2}+\\frac{\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,m^2 \\,r^2 \\,{v_2 }^2 }{2}-\\frac{{\\textrm{Iz}}_1 \\,\\sin \\left(2\\,q_2 \\right)\\,l^2 \\,m\\,{v_1 }^2 }{2}-b_2 \\,l^2 \\,m\\,v_2 \\,{\\cos \\left(q_2 \\right)}^2 +b_2 \\,l^2 \\,m\\,v_2 +g\\,l\\,m^2 \\,r^2 \\,\\sin \\left(q_2 \\right)-b_1 \\,l\\,m\\,r\\,v_1 \\,\\cos \\left(q_2 \\right)+\\tau_1 \\,l\\,m\\,r\\,\\cos \\left(q_2 \\right)+{\\textrm{Iz}}_1 \\,g\\,l\\,m\\,\\sin \\left(q_2 \\right)+b_2 \\,m\\,r^2 \\,v_2 +{\\textrm{Iz}}_1 \\,b_2 \\,v_2 }{l^2 \\,m\\,{\\left(-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 -m\\,r^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,r^2 +{\\textrm{Iz}}_1 \\right)}}"}}
%---
%[output:3f4ba4e1]
%   data: {"dataType":"symbolic","outputData":{"name":"f1","value":"v_1"}}
%---
%[output:3656c39c]
%   data: {"dataType":"symbolic","outputData":{"name":"f3","value":"v_2"}}
%---
%[output:5c3dfe22]
%   data: {"dataType":"symbolic","outputData":{"name":"M_mat","value":"\\left(\\begin{array}{cc}\n-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 +m\\,r^2 +{\\textrm{Iz}}_1  & l\\,m\\,r\\,\\cos \\left(q_2 \\right)\\\\\nl\\,m\\,r\\,\\cos \\left(q_2 \\right) & l^2 \\,m\n\\end{array}\\right)"}}
%---
%[output:7358dc4e]
%   data: {"dataType":"symbolic","outputData":{"name":"C_mat","value":"\\left(\\begin{array}{cc}\n\\frac{l^2 \\,m\\,v_2 \\,\\sin \\left(2\\,q_2 \\right)}{2} & -\\frac{l\\,m\\,{\\left(2\\,r\\,v_2 \\,\\sin \\left(q_2 \\right)-l\\,v_1 \\,\\sin \\left(2\\,q_2 \\right)\\right)}}{2}\\\\\n-\\frac{l^2 \\,m\\,v_1 \\,\\sin \\left(2\\,q_2 \\right)}{2} & 0\n\\end{array}\\right)"}}
%---
%[output:9eed4343]
%   data: {"dataType":"symbolic","outputData":{"name":"G_vec","value":"\\left(\\begin{array}{c}\n0\\\\\ng\\,l\\,m\\,\\sin \\left(q_2 \\right)\n\\end{array}\\right)"}}
%---
%[output:50dbdd01]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"B_tau","rows":2,"type":"double","value":[["1"],["0"]]}}
%---
%[output:042c0fc5]
%   data: {"dataType":"text","outputData":{"text":"M(q):\n","truncated":false}}
%---
%[output:3ace2f4a]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cc}\n-m\\,l^2 \\,{\\cos \\left(q_2 \\right)}^2 +m\\,l^2 +m\\,r^2 +{\\textrm{Iz}}_1  & l\\,m\\,r\\,\\cos \\left(q_2 \\right)\\\\\nl\\,m\\,r\\,\\cos \\left(q_2 \\right) & l^2 \\,m\n\\end{array}\\right)"}}
%---
%[output:0a70b61d]
%   data: {"dataType":"text","outputData":{"text":"C(q,qdot):\n","truncated":false}}
%---
%[output:4ba6c06e]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cc}\n\\frac{l^2 \\,m\\,v_2 \\,\\sin \\left(2\\,q_2 \\right)}{2} & -\\frac{l\\,m\\,{\\left(2\\,r\\,v_2 \\,\\sin \\left(q_2 \\right)-l\\,v_1 \\,\\sin \\left(2\\,q_2 \\right)\\right)}}{2}\\\\\n-\\frac{l^2 \\,m\\,v_1 \\,\\sin \\left(2\\,q_2 \\right)}{2} & 0\n\\end{array}\\right)"}}
%---
%[output:6534c84a]
%   data: {"dataType":"text","outputData":{"text":"B_damp:\n","truncated":false}}
%---
%[output:2bcae8db]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cc}\nb_1  & 0\\\\\n0 & b_2 \n\\end{array}\\right)"}}
%---
%[output:9dcbb90c]
%   data: {"dataType":"text","outputData":{"text":"G(q):\n","truncated":false}}
%---
%[output:828ff114]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{c}\n0\\\\\ng\\,l\\,m\\,\\sin \\left(q_2 \\right)\n\\end{array}\\right)"}}
%---
%[output:52619c1a]
%   data: {"dataType":"symbolic","outputData":{"name":"verify","value":"\\left(\\begin{array}{c}\n0\\\\\n0\n\\end{array}\\right)"}}
%---
%[output:51270805]
%   data: {"dataType":"text","outputData":{"text":"Verification (should be all zeros):\n","truncated":false}}
%---
%[output:71c6d8a1]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{c}\n0\\\\\n0\n\\end{array}\\right)"}}
%---
%[output:0a9895d8]
%   data: {"dataType":"text","outputData":{"text":"Symbolic EOM saved to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\RRpendulum_EOM.mat\n","truncated":false}}
%---
