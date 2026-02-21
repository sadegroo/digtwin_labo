% define numerical values for all parameters
%mechanism
m_num = 0.0126; % kg
l_num = 0.253; % m
r_num = 0.151; % m
g_num = 9.80665; % m/s²
b1_num = 0.0001; %Nms/rad
b2_num = 0.00003; %Nms/rad
Iz_1_num = 3e-5; %kgm2

%actuation
u_sat = 0.2; % Nm, symmetric

%sensing
q1_cpt = 8192; %counts per turn
q2_cpt = 2400; %counts per turn

% initial conditions
q1_0 = 0; % Initial position for q1
v1_0 = 0; % Initial velocity for q1
q2_0 = pi+0.05; % Initial position for q2
v2_0 = 0;   % Initial velocity for q2
z0 = [q1_0; v1_0; q2_0; v2_0];  % Initial state vector

%% Save to .mat (hierarchical struct for downstream scripts)
params = struct();

params.meta.created       = datetime('now');
params.meta.source_script = 'RRpendulum_Parameters_num_BLDC.m';
params.meta.description   = 'Numerical parameters for RR pendulum (BLDC variant)';

% Mechanism (rigid body dynamics)
params.mechanism.m    = m_num;      % [kg] point mass
params.mechanism.l    = l_num;      % [m] pendulum length
params.mechanism.r    = r_num;      % [m] link 1 offset
params.mechanism.g    = g_num;      % [m/s^2] gravitational acceleration
params.mechanism.b1   = b1_num;     % [Nms/rad] joint 1 viscous damping
params.mechanism.b2   = b2_num;     % [Nms/rad] joint 2 viscous damping
params.mechanism.Iz_1 = Iz_1_num;   % [kgm^2] joint 1 rotational inertia

% Actuation
params.actuation.u_sat = u_sat;     % [Nm] symmetric torque saturation

% Sensing (encoders)
params.sensing.q1_cpt = q1_cpt;     % [counts/turn] joint 1 encoder
params.sensing.q2_cpt = q2_cpt;     % [counts/turn] joint 2 encoder

% Initial conditions
params.ic.q1_0 = q1_0;
params.ic.v1_0 = v1_0;
params.ic.q2_0 = q2_0;
params.ic.v2_0 = v2_0;
params.ic.z0   = z0;

% Symbolic parameter ordering (matches EOM.sym.params = [g l r m b_1 b_2 Iz_1])
params.sym_values = [g_num, l_num, r_num, m_num, b1_num, b2_num, Iz_1_num];

% Save
if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
save_dir = fullfile(prj.RootFolder, 'data');
if ~isfolder(save_dir), mkdir(save_dir); end
save(fullfile(save_dir, 'RRpendulum_params_BLDC.mat'), 'params');
fprintf('Parameters saved to: %s\n', fullfile(save_dir, 'RRpendulum_params_BLDC.mat'));