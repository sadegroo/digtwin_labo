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
q2_0 = pi/2; % Initial position for q2
v2_0 = 0;   % Initial velocity for q2
z0 = [q1_0; v1_0; q2_0; v2_0];  % Initial state vector