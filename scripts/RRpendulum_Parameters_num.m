% define numerical values for all parameters
m_num = 0.0126; % kg
l_num = 0.253; % m
r_num = 0.151; % m
g_num = 9.80665; % m/s²
b1_num = 0.0001; %Nms/rad
b2_num = 0.00003; %Nms/rad
satlimits = [64.34 19.63]; % [rad/s², rad/s]

% initial conditions
q1_0 = 0; % Initial position for q1
v1_0 = 0; % Initial velocity for q1
q2_0 = pi/2; % Initial position for q2
v2_0 = 0;   % Initial velocity for q2
z0 = [q1_0; v1_0; q2_0; v2_0];  % Initial state vector