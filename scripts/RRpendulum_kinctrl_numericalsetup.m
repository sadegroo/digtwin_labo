clear
clc
run RRpendulum_forkin_dyn.mlx %[output:835b39ec]
%%
% define numerical values for all parameters
m_num = 0.0126; % kg
l_num = 0.253; % m
r_num = 0.151; % m
g_num = 9.80665; % m/s²
b1_num = 0.0001; %Nms/rad
b2_num = 0.00003; %Nms/rad
satlimits = [64.34 19.63]; % [rad/s², rad/s]
%[text] **Excitation:Forced q1 movement**
%[text] assume that the motion(position, acceleration, velocity) of q1 is determined by the stepper drive, assuming it can deliver enough torque to not skip any steps ==\> only solve equation for v\_dot(2)
v1ampl = 3; %rad/s
v1freq = 1; %Hz

v1_forced = v1ampl*sin(2*pi*v1freq*t)
v1dot_forced = diff(v1_forced, t)

% Define initial conditions
q1_0 = 0;
v1_0 = double(subs(v1_forced, t, 0));
q2_0 = pi/2; % Initial condition for q2
v2_0 = 0;   % Initial velocity for q2
z0 = [q1_0; v1_0; q2_0; v2_0];  % Initial state vector

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":22}
%---
%[output:835b39ec]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Solutions are only valid under certain conditions. To include parameters and conditions in the solution, specify the 'ReturnConditions' value as 'true'."}}
%---
