function tau1 = swingup_energy_controller(q1, v1, q2, v2, k_e, alpha_max, E0, m, l, g, r, Iz_1)
%swingup_energy_controller Energy-based swing-up for rotary Furuta pendulum.
%
%   tau1 = swingup_energy_controller(q1, v1, q2, v2, k_e, alpha_max, E0,
%                                     m, l, g, r, Iz_1)
%
%   Implements the Åström-Furuta (Automatica, 2000) energy control law
%   adapted for a rotary (RR) inverted pendulum:
%
%     E     = ½·m·l²·v2² - m·g·l·(cos(q2) + 1)    (pendulum energy, 0 at upright)
%     alpha1 = sat( k_e·(E - E0)·sign(v2·cos(q2)), alpha_max )
%     tau1  = Iz1_total(q1, q2) · alpha1
%
%   Inputs:
%     q1, v1      – joint 1 angle [rad] and velocity [rad/s]
%     q2, v2      – pendulum angle [rad] and velocity [rad/s]
%     k_e         – energy feedback gain [1/(J·s)]
%     alpha_max   – maximum angular acceleration of joint 1 [rad/s²]
%     E0          – target energy [J] (0 for upright)
%     m, l, g, r, Iz_1 – physical parameters (see RRpendulum_Parameters_num_BLDC.m)
%
%   Output:
%     tau1 – torque command for joint 1 [Nm]

% --- Pendulum energy (referenced to upright = 0) ---
E = 0.5 * m * l^2 * v2^2 - m * g * l * (cos(q2) + 1);

% --- Energy error ---
E_err = E - E0;

% --- Coupling factor: sign(v2 * cos(q2)) ---
%   cos(q2) captures the geometric coupling between arm rotation and
%   pendulum swing in the rotary Furuta configuration.
coupling = v2 * cos(q2);

% --- Desired angular acceleration (Åström-Furuta Eq. 8, saturated) ---
alpha1_des = k_e * E_err * sign(coupling);

% Saturate acceleration
if alpha1_des > alpha_max
    alpha1_des = alpha_max;
elseif alpha1_des < -alpha_max
    alpha1_des = -alpha_max;
end

% --- Total moment of inertia about motor axis (pose-dependent) ---
Iz1_total = RRpendulum_totalIz1([q1; q2], Iz_1, m, r, l);

% --- Torque = inertia × acceleration ---
tau1 = Iz1_total * alpha1_des;

end
