function [x_hat_new, P_new] = ukf_observer_step(x_hat, P, u, y, ...
    Q, R, Wm, Wc, lambda_ukf, C_meas, Ts, nx)
%UKF_OBSERVER_STEP  One predict+update cycle of the Unscented Kalman Filter.
%
%   [x_hat_new, P_new] = ukf_observer_step(x_hat, P, u, y, ...
%       Q, R, Wm, Wc, lambda_ukf, C_meas, Ts, nx)
%
%   Inputs:
%       x_hat       (nx x 1)  current state estimate (absolute coordinates)
%       P           (nx x nx) current estimation error covariance
%       u           (scalar)  control input (torque tau_1)
%       y           (ny x 1)  measurement vector (absolute coordinates)
%       Q           (nx x nx) process noise covariance
%       R           (ny x ny) measurement noise covariance
%       Wm          (2*nx+1 x 1) sigma point weights for mean
%       Wc          (2*nx+1 x 1) sigma point weights for covariance
%       lambda_ukf  (scalar)  sigma point scaling parameter
%       C_meas      (ny x nx) measurement matrix (y = C*x)
%       Ts          (scalar)  sample time [s]
%       nx          (scalar)  number of states
%
%   Outputs:
%       x_hat_new   (nx x 1)  updated state estimate
%       P_new       (nx x nx) updated covariance
%
%   Calls RRpendulum_dynamics_ct (must be on the MATLAB path).

    ny = size(C_meas, 1);
    n_sigma = 2 * nx + 1;

    % --- Generate sigma points ---
    P_scaled = (nx + lambda_ukf) * P;
    % Force symmetry to avoid numerical issues in chol
    P_scaled = 0.5 * (P_scaled + P_scaled');
    P_sqrt = chol(P_scaled, 'lower');

    X_sigma = zeros(nx, n_sigma);
    X_sigma(:, 1) = x_hat;
    for i = 1:nx
        X_sigma(:, 1+i)      = x_hat + P_sqrt(:, i);
        X_sigma(:, 1+nx+i)   = x_hat - P_sqrt(:, i);
    end

    % --- Predict: propagate sigma points through nonlinear dynamics ---
    X_pred = zeros(nx, n_sigma);
    for i = 1:n_sigma
        X_pred(:, i) = rk4_step(X_sigma(:, i), u, Ts);
    end

    % Predicted mean
    x_pred = X_pred * Wm;

    % Predicted covariance
    P_pred = Q;
    for i = 1:n_sigma
        dx = X_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (dx * dx');
    end
    P_pred = 0.5 * (P_pred + P_pred');  % enforce symmetry

    % --- Update: measurement prediction ---
    Y_pred = C_meas * X_pred;
    y_pred = Y_pred * Wm;

    % Innovation covariance and cross-covariance
    S = R;
    Pxy = zeros(nx, ny);
    for i = 1:n_sigma
        dy = Y_pred(:, i) - y_pred;
        S   = S   + Wc(i) * (dy * dy');
        Pxy = Pxy + Wc(i) * ((X_pred(:, i) - x_pred) * dy');
    end

    % Kalman gain
    K = Pxy / S;

    % State and covariance update
    x_hat_new = x_pred + K * (y - y_pred);
    P_new = P_pred - K * S * K';
    P_new = 0.5 * (P_new + P_new');  % enforce symmetry
end

%% --- Local functions ---
function x_next = rk4_step(x, u, dt)
%RK4_STEP  Single Runge-Kutta 4th order integration step using
%   RRpendulum_dynamics_ct as the continuous-time dynamics.
    k1 = RRpendulum_dynamics_ct(x, u);
    k2 = RRpendulum_dynamics_ct(x + dt/2 * k1, u);
    k3 = RRpendulum_dynamics_ct(x + dt/2 * k2, u);
    k4 = RRpendulum_dynamics_ct(x + dt * k3, u);
    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end
