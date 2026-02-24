function [x_hat_new, P_new] = ukf_observer_step(x_hat, P, u, y, ...
    Q, R, Wm, Wc, lambda_ukf, C_meas, Ts)
%UKF_OBSERVER_STEP  One predict+update cycle of the Unscented Kalman Filter.
%
%   [x_hat_new, P_new] = ukf_observer_step(x_hat, P, u, y, ...
%       Q, R, Wm, Wc, lambda_ukf, C_meas, Ts)
%
%   Inputs:
%       x_hat       (4 x 1)  current state estimate (absolute coordinates)
%       P           (4 x 4)  current estimation error covariance
%       u           (scalar)  control input (torque tau_1)
%       y           (2 x 1)  measurement vector (absolute coordinates)
%       Q           (4 x 4)  process noise covariance
%       R           (2 x 2)  measurement noise covariance
%       Wm          (9 x 1)  sigma point weights for mean
%       Wc          (9 x 1)  sigma point weights for covariance
%       lambda_ukf  (scalar)  sigma point scaling parameter
%       C_meas      (2 x 4)  measurement matrix (y = C*x)
%       Ts          (scalar)  sample time [s]
%
%   Outputs:
%       x_hat_new   (4 x 1)  updated state estimate
%       P_new       (4 x 4)  updated covariance
%
%   State: z = [q1; v1; q2; v2] (nx=4, ny=2, n_sigma=9).
%   Calls RRpendulum_dynamics_ct (must be on the MATLAB path).

    nx = 4;
    ny = 2;
    n_sigma = 9;  % 2*nx + 1

    % --- NaN/Inf firewall: reset rather than propagate garbage ---
    if any(~isfinite(x_hat(:))) || any(~isfinite(P(:)))
        x_hat_new = [0; 0; 0; 0];   % pendulum hanging down (safe)
        P_new = Q * 1e3;
        return;
    end

    % --- Generate sigma points ---
    P_scaled = (nx + lambda_ukf) * P;
    P_scaled = 0.5 * (P_scaled + P_scaled');  % enforce symmetry
    [P_sqrt, chol_flag] = chol(P_scaled, 'lower');
    if chol_flag ~= 0
        % Repair: add diagonal loading and retry
        P_scaled = P_scaled + 1e-8 * eye(nx);
        [P_sqrt, chol_flag2] = chol(P_scaled, 'lower');
        if chol_flag2 ~= 0
            % Last resort: use scaled identity
            P_sqrt = sqrt(1e-6) * eye(nx);
        end
    end

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

    % State and covariance update (Joseph form — guaranteed PSD)
    x_hat_new = x_pred + K * (y - y_pred);
    IKC = eye(nx) - K * C_meas;
    P_new = IKC * P_pred * IKC' + K * R * K';
    P_new = 0.5 * (P_new + P_new');  % enforce symmetry

    % --- Covariance bounds ---
    P_floor = 1e-10;
    P_ceil  = [40; 1e4; 40; 1e4];  % [pos; vel; pos; vel]
    for i = 1:nx
        if P_new(i,i) < P_floor
            P_new(i,i) = P_floor;
        elseif P_new(i,i) > P_ceil(i)
            P_new(i,i) = P_ceil(i);
        end
    end

    % --- Velocity clamping ---
    v_max = 100;  % rad/s — generous physical bound
    x_hat_new(2) = max(-v_max, min(v_max, x_hat_new(2)));
    x_hat_new(4) = max(-v_max, min(v_max, x_hat_new(4)));
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
