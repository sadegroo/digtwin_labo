clear; clc;
%%
%[text] **Unscented Kalman Filter (UKF) observer design** for the RR inverted pendulum.
%[text] 
%[text] Loads:
%[text] - symbolic EOM from `data/RRpendulum\_EOM.mat`
%[text] - numerical parameters from `data/RRpendulum\_params\_BLDC.mat` \
%[text] 
%[text] Produces:
%[text] - `resources/functions/RRpendulum\_dynamics\_ct.m` — numerical CT dynamics (auto-generated)
%[text] - `data/UKF\_design.mat` — all UKF design parameters \
%[text] 
%[text] State: $z = \[q\_1;; v\_1;; q\_2;; v\_2\]$ (4 states), Input: $u = \\tau\_1$ (1 input), Output: $y = \[q\_1;; q\_2\]$ (2 encoder positions)

if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
data_dir = fullfile(prj.RootFolder, 'data');

load(fullfile(data_dir, 'RRpendulum_EOM.mat'), 'EOM');
load(fullfile(data_dir, 'RRpendulum_params_BLDC.mat'), 'params');
%%
%[text] **Unpack parameters** from the hierarchical struct.

% Mechanism
m   = params.mechanism.m;
l   = params.mechanism.l;
r   = params.mechanism.r;
g   = params.mechanism.g;
b1  = params.mechanism.b1;
b2  = params.mechanism.b2;
Iz1 = params.mechanism.Iz_1;

% Sensing
q1_cpt = params.sensing.q1_cpt;
q2_cpt = params.sensing.q2_cpt;

% Actuation
u_sat = params.actuation.u_sat;

% Sample time
Ts = 1/1000;  % [s] (1 kHz, matches hardware interface)

%%
%[text] **Generate numerical dynamics function**
%[text] Substitute numerical parameter values into the symbolic $f(x,u)$ and generate a standalone
%[text] MATLAB function file via `matlabFunction`. This avoids needing the Symbolic Math Toolbox at runtime.
%[text] Discretization uses a single RK4 step per sample period.

f_sym = EOM.nlss.f;           % symbolic dx/dt = f(x, u)
x_sym = EOM.sym.x;            % [q_1; v_1; q_2; v_2]
u_sym = EOM.sym.u;            % tau_1
p_sym = EOM.sym.params;       % [g l r m b_1 b_2 Iz_1]

f_num_sym = subs(f_sym, p_sym, params.sym_values);

% Generate function file (no Symbolic Math Toolbox needed at runtime)
func_dir = fullfile(prj.RootFolder, 'resources', 'functions');
f_path = fullfile(func_dir, 'RRpendulum_dynamics_ct.m');
matlabFunction(f_num_sym, ...
    'File', f_path, ...
    'Vars', {x_sym, u_sym}, ...
    'Outputs', {'xdot'}, ...
    'Comments', 'Continuous-time dynamics for RR pendulum (auto-generated).');
fprintf('Dynamics function generated: %s\n', f_path); %[output:531d6ab6]

% Function handles for use in this script
f_ct = @(x, u) RRpendulum_dynamics_ct(x, u);
f_dt = @(x, u) rk4_step(f_ct, x, u, Ts);

%%
%[text] **Measurement model**
%[text] Encoders measure joint positions only: $y = C,x = \[q\_1;; q\_2\]$

nx = 4;  % states
ny = 2;  % measurements
nu = 1;  % inputs

C_meas = [1 0 0 0;
          0 0 1 0];
h = @(x) C_meas * x;

%%
%[text] **Sensor noise covariance** (from first principles)
%[text] Encoder quantization modeled as uniform white noise over one count:
%[text] $\\Delta\_i = \\frac{2\\pi}{N\_i}$, $\\quad \\mathrm{var}\_i = \\frac{\\Delta\_i^2}{12} = \\frac{\\pi^2}{3\\,N\_i^2}$

R_q1 = pi^2 / (3 * q1_cpt^2);
R_q2 = pi^2 / (3 * q2_cpt^2);
R = diag([R_q1, R_q2]);

fprintf('\n--- Sensor noise covariance (encoder quantization) ---\n'); %[output:4b39aad6]
fprintf('Joint 1 (N=%d): sigma_q1 = %.4e rad  (%.4e deg)\n', ... %[output:group:6a87b5f5] %[output:0f624d8c]
    q1_cpt, sqrt(R_q1), rad2deg(sqrt(R_q1))); %[output:group:6a87b5f5] %[output:0f624d8c]
fprintf('Joint 2 (N=%d): sigma_q2 = %.4e rad  (%.4e deg)\n', ... %[output:group:3a4e1aed] %[output:66e12da5]
    q2_cpt, sqrt(R_q2), rad2deg(sqrt(R_q2))); %[output:group:3a4e1aed] %[output:66e12da5]
fprintf('R = diag([%.4e, %.4e])\n', R_q1, R_q2); %[output:6c5750d5]

%%
%[text] **Process noise covariance** (tuning parameter)
%[text] Unmodeled acceleration disturbances enter each DOF independently.
%[text] Noise input matrix per DOF (ZOH double integrator):
%[text] $\\Gamma = \[T\_s^2/2;\\; T\_s\]$
%[text] Discrete process noise per DOF:
%[text] $Q\_i = q\_{a,i}^2 \\, \\Gamma \\, \\Gamma^\\top$
%[text] Only two tuning knobs: $q\_{a,1}$ and $q\_{a,2}$ (acceleration noise std per joint).

% --- Tuning knobs ---
qa1 = 10;    % [rad/s^2] joint 1 acceleration noise std
qa2 = 50;    % [rad/s^2] joint 2 acceleration noise std

Gamma = [Ts^2/2; Ts];
Q_ukf = blkdiag(qa1^2 * (Gamma * Gamma'), qa2^2 * (Gamma * Gamma'));

fprintf('\n--- Process noise covariance ---\n'); %[output:4db35143]
fprintf('Acceleration noise std: qa1 = %.1f rad/s^2, qa2 = %.1f rad/s^2\n', qa1, qa2); %[output:46292070]
fprintf('Q_ukf =\n'); disp(Q_ukf); %[output:75e1d56f]

%%
%[text] **UKF sigma point parameters** (Wan–Merwe scaled unscented transform)
%[text] $\\lambda = \\alpha^2 (n\_x + \\kappa) - n\_x$. With $\\alpha = 0.5$, $(n\_x + \\lambda) = 1$
%[text] so $P$ is not scaled — avoids extreme numerical conditioning issues.

alpha = 0.5;      % spread of sigma points (gives (nx+lambda)=1, no P scaling)
beta  = 2;        % optimal for Gaussian distributions
kappa = 0;        % secondary scaling parameter

lambda_ukf = alpha^2 * (nx + kappa) - nx;

% Sigma point weights
n_sigma = 2 * nx + 1;
Wm = zeros(n_sigma, 1);  % weights for mean
Wc = zeros(n_sigma, 1);  % weights for covariance

Wm(1) = lambda_ukf / (nx + lambda_ukf);
Wc(1) = lambda_ukf / (nx + lambda_ukf) + (1 - alpha^2 + beta);
for i = 2:n_sigma
    Wm(i) = 1 / (2 * (nx + lambda_ukf));
    Wc(i) = 1 / (2 * (nx + lambda_ukf));
end

fprintf('\n--- UKF sigma point parameters ---\n'); %[output:33a1141f]
fprintf('alpha = %.1e, beta = %d, kappa = %d\n', alpha, beta, kappa); %[output:81e6dd72]
fprintf('lambda = %.6f\n', lambda_ukf); %[output:7937f451]
fprintf('n_sigma = %d\n', n_sigma); %[output:4bfb4daf]
fprintf('Wm(0) = %.6e, Wc(0) = %.6f\n', Wm(1), Wc(1)); %[output:4ab062d0]
fprintf('Wm(i) = Wc(i) = %.6e  (i = 1..%d)\n', Wm(2), n_sigma-1); %[output:0b432afa]

%%
%[text] **Initial state estimate and covariance**
%[text] Large initial uncertainty to tolerate IC mismatch between UKF and plant.
%[text] Position $\\sigma \\approx 1$ rad ($57\\degree $), velocity $\\sigma \\approx 3.2$ rad/s.

x0_ukf = params.ic.z0;

% Generous initial uncertainty (robust to large IC mismatch)
P0 = diag([1, 10, 1, 10]);

fprintf('\n--- Initial conditions ---\n'); %[output:2db0e655]
fprintf('x0 = [%.4f; %.4f; %.4f; %.4f]\n', x0_ukf); %[output:1efbc074]
fprintf('P0 = diag([%.2e, %.2f, %.2e, %.2f])\n', P0(1,1), P0(2,2), P0(3,3), P0(4,4)); %[output:940b587f]

%%
%[text] **Sanity check**: one full predict + update cycle at the initial condition to verify
%[text] that all function handles, sigma point generation, and matrix dimensions are correct.

fprintf('\n--- Sanity check ---\n'); %[output:1af6c437]

% Test continuous dynamics at initial condition with zero input
xdot_test = f_ct(x0_ukf, 0);
fprintf('f_ct(x0, 0) = [%.4f; %.4f; %.4f; %.4f]\n', xdot_test); %[output:66980da6]

% One RK4 step
x1_test = f_dt(x0_ukf, 0);
fprintf('f_dt(x0, 0) = [%.6f; %.6f; %.6f; %.6f]  (one Ts step)\n', x1_test); %[output:03e46943]

% Sigma point generation
P_sqrt = chol((nx + lambda_ukf) * P0, 'lower');
X_sigma = zeros(nx, n_sigma);
X_sigma(:, 1) = x0_ukf;
for i = 1:nx
    X_sigma(:, 1+i)    = x0_ukf + P_sqrt(:, i);
    X_sigma(:, 1+nx+i) = x0_ukf - P_sqrt(:, i);
end
fprintf('Sigma points generated: %d x %d matrix\n', size(X_sigma)); %[output:75ff2bc5]

% Propagate all sigma points
X_pred = zeros(nx, n_sigma);
for i = 1:n_sigma
    X_pred(:, i) = f_dt(X_sigma(:, i), 0);
end
fprintf('Sigma points propagated through f_dt\n'); %[output:54a30534]

% Predicted mean and covariance
x_pred = X_pred * Wm;
P_pred = Q_ukf;
for i = 1:n_sigma
    dx = X_pred(:, i) - x_pred;
    P_pred = P_pred + Wc(i) * (dx * dx');
end
fprintf('Predicted mean:   [%.6f; %.6f; %.6f; %.6f]\n', x_pred); %[output:4dbf0391]
fprintf('Predicted P diag: [%.2e, %.2e, %.2e, %.2e]\n', diag(P_pred)); %[output:33226ad8]

% Measurement update
Y_pred = zeros(ny, n_sigma);
for i = 1:n_sigma
    Y_pred(:, i) = h(X_pred(:, i));
end
y_pred = Y_pred * Wm;

S = R;
Pxy = zeros(nx, ny);
for i = 1:n_sigma
    dy = Y_pred(:, i) - y_pred;
    S   = S   + Wc(i) * (dy * dy');
    Pxy = Pxy + Wc(i) * ((X_pred(:, i) - x_pred) * dy');
end
K_ukf = Pxy / S;
fprintf('UKF gain K (at x0):\n'); disp(K_ukf); %[output:90eec144]
fprintf('All checks passed.\n'); %[output:5a546010]

%%
%[text] **Save UKF design** to struct and .mat file

ukf = struct();

% Metadata
ukf.meta.created        = datetime('now');
ukf.meta.matlab_version = version;
ukf.meta.source_script  = mfilename('fullpath');
ukf.meta.description    = 'UKF observer design for RR inverted pendulum';
ukf.meta.state          = 'z = [q1; v1; q2; v2]';
ukf.meta.input          = 'u = tau_1';
ukf.meta.output         = 'y = [q1; q2]';

% Dimensions
ukf.dims.nx = nx;
ukf.dims.ny = ny;
ukf.dims.nu = nu;

% Timing
ukf.Ts = Ts;

% Noise covariances
ukf.noise.R   = R;
ukf.noise.R_q1 = R_q1;
ukf.noise.R_q2 = R_q2;
ukf.noise.R_derivation = 'Encoder quantization: var = pi^2/(3*N^2)';
ukf.noise.Q   = Q_ukf;
ukf.noise.qa1 = qa1;
ukf.noise.qa2 = qa2;

% Sigma point parameters
ukf.sigma.alpha  = alpha;
ukf.sigma.beta   = beta;
ukf.sigma.kappa  = kappa;
ukf.sigma.lambda = lambda_ukf;
ukf.sigma.Wm     = Wm;
ukf.sigma.Wc     = Wc;

% Measurement model
ukf.measurement.C = C_meas;

% Initial conditions
ukf.ic.x0 = x0_ukf;
ukf.ic.P0 = P0;

% Dynamics reference
ukf.dynamics.f_ct_file   = 'RRpendulum_dynamics_ct';
ukf.dynamics.Ts          = Ts;
ukf.dynamics.integration = 'RK4';

% Save
save_path = fullfile(data_dir, 'UKF_design.mat');
save(save_path, 'ukf');
fprintf('\nUKF design saved to: %s\n', save_path); %[output:851c0bb9]

%% --- Local functions ---
function x_next = rk4_step(f, x, u, dt)
%RK4_STEP Single Runge-Kutta 4th order integration step.
    k1 = f(x, u);
    k2 = f(x + dt/2 * k1, u);
    k3 = f(x + dt/2 * k2, u);
    k4 = f(x + dt * k3, u);
    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":31.6}
%---
%[output:531d6ab6]
%   data: {"dataType":"text","outputData":{"text":"Dynamics function generated: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\resources\\functions\\RRpendulum_dynamics_ct.m\n","truncated":false}}
%---
%[output:4b39aad6]
%   data: {"dataType":"text","outputData":{"text":"\n--- Sensor noise covariance (encoder quantization) ---\n","truncated":false}}
%---
%[output:0f624d8c]
%   data: {"dataType":"text","outputData":{"text":"Joint 1 (N=8192): sigma_q1 = 2.2141e-04 rad  (1.2686e-02 deg)\n","truncated":false}}
%---
%[output:66e12da5]
%   data: {"dataType":"text","outputData":{"text":"Joint 2 (N=2400): sigma_q2 = 7.5575e-04 rad  (4.3301e-02 deg)\n","truncated":false}}
%---
%[output:6c5750d5]
%   data: {"dataType":"text","outputData":{"text":"R = diag([4.9023e-08, 5.7116e-07])\n","truncated":false}}
%---
%[output:4db35143]
%   data: {"dataType":"text","outputData":{"text":"\n--- Process noise covariance ---\n","truncated":false}}
%---
%[output:46292070]
%   data: {"dataType":"text","outputData":{"text":"Acceleration noise std: qa1 = 10.0 rad\/s^2, qa2 = 50.0 rad\/s^2\n","truncated":false}}
%---
%[output:75e1d56f]
%   data: {"dataType":"text","outputData":{"text":"Q_ukf =\n    0.0000    0.0000         0         0\n    0.0000    0.0001         0         0\n         0         0    0.0000    0.0000\n         0         0    0.0000    0.0025\n\n","truncated":false}}
%---
%[output:33a1141f]
%   data: {"dataType":"text","outputData":{"text":"\n--- UKF sigma point parameters ---\n","truncated":false}}
%---
%[output:81e6dd72]
%   data: {"dataType":"text","outputData":{"text":"alpha = 5.0e-01, beta = 2, kappa = 0\n","truncated":false}}
%---
%[output:7937f451]
%   data: {"dataType":"text","outputData":{"text":"lambda = -3.000000\n","truncated":false}}
%---
%[output:4bfb4daf]
%   data: {"dataType":"text","outputData":{"text":"n_sigma = 9\n","truncated":false}}
%---
%[output:4ab062d0]
%   data: {"dataType":"text","outputData":{"text":"Wm(0) = -3.000000e+00, Wc(0) = -0.250000\n","truncated":false}}
%---
%[output:0b432afa]
%   data: {"dataType":"text","outputData":{"text":"Wm(i) = Wc(i) = 5.000000e-01  (i = 1..8)\n","truncated":false}}
%---
%[output:2db0e655]
%   data: {"dataType":"text","outputData":{"text":"\n--- Initial conditions ---\n","truncated":false}}
%---
%[output:1efbc074]
%   data: {"dataType":"text","outputData":{"text":"x0 = [0.0000; 0.0000; 3.1916; 0.0000]\n","truncated":false}}
%---
%[output:940b587f]
%   data: {"dataType":"text","outputData":{"text":"P0 = diag([1.00e+00, 10.00, 1.00e+00, 10.00])\n","truncated":false}}
%---
%[output:1af6c437]
%   data: {"dataType":"text","outputData":{"text":"\n--- Sanity check ---\n","truncated":false}}
%---
%[output:66980da6]
%   data: {"dataType":"text","outputData":{"text":"f_ct(x0, 0) = [0.0000; 28.4537; 0.0000; 18.8983]\n","truncated":false}}
%---
%[output:03e46943]
%   data: {"dataType":"text","outputData":{"text":"f_dt(x0, 0) = [0.000014; 0.028406; 3.191602; 0.018870]  (one Ts step)\n","truncated":false}}
%---
%[output:75ff2bc5]
%   data: {"dataType":"text","outputData":{"text":"Sigma points generated: 4 x 9 matrix\n","truncated":false}}
%---
%[output:54a30534]
%   data: {"dataType":"text","outputData":{"text":"Sigma points propagated through f_dt\n","truncated":false}}
%---
%[output:4dbf0391]
%   data: {"dataType":"text","outputData":{"text":"Predicted mean:   [-0.000001; -0.001148; 3.191593; 0.000906]\n","truncated":false}}
%---
%[output:33226ad8]
%   data: {"dataType":"text","outputData":{"text":"Predicted P diag: [1.00e+00, 9.94e+00, 1.00e+00, 1.00e+01]\n","truncated":false}}
%---
%[output:90eec144]
%   data: {"dataType":"text","outputData":{"text":"UKF gain K (at x0):\n    1.0000    0.0000\n    0.0100    0.0106\n    0.0000    1.0000\n   -0.0000    0.0460\n\n","truncated":false}}
%---
%[output:5a546010]
%   data: {"dataType":"text","outputData":{"text":"All checks passed.\n","truncated":false}}
%---
%[output:851c0bb9]
%   data: {"dataType":"text","outputData":{"text":"\nUKF design saved to: C:\\Users\\u0130154\\MATLAB\\projects\\digtwin_labo\\data\\UKF_design.mat\n","truncated":false}}
%---
