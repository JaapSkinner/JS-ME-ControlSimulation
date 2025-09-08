clear UKF;
disp('Setting up UKF parameters...');

%% 1. State Vector Definition
n_dynamics = 13;
n_params = 4;
n_total = n_dynamics + n_params;

%% 2. Ground Truth and Initial Guesses for Parameters
GROUND_TRUTH.mass = Uav.M;
GROUND_TRUTH.Ixx  = Uav.I(1,1);
GROUND_TRUTH.Iyy  = Uav.I(2,2);
GROUND_TRUTH.Izz  = Uav.I(3,3);

INITIAL_GUESS.mass = GROUND_TRUTH.mass + (GROUND_TRUTH.mass * 0.1 * rand(1));
INITIAL_GUESS.Ixx   = Uav.I(1,1) + GROUND_TRUTH.Ixx * 0.1 * rand(1);
INITIAL_GUESS.Iyy   = Uav.I(2,2) + GROUND_TRUTH.Iyy * 0.1 * rand(1);
INITIAL_GUESS.Izz   = Uav.I(3,3) + GROUND_TRUTH.Izz * 0.1 * rand(1);

%% 3. UKF Block Parameters

% --- Initial State (x0) ---
UKF.InitialState = zeros(n_total, 1);
UKF.InitialState(7) = 1;
UKF.InitialState(14) = INITIAL_GUESS.mass;
UKF.InitialState(15) = INITIAL_GUESS.Ixx;
UKF.InitialState(16) = INITIAL_GUESS.Iyy;
UKF.InitialState(17) = INITIAL_GUESS.Izz;

% --- Initial Covariance (P0) ---
% Using the values from the last stable run. A large parameter variance
% allows for an aggressive initial correction.
P0_dyn_var   = 1e-6;
P0_param_var = 0.1;
P0_dynamics = P0_dyn_var * eye(n_dynamics);
P0_params   = P0_param_var * eye(n_params);
UKF.InitialCovariance = blkdiag(P0_dynamics, P0_params);

% --- Process Noise (Q) ---
% Using the values from the last stable run. A higher parameter variance
% allows for a faster "learning rate".
Q_pos_var    = 1e-7;
Q_vel_var    = 1e-2;
Q_quat_var   = 1e-8;
Q_angvel_var = 1e-2;
Q_param_var  = 5e-4;
Q_variances = [ repmat(Q_pos_var, 3, 1);
                repmat(Q_vel_var, 3, 1);
                repmat(Q_quat_var, 4, 1);
                repmat(Q_angvel_var, 3, 1);
                repmat(Q_param_var, n_params, 1) ];
UKF.ProcessNoise = diag(Q_variances);

% --- Measurement Noise (R) ---
% Models the uncertainty/noise of our sensors.
% This MUST match the size and order of the measurement vector from uavMeasurementFcn.m
% z_predicted = [accel (3x1); gyro (3x1); mocap (7x1)]

% --- Noise Standard Deviations ---
% These values usually come from the sensor datasheet or characterization.
accel_noise_std = 0.05;    % [m/s^2]
gyro_noise_std  = 0.01;    % [rad/s]
mocap_pos_noise_std = 0.0005; % [m] - Motion capture is typically very precise.
mocap_quat_noise_std = 0.0001;% [unitless] - Quaternion noise.

% --- Calculate Variances (std^2) ---
accel_var      = accel_noise_std^2;
gyro_var       = gyro_noise_std^2;
mocap_pos_var  = mocap_pos_noise_std^2;
mocap_quat_var = mocap_quat_noise_std^2;

% --- Assemble the 13x1 Vector of Variances ---
% The order here MUST match the order in uavMeasurementFcn.m
R_variances = [ ...
    repmat(accel_var, 3, 1);      % Accel [x, y, z]
    repmat(gyro_var, 3, 1);       % Gyro [x, y, z]
    repmat(mocap_pos_var, 3, 1);  % Mocap Position [x, y, z]
    repmat(mocap_quat_var, 4, 1)  % Mocap Quaternion [w, x, y, z]
];

% --- Create the final 13x13 Diagonal Covariance Matrix ---
UKF.MeasurementNoise = diag(R_variances);

% --- UKF Algorithm Parameters (Alpha, Beta, Kappa) ---
UKF.Alpha = 1e-3;
UKF.Beta  = 2;
UKF.Kappa = 0;

disp('UKF parameters defined in workspace variable "UKF".');

