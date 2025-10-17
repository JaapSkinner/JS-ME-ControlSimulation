clear UKF;
disp('Setting up UKF for 6-DOF Motor Effectiveness Estimation (Lumped-Parameter Model)...');

%% 1. State Vector Definition
n_dynamics = 13;
N_MOTORS = Uav.N_ROTORS;

% >>>>>>>>>>>> THIS IS THE CRITICAL CHANGE #1 <<<<<<<<<<<<<<<
% The parameter state now ONLY contains the 6xN effectiveness matrix elements.
% Mass and Inertia have been removed from the state.
n_params = 6 * N_MOTORS; % 3 for force (Fx,Fy,Fz), 3 for torque (Tx,Ty,Tz) per motor

n_total = n_dynamics + n_params; % Will be 61 for an 8-rotor UAV

%% 2. Initial Guesses for Parameters
% Our initial guess for the effectiveness matrix is the B-matrix derived
% from the nominal geometry and motor constants.
kf_vec = 0.5 * Uav.RHO_AIR * Aero.Cz3P.coefs(1) * (Uav.D_PROP.^2) .* Uav.A_PROP;
km_vec = Motor.C_TAU;
B_matrix_nominal = zeros(6, N_MOTORS);
for m = 1:N_MOTORS
    t_m_nominal = [0; 0; -kf_vec(m)];
    tau_m_nominal = [0; 0; -Uav.ROTOR_DIRECTION(m) * km_vec(m)];
    R_m_b = Uav.R_MOTOR_TO_BODY(:,:,m);
    T_body_i_nominal = R_m_b * t_m_nominal;
    tau_body_i_nominal = R_m_b * tau_m_nominal;
    r_i = Uav.MotorLoc(m, 1:3)';
    tau_from_thrust = cross(r_i, T_body_i_nominal);
    B_matrix_nominal(:, m) = [T_body_i_nominal; tau_body_i_nominal + tau_from_thrust];
end

% We flatten the nominal B-matrix into a vector for the state.
INITIAL_GUESS.effectiveness = reshape(B_matrix_nominal, [], 1);

%% 3. UKF Block Parameters
% --- Initial State (x0) ---
UKF.InitialState = zeros(n_total, 1);
UKF.InitialState(7) = 1; % Set quaternion w-component

% >>>>>>>>>>>> THIS IS THE CRITICAL CHANGE #2 <<<<<<<<<<<<<<<
% The parameter states now start at the nominal effectiveness values.
UKF.InitialState(14:end) = INITIAL_GUESS.effectiveness;

% --- Initial Covariance (P0) ---
P0_dyn_var   = 1e-6;
P0_param_var_eff  = 0.0005;

P0_dynamics = P0_dyn_var * eye(n_dynamics);
P0_params_vec = repmat(P0_param_var_eff, n_params, 1);
UKF.InitialCovariance = blkdiag(P0_dynamics, diag(P0_params_vec));

% --- Process Noise (Q) ---
Q_pos_var    = 1e-7;
Q_vel_var    = 1e-2;
Q_quat_var   = 1e-8;
Q_angvel_var = 1e-2;
Q_param_var_eff  = 1e-5;

% >>>>>>>>>>>> THIS IS THE CRITICAL CHANGE #3 <<<<<<<<<<<<<<<
% Q matrix is now smaller as it only contains effectiveness params.
Q_variances = [ repmat(Q_pos_var, 3, 1);
                repmat(Q_vel_var, 3, 1);
                repmat(Q_quat_var, 4, 1);
                repmat(Q_angvel_var, 3, 1);
                repmat(Q_param_var_eff, n_params, 1) ];
UKF.ProcessNoise = diag(Q_variances);

% --- Measurement Noise (R) & Algorithm Parameters ---
% ... (existing setup is correct) ...
% --- Measurement Noise (R) ---
% This section is unchanged as the measurement vector size is the same.
accel_noise_std = 0.2;
gyro_noise_std  = 0.1;
mocap_pos_noise_std = 0.001;
mocap_quat_noise_std = 0.001;

accel_var      = accel_noise_std^2;
gyro_var       = gyro_noise_std^2;
mocap_pos_var  = mocap_pos_noise_std^2;
mocap_quat_var = mocap_quat_noise_std^2;

R_variances = [ ...
    repmat(accel_var, 3, 1);
    repmat(gyro_var, 3, 1);
    repmat(mocap_pos_var, 3, 1);
    repmat(mocap_quat_var, 4, 1)
];
UKF.MeasurementNoise = diag(R_variances);

% --- UKF Algorithm Parameters (Alpha, Beta, Kappa) ---
UKF.Alpha = 1e-3;
UKF.Beta  = 2;
UKF.Kappa = 0;

disp('UKF parameters defined in workspace variable "UKF".');