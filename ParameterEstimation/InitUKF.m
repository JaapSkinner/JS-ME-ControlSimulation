clear UKF;
disp('Setting up UKF for 6-DOF Motor Effectiveness Estimation (Lumped-Parameter Model)...');

%% 1. State Vector Definition
n_dynamics = 13 + Uav.N_ROTORS; % add N_ROTORS to the state to hold OMEGA
N_MOTORS = Uav.N_ROTORS;

n_params_eff = 6 * N_MOTORS; % 3 for force (Fx,Fy,Fz), 3 for torque (Tx,Ty,Tz) per motor
n_params_motors = 3 * N_MOTORS; % Gain_u, Coeff_omega, Coeff_omega_sq per motor
n_params = n_params_eff + n_params_motors;
n_total = n_dynamics + n_params; % Will be 93 for an 8-rotor UAV

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

% Calculate nominal values for the lumped parameters for a good initial guess.
Kt_nom   = Motor.K_T(1); Ke_nom   = Motor.K_E(1); R_nom    = Motor.R(1);
Ctau_nom = Motor.C_TAU(1); mv_nom   = Motor.volt_slope(1); Irzz_nom = Motor.I_R_ZZ(1);

Gain_u_nom = (Kt_nom * mv_nom) / (Irzz_nom * R_nom);
Coeff_omega_nom = (Kt_nom * Ke_nom) / (Irzz_nom * R_nom);
Coeff_omega_sq_nom = Ctau_nom / Irzz_nom;

INITIAL_GUESS.motor_gain_u = repmat(Gain_u_nom, N_MOTORS, 1);
INITIAL_GUESS.motor_coeff_w = repmat(Coeff_omega_nom, N_MOTORS, 1);
INITIAL_GUESS.motor_coeff_w2 = repmat(Coeff_omega_sq_nom, N_MOTORS, 1);

%% 3. UKF Block Parameters
% --- Initial State (x0) ---
UKF.InitialState = zeros(n_total, 1);
UKF.InitialState(7) = 1; % Set quaternion w-component

% The parameter states now start at the nominal effectiveness values.
jk = n_dynamics +1;
UKF.InitialState(jk: 13+7*N_MOTORS) = INITIAL_GUESS.effectiveness;
UKF.InitialState(jk+6*N_MOTORS:13+8*N_MOTORS) = INITIAL_GUESS.motor_gain_u;
UKF.InitialState(jk+7*N_MOTORS:13+9*N_MOTORS) = INITIAL_GUESS.motor_coeff_w;
UKF.InitialState(jk+8*N_MOTORS:13+10*N_MOTORS) = INITIAL_GUESS.motor_coeff_w2;

% --- Initial Covariance (P0) ---
P0_dyn_var   = 1e-6;
P0_param_var_eff  = 1e-20;

P0_dynamics = P0_dyn_var * eye(n_dynamics);
P0_params_vec = repmat(P0_param_var_eff, n_params, 1);
UKF.InitialCovariance = blkdiag(P0_dynamics, diag(P0_params_vec));

% --- Process Noise (Q) ---
Q_pos_var    = 1e-7;
Q_vel_var    = 1e-2;
Q_quat_var   = 1e-8;
Q_angvel_var = 1e-2;
Q_omega_var  = 1e-3;
Q_param_var_eff  = 1e-50;

% >>>>>>>>>>>> THIS IS THE CRITICAL CHANGE #3 <<<<<<<<<<<<<<<
% Q matrix is now smaller as it only contains effectiveness params.
Q_variances = [ repmat(Q_pos_var, 3, 1);
                repmat(Q_vel_var, 3, 1);
                repmat(Q_quat_var, 4, 1);
                repmat(Q_angvel_var, 3, 1);
                repmat(Q_omega_var, N_MOTORS,1);
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
DSHOT_omega_noise_std = 0.001;

accel_var      = accel_noise_std^2;
gyro_var       = gyro_noise_std^2;
mocap_pos_var  = mocap_pos_noise_std^2;
mocap_quat_var = mocap_quat_noise_std^2;
DSHOT_omega_noise_var = DSHOT_omega_noise_std^2;

R_variances = [ ...
    repmat(accel_var, 3, 1);
    repmat(gyro_var, 3, 1);
    repmat(mocap_pos_var, 3, 1);
    repmat(mocap_quat_var, 4, 1);
    repmat(DSHOT_omega_noise_var, N_MOTORS,1);
];
UKF.MeasurementNoise = diag(R_variances);

% --- UKF Algorithm Parameters (Alpha, Beta, Kappa) ---
UKF.Alpha = 1e-3;
UKF.Beta  = 2;
UKF.Kappa = 0;

disp('UKF parameters defined in workspace variable "UKF".');