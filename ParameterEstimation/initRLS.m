%% --- Initialization Script for RLS Estimator (Original Model) ---
%
% This script computes the nominal parameters for the RLS estimator based
% on the UAV's physical properties.
%
% Assumes the following structs are loaded:
%   - Uav     (with .RHO_AIR, .D_PROP, .A_PROP, .ROTOR_DIRECTION, .MotorLoc, etc.)
%   - Aero    (with .Cz3P.coefs)
%   - Motor   (with .C_TAU, .K_T, .K_E, .R, .volt_slope, .I_R_ZZ)
%
% Assumes these required properties exist:
%   - Uav.MASS
%   - Uav.INERTIA_MATRIX (must be a 3x3 matrix)
%

% --- 1. Set Up Constants ---
N_MOTORS = Uav.rotor_count; % Or use obj.rotor_count
RLS_INITIAL_GUESS = struct();

% --- 2. Initial Guess for Motor Parameters ---
%
% WARNING: The RLS model (omega = f(u, omega_dot)) and UKF model 
% (omega_dot = f(u, omega)) are physically different. We will create the
% best-effort mapping.
%
% RLS Model: omega = Th1*u + Th2*sqrt(u) + Th3 - Th4*omega_dot
% UKF Model: omega_dot = G*u - C_w*omega - C_w2*omega^2
%
% We can map them by analyzing the linearized UKF model:
%   omega_dot + C_w*omega = G*u
%
% The time constant of this system is (1 / C_w).
% The steady-state (omega_dot=0) is omega_ss = (G / C_w) * u.
%
% Let's map this to the RLS:
% Rewrite RLS: Th4*omega_dot + omega = Th1*u + Th2*sqrt(u) + Th3
% The time constant is Th4.
% The steady-state is omega_ss = Th1*u + Th2*sqrt(u) + Th3.
%
% By comparison, a good initial guess is:
%   Th4 = 1 / C_w
%   Th1 = G / C_w
%   Th2 = 0 (The sqrt(u) term doesn't map)
%   Th3 = 0 (Bias is zero)

% Calculate nominal values from your UKF init
Kt_nom   = Motor.K_T(1); 
Ke_nom   = Motor.K_E(1); 
R_nom    = Motor.R(1);
Ctau_nom = Motor.C_TAU(1); 
mv_nom   = Motor.volt_slope(1); 
Irzz_nom = Motor.I_R_ZZ(1);

% These are the UKF parameters 'G' and 'C_w'
Gain_u_nom = (Kt_nom * mv_nom) / (Irzz_nom * R_nom);
Coeff_omega_nom = (Kt_nom * Ke_nom) / (Irzz_nom * R_nom);

% Calculate the RLS guess parameters
% NOTE: Add a small epsilon to avoid divide-by-zero if Coeff_omega_nom is 0
if Coeff_omega_nom < 1e-6
    Coeff_omega_nom = 1e-6;
end

Th1_guess = Gain_u_nom / Coeff_omega_nom;
Th2_guess = 0;
Th3_guess = 0;
Th4_guess = 1 / Coeff_omega_nom;

% Build the [4, n] initial guess matrix
motor_guess = [
    repmat(Th1_guess, 1, N_MOTORS);
    repmat(Th2_guess, 1, N_MOTORS);
    repmat(Th3_guess, 1, N_MOTORS);
    repmat(Th4_guess, 1, N_MOTORS)
];
RLS_INITIAL_GUESS.motor = motor_guess;


% --- 3. Initial Guess for Effectiveness Parameters ---
%
% This logic is compatible with your RLS model.
% RLS Force Model: delta_v(1:3) = Th_force * delta(omega^2)
%   --> Th_force = (1/mass) * B_force
% RLS Torque Model: delta_v(4:6) = Th_torque * [delta(omega^2); delta(omega_dot)]
%   --> Th_torque = inv(Inertia) * [B_torque, B_gyro]

% First, calculate the *unscaled* B-matrix and B-gyro matrix
kf_vec = 0.5 * Uav.RHO_AIR * Aero.Cz3P.coefs(1) * (Uav.D_PROP.^2) .* Uav.A_PROP;
km_vec = Motor.C_TAU;
Irzz_vec = Motor.I_R_ZZ; % Get inertia for each rotor

B_matrix_nominal = zeros(6, N_MOTORS);
B_gyro_nominal = zeros(3, N_MOTORS);

for m = 1:N_MOTORS
    % --- Thrust and Drag Torque (Same as your UKF code) ---
    t_m_nominal = [0; 0; -kf_vec(m)];
    tau_m_nominal = [0; 0; -Uav.ROTOR_DIRECTION(m) * km_vec(m)];
    
    R_m_b = Uav.R_MOTOR_TO_BODY(:,:,m);
    
    T_body_i_nominal = R_m_b * t_m_nominal;
    tau_body_i_nominal = R_m_b * tau_m_nominal;
    
    r_i = Uav.MotorLoc(m, 1:3)';
    tau_from_thrust = cross(r_i, T_body_i_nominal);
    
    B_matrix_nominal(:, m) = [T_body_i_nominal; tau_body_i_nominal + tau_from_thrust];
    
    % --- Gyroscopic Torque (for the B_gyro matrix) ---
    tau_gyro_motor_frame = [0; 0; -Irzz_vec(m)];
    B_gyro_nominal(:, m) = R_m_b * tau_gyro_motor_frame;
end

% Separate B-matrix into force and torque components
B_force_nominal = B_matrix_nominal(1:3, :);   % Size [3, n]
B_torque_nominal = B_matrix_nominal(4:6, :);  % Size [3, n]

% --- 4. Scale Guesses for the RLS (Lumped Parameters) ---

% RLS force guess: Th_force = (1/mass) * B_force
RLS_INITIAL_GUESS.force = (1 / Uav.MASS) * B_force_nominal;

% RLS torque guess: Th_torque = inv(Inertia) * [B_torque, B_gyro]
% Note: The RLS regressor is [2*w*dw; dw_dot], so B_torque comes first.
combined_torque_matrix = [B_torque_nominal, B_gyro_nominal]; % Size [3, 2*n]
RLS_INITIAL_GUESS.torque = Uav.INERTIA_MATRIX \ combined_torque_matrix; % Size [3, 2*n]