% =========================================================================
% CALCULATE MAXIMUM WRENCH OUTPUT
% =========================================================================
% This script uses the nominal vehicle parameters and the computeWrench
% function to determine the theoretical maximum force and torque the vehicle
% can produce on each axis. The results can be used to set the normalization
% values in the visualizeSingleResult.m script.
%
clear; clc; close all;

fprintf('Calculating theoretical maximum wrench output...\n');

% --- 1. Load Nominal Vehicle Parameters ---
% This script MUST define 'M', and the nominal 'Motor' and 'Uav' structs.
run('ParameterEstimationBaseOL.m'); 

% Use the nominal parameters, not randomized ones
Motor_nom = Motor;
Uav_nom = Uav;
N_motors = Uav.N_ROTORS;

% --- 2. Assemble the Nominal 'params' Struct ---
% This logic is identical to the Monte Carlo script, but for nominal params.
params = struct();

% a) Define clip limits and scaling factors
u_min = [-1.0; -1.0; 0.0; -1.0; -1.0; -1.0];
u_max = [ 1.0;  1.0; 1.0;  1.0;  1.0;  1.0];
alpha_Tx = Motor_nom.CommandMixing(2) / 10000;
alpha_Ty = Motor_nom.CommandMixing(3) / 10000;
alpha_Tz = Motor_nom.CommandMixing(4) / 10000;

% b) Construct the Effective Scaling & Reordering Matrix (A_eff)
P = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 1 0 0 0; 1 0 0 0 0 0; 0 1 0 0 0 0];
A_scale = diag([alpha_Tx, alpha_Ty, alpha_Tz, 1, 1, 1]);
params.A_eff = A_scale * P;

% c) Control Allocation parameters
params.u_min = u_min;
params.u_max = u_max;
params.M     = Motor.mixingMatrix;
params.N     = N_motors;

% d) Motor dynamics parameters (using nominal motor vectors)
params.Kt = Motor_nom.K_T; params.Ke = Motor_nom.K_E; params.R = Motor_nom.R;
params.C_tau = Motor_nom.C_TAU; params.mv = Motor_nom.volt_slope;
params.V_off = Motor_nom.Volt_offset; params.I0 = Motor_nom.I_0;

% e) B-Matrix construction (using nominal parameters)
kf_vec = 0.5 * Uav_nom.RHO_AIR * Aero.Cz3P.coefs(1) * (Uav_nom.D_PROP.^2) .* Uav_nom.A_PROP;
km_vec = Motor_nom.C_TAU;
B_matrix = zeros(6, N_motors);
for m = 1:N_motors
    t_m = [0; 0; -kf_vec(m)];
    tau_m = [0; 0; -Uav_nom.ROTOR_DIRECTION(m) * km_vec(m)];
    R_m_b = Uav_nom.R_MOTOR_TO_BODY(:,:,m);
    T_body_i = R_m_b * t_m;
    tau_body_i = R_m_b * tau_m;
    r_i = Uav_nom.MotorLoc(m, 1:3)';
    tau_from_thrust = cross(r_i, T_body_i);
    B_matrix(:, m) = [T_body_i; tau_body_i + tau_from_thrust];
end
params.B_matrix = B_matrix;

% --- 3. Define Maximum Test Setpoints ---
% Each setpoint commands a maximum effort on a single axis.
% A baseline thrust of 0.5 is used for torque tests for realism.
setpoints_max_test = [
    % Fx,  Fy,  Fz,  Tx,  Ty,  Tz   (Description)
     1.0, 0.0, 0.5, 0.0, 0.0, 0.0;  % Max positive Fx
    -1.0, 0.0, 0.5, 0.0, 0.0, 0.0;  % Max negative Fx
     0.0, 1.0, 0.5, 0.0, 0.0, 0.0;  % Max positive Fy
     0.0,-1.0, 0.5, 0.0, 0.0, 0.0;  % Max negative Fy
     0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  % Max positive Fz (max thrust)
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  % Min positive Fz (zero thrust)
     0.0, 0.0, 0.5, 1.0, 0.0, 0.0;  % Max positive Tx (roll)
     0.0, 0.0, 0.5,-1.0, 0.0, 0.0;  % Max negative Tx
     0.0, 0.0, 0.5, 0.0, 1.0, 0.0;  % Max positive Ty (pitch)
     0.0, 0.0, 0.5, 0.0,-1.0, 0.0;  % Max negative Ty
     0.0, 0.0, 0.5, 0.0, 0.0, 1.0;  % Max positive Tz (yaw)
     0.0, 0.0, 0.5, 0.0, 0.0,-1.0;  % Max negative Tz
];
nTests = size(setpoints_max_test, 1);
results = zeros(6, nTests);

% --- 4. Run Calculations ---
for i = 1:nTests
    W_des = setpoints_max_test(i, :)';
    results(:, i) = computeWrench(W_des, params);
end

% --- 5. Analyze and Display Results ---
max_Fx = max(abs(results(1, :)));
max_Fy = max(abs(results(2, :)));
max_Fz = max(abs(results(3, :)));
max_Tx = max(abs(results(4, :)));
max_Ty = max(abs(results(5, :)));
max_Tz = max(abs(results(6, :)));

fprintf('\n--- Maximum Wrench Calculation Results ---\n');
fprintf('Max Fx output: %.2f N\n', max_Fx);
fprintf('Max Fy output: %.2f N\n', max_Fy);
fprintf('Max Fz output: %.2f N\n', max_Fz);
fprintf('------------------------------------------\n');
fprintf('Max Tx output: %.2f Nm\n', max_Tx);
fprintf('Max Ty output: %.2f Nm\n', max_Ty);
fprintf('Max Tz output: %.2f Nm\n', max_Tz);
fprintf('------------------------------------------\n\n');

% --- 6. Recommendation for Visualization Script ---
recommended_max_force = ceil(max([max_Fx, max_Fy, max_Fz]));
recommended_max_torque = ceil(max([max_Tx, max_Ty, max_Tz])*10)/10; % Round up to nearest 0.1

fprintf('>>> RECOMMENDATION <<<\n');
fprintf('For your ''visualizeSingleResult.m'' script, use these values:\n');
fprintf('nominal_max_force = %.1f;\n', recommended_max_force);
fprintf('nominal_max_torque = %.1f;\n', recommended_max_torque);
fprintf('\n');