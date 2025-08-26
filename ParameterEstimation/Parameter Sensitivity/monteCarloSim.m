% =========================================================================
% MONTE CARLO SCRIPT using direct wrench calculation (Matrix Version)
% =========================================================================
clear; clc; close all;

% --- 1. Simulation Setup ---
N_samples = 5000;
N_motors = 8;
resultsFolder = 'Results/ParameterEstimation/MonteCarlo/wrench_calculation';

if exist(resultsFolder, 'dir')
    delete(fullfile(resultsFolder, '*'));
end
mkdir(resultsFolder);

% Load base parameters. This script MUST define 'M', 'u_min'/'u_max', and
% the torque scaling factors 'alpha_Tx', 'alpha_Ty', 'alpha_Tz'.
run('ParameterEstimationBaseOL.m'); 

Motor_nom = Motor;
Uav_nom = Uav;
Uav_nom.COM = [0 0 0];
variationPercent = 0 * ones(17,1);

% Define the setpoints to be tested for EACH parameter sample.
% Each row is a desired NORMALIZED WRENCH setpoint: [Fx, Fy, Fz, Tx, Ty, Tz]
% Fz is in [0, 1], all others are in [-1, 1].

setpoints = [
    % --- Single-Axis Positive Tests (with 0% baseline hover thrust) ---
    % Fx, Fy, Fz, Tx, Ty, Tz
      0.7,   0,   0,   0,   0,   0;        % 1: Strong forward thrust (Fx)
      0, 0.7,   0,   0,   0,   0;      % 2: Strong right thrust (Fy)
      0,   0,   0.9,   0,   0,   0;      % 3: High vertical thrust (Fz)
      0,   0,   0, 0.7,   0,   0;      % 4: Strong roll right torque (Tx)
      0,   0,   0,   0, 0.7,   0;      % 5: Strong pitch forward torque (Ty)
      0,   0,   0,   0,   0, 0.7;      % 6: Strong yaw right torque (Tz)

    % --- Single-Axis Negative Tests ---
   -0.7,   0,   0,   0,   0,   0;      % 7: Strong backward thrust (Fx)
      0, -0.7,   0,   0,   0,   0;      % 8: Strong left thrust (Fy)
      0,   0,   0.1,   0,   0,   0;      % 9: Low vertical thrust (Fz)
      0,   0,   0,-0.7,   0,   0;      % 10: Strong roll left torque (Tx)
      0,   0,   0,   0,-0.7,   0;      % 11: Strong pitch backward torque (Ty)
      0,   0,   0,   0,   0,-0.7;      % 12: Strong yaw left torque (Tz)

    % --- Combination Tests ---
    0.5,   0,   0.6,   0, 0.5,   0;      % 13: Forward flight (Fx + Ty)
      0, -0.5,   0.6, 0.5,   0,   0;      % 14: Rolling while moving left (Fy + Tx)
      0,   0,   0.8,   0,   0, 0.5;      % 15: High thrust climb with yaw
    0.2, 0.2,   0.7,-0.3, 0.3,-0.2;      % 16: Complex maneuver: forward-right, climbing, and turning
];
nSetpoints = size(setpoints, 1);
fprintf('Starting Monte Carlo simulation with %d parameter samples.\n', N_samples);
fprintf('Each sample will be tested against %d 6-DOF wrench setpoints.\n', nSetpoints);


% --- 2. Main Monte Carlo Loop ---
for i = 1:N_samples
    [Motor_i, Uav_i, features_i] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], N_motors);
    Uav_i.COM = [0,0,0];

    % --- Assemble the 'params' struct required by computeWrench ---
    params = struct();
    
    % a) Construct the Effective Scaling & Reordering Matrix (A_eff)
    %    Step 1: Define the permutation matrix P to reorder the wrench
    P = [0 0 0 1 0 0;  % Tx -> 1st pos
         0 0 0 0 1 0;  % Ty -> 2nd pos
         0 0 0 0 0 1;  % Tz -> 3rd pos
         0 0 1 0 0 0;  % Fz -> 4th pos
         1 0 0 0 0 0;  % Fx -> 5th pos
         0 1 0 0 0 0]; % Fy -> 6th pos

    % Define the clip limits based on the hard-coded values in MultirotorMixer.m
    % The order is the standard wrench order: [Fx, Fy, Fz, Tx, Ty, Tz]
    u_min = [-1.0; -1.0; 0.0; -1.0; -1.0; -1.0];
    u_max = [ 1.0;  1.0; 1.0;  1.0;  1.0;  1.0]; 
    % Extract the torque scaling factors from the 'command_mixing' variable
    % which is loaded by the base script. The mapping below is taken
    % directly from the MultirotorMixer.m source code.
    alpha_Tx = Motor.CommandMixing(2) / 10000; % Roll scaling
    alpha_Ty = Motor.CommandMixing(3) / 10000; % Pitch scaling
    alpha_Tz = Motor.CommandMixing(4) / 10000; % Yaw scaling
    % -------------------------
         
    %    Step 2: Define the scaling matrix for the reordered vector
    A_scale = diag([alpha_Tx, alpha_Ty, alpha_Tz, 1, 1, 1]);
    
    %    Step 3: Combine them into the final effective matrix
    params.A_eff = A_scale * P;

    % b) Control Allocation parameters
    params.u_min = u_min;   % In standard order [Fx Fy Fz Tx Ty Tz]
    params.u_max = u_max;   % In standard order [Fx Fy Fz Tx Ty Tz]
    params.M     = Motor.mixingMatrix;       % Mixer matrix, columns are [Roll Pitch Yaw Fz Fx Fy]
    params.N     = N_motors;
    
    % c) Motor dynamics parameters (vectors)
    params.Kt     = Motor_i.K_T;
    params.Ke     = Motor_i.K_E;
    params.R      = Motor_i.R;
    params.C_tau  = Motor_i.C_TAU;
    params.mv     = Motor_i.volt_slope;
    params.V_off  = Motor_i.Volt_offset;
    params.I0     = Motor_i.I_0;

    % d) B-Matrix construction
    kf_vec = 0.5 * Uav_i.RHO_AIR * Aero.Cz3P.coefs(1) * (Uav_i.D_PROP.^2) .* Uav_i.A_PROP;
    km_vec = Motor_i.C_TAU;
    B_matrix = zeros(6, N_motors);
    for m = 1:N_motors
        t_m = [0; 0; -kf_vec(m)];
        tau_m = [0; 0; -Uav_i.ROTOR_DIRECTION(m) * km_vec(m)];
        R_m_b = Uav_i.R_MOTOR_TO_BODY(:,:,m);
        T_body_i = R_m_b * t_m;
        tau_body_i = R_m_b * tau_m;

        % Select only the first 3 elements [x,y,z] for the position vector
        r_i = Uav_i.MotorLoc(m, 1:3)';
        
        tau_from_thrust = cross(r_i, T_body_i);
        B_matrix(:, m) = [T_body_i; tau_body_i + tau_from_thrust];
    end
    params.B_matrix = B_matrix;
    
    % --- Calculation Loop ---
    wrench_results = zeros(6, nSetpoints);
    for sp = 1:nSetpoints
        W_des = setpoints(sp, :)';
        wrench_results(:, sp) = computeWrench(W_des, params);
    end
    
    % --- 3. Save Results ---
    dataStruct = struct();
    dataStruct.Motor_params = Motor_i;
    dataStruct.Uav_params = Uav_i;
    dataStruct.Sampled_features = features_i;
    dataStruct.setpoints = setpoints;
    dataStruct.wrenches = wrench_results;
    dataStruct.A_eff_used = params.A_eff;

    filename = fullfile(resultsFolder, sprintf('simResult_%04d.mat', i));
    save(filename, '-struct', 'dataStruct');

    if mod(i, 500) == 0
        fprintf('  Completed and saved sample %d / %d.\n', i, N_samples);
    end
end

fprintf('Monte Carlo simulation finished. Results are in %s\n', resultsFolder);