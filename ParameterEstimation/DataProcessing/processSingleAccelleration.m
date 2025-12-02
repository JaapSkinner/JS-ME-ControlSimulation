%% ANALYZE SINGLE SAMPLE (ACCELERATION DOMAIN)
%
% This script compares the "Motor-Induced Acceleration" estimated by
% different methods. 
%
% DOMAIN: 
%   Linear Acceleration (m/s^2) and Angular Acceleration (rad/s^2).
%   Note: This isolates the actuator contribution (Gravity is removed).
%
% Comparison:
%   - True: Real_Wrench / Mass & Inertia
%   - Nominal: (B_nom * w^2) / Mass & Inertia
%   - UKF: (B_ukf * w^2) / Mass & Inertia
%   - RLS: Direct Output (Kinematic Estimation)
%
clear; clc; close all;

%% --- CONFIGURATION ---
% FLAGS
PLOT_UKF_DATA = false;   
PLOT_RLS_DATA = true;   

RLS_SAMPLE_TIME = 50.0; 
TRIM_SECONDS = 5.0;     

% Plot Colors
c.nom = [0.6350 0.0780 0.1840]; % Red 
c.ukf = [0.0000 0.4470 0.7410]; % Blue 
c.rls = [0.4660 0.6740 0.1880]; % Green 
c.true = [0 0 0];               % Black 

%% --- 1. Select Files ---
fprintf('Step 1: Select a specific UKF result file (estimation_*.mat)...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end
[ukfFileName, ukfPath] = uigetfile(fullfile(startPath, 'estimation_*.mat'), 'Select UKF File');
if isequal(ukfFileName, 0), disp('Cancelled.'); return; end
ukfFullFile = fullfile(ukfPath, ukfFileName);

rlsFileName = 'Skipped';
if PLOT_RLS_DATA
    % Try to auto-detect based on "SampleXXX"
    sampleIdMatch = regexp(ukfFileName, 'Sample\d+', 'match');
    if ~isempty(sampleIdMatch)
        sampleID = sampleIdMatch{1};
        fprintf('Step 2: Select RLS file for %s...\n', sampleID);
        [rlsFileName, rlsPath] = uigetfile(fullfile(ukfPath, ['*' sampleID '*.mat']), 'Select RLS File');
    else
        [rlsFileName, rlsPath] = uigetfile(fullfile(ukfPath, '*.mat'), 'Select RLS File');
    end
    
    if isequal(rlsFileName, 0)
        disp('Cancelled RLS selection. Proceeding without RLS.'); 
        PLOT_RLS_DATA = false; 
    else
        rlsFullFile = fullfile(rlsPath, rlsFileName);
    end
end
fprintf('Processing: %s\n', ukfFileName);

%% --- 2. Process Simulation Physics & Truth ---
S_ukf = load(ukfFullFile);

% 2.1 EXTRACT PHYSICS CONSTANTS (Automatic from Sim)
if ~isfield(S_ukf, 'Uav')
    error('Uav structure not found in file. Cannot retrieve Mass/Inertia.');
end
Uav_Mass = S_ukf.Uav.M;     % Mass (kg)
Uav_Inertia = S_ukf.Uav.I;  % Inertia Matrix (3x3)
N_ROTORS = S_ukf.Uav.N_ROTORS;

fprintf('Sim Physics detected: Mass = %.2f kg, Rotors = %d\n', Uav_Mass, N_ROTORS);

% 2.2 Process True Data (Wrench -> Acceleration)
ukfData = S_ukf.simOut.UKFData;
Omega = ukfData.Omega;
Real_Wrench = ukfData.Real_Wrench; % [Fx Fy Fz Tx Ty Tz]

% Convert True Wrench to True Acceleration (Kinematic Truth)
% Linear: a = F / m
True_Lin_Accel = Real_Wrench.Data(:, 1:3) / Uav_Mass;

% Angular: alpha = inv(I) * tau
% Note: We calculate alpha caused by MOTORS only to match RLS/Nominal definition
True_Torque = Real_Wrench.Data(:, 4:6)'; % 3 x T
True_Ang_Accel = (Uav_Inertia \ True_Torque)'; % (I^-1 * Tau)' -> T x 3

True_Accel_Data = [True_Lin_Accel, True_Ang_Accel];
True_Accel = timeseries(True_Accel_Data, Real_Wrench.Time);

%% --- 3. Process Nominal Model (Force -> Accel) ---
B_nom = S_ukf.B_matrix_nominal; % N/rad^2
if size(B_nom, 2) ~= N_ROTORS, B_nom = B_nom(:, 1:N_ROTORS); end

Omega_sq = Omega.Data .^ 2; 

% Predict Force
Wrench_Nom_Val = (B_nom * Omega_sq')';

% Scale to Accel
Accel_Nom_Lin = Wrench_Nom_Val(:, 1:3) / Uav_Mass;
Accel_Nom_Ang = (Uav_Inertia \ Wrench_Nom_Val(:, 4:6)')';

Accel_Nom = timeseries([Accel_Nom_Lin, Accel_Nom_Ang], Omega.Time);

%% --- 4. Process UKF (Force -> Accel) ---
if PLOT_UKF_DATA
    % Extract UKF B-Matrix
    endTime = ukfData.UKF_DATA.Time(end);
    startTime = max(ukfData.UKF_DATA.Time(1), endTime - 3.0);
    ts_subset = getsampleusingtime(ukfData.UKF_DATA, startTime, endTime);
    
    % Assuming params start after state vector
    param_start = 12 + N_ROTORS + 1; 
    parametersWindow = ts_subset.Data(:, param_start:end);
    est_params_avg = mean(parametersWindow, 1);
    
    B_vector = est_params_avg(1 : 6*N_ROTORS);
    B_Matrix_UKF = reshape(B_vector, [6, N_ROTORS]);
    
    % Predict Force
    Wrench_UKF_Val = (B_Matrix_UKF * Omega_sq')';
    
    % Scale to Accel
    Accel_UKF_Lin = Wrench_UKF_Val(:, 1:3) / Uav_Mass;
    Accel_UKF_Ang = (Uav_Inertia \ Wrench_UKF_Val(:, 4:6)')';
    
    Accel_UKF = timeseries([Accel_UKF_Lin, Accel_UKF_Ang], Omega.Time);
end

%% --- 5. Process RLS (Direct Acceleration) ---
if PLOT_RLS_DATA
    S_rls = load(rlsFullFile);
    rlsData = S_rls.simOut.RLSData;
    
    % Find Sample Index
    [~, time_idx] = min(abs(rlsData.MotorParams.Time - RLS_SAMPLE_TIME));
    if rlsData.MotorParams.Time(end) < (RLS_SAMPLE_TIME - 1.0)
        time_idx = length(rlsData.MotorParams.Time);
    end
    
    % Extract Kinematic Matrices (Already scaled by Mass/Inertia inside the physics of RLS)
    Est_SpecForce = rlsData.ForceEffectiveness.Data(:, :, time_idx);   % m/s^2 / rad^2
    Est_SpecTorque = rlsData.TorqueEffectiveness.Data(:, :, time_idx); % rad/s^2 / rad^2
    
    % Safety resize
    if size(Est_SpecForce, 2) ~= N_ROTORS
         Est_SpecForce = Est_SpecForce(:, 1:N_ROTORS);
         Est_SpecTorque = Est_SpecTorque(:, 1:N_ROTORS);
    end
    
    B_Matrix_RLS = [Est_SpecForce; Est_SpecTorque];
    
    % Predict Acceleration Directly
    Accel_RLS_Val = (B_Matrix_RLS * Omega_sq')';
    Accel_RLS = timeseries(Accel_RLS_Val, Omega.Time);
end

%% --- 6. Plotting & Metrics ---
start_times = [True_Accel.Time(1)];
end_times   = [True_Accel.Time(end)];
if PLOT_RLS_DATA, start_times(end+1)=Accel_RLS.Time(1); end_times(end+1)=Accel_RLS.Time(end); end

t_start = max(start_times) + TRIM_SECONDS;
t_end   = min(end_times) - TRIM_SECONDS;
time_vec = True_Accel.Time;
mask = (time_vec >= t_start) & (time_vec <= t_end);
t_plot = time_vec(mask);

% Resample
a_real = resample(True_Accel, t_plot).Data;
a_nom  = resample(Accel_Nom, t_plot).Data;
if PLOT_UKF_DATA, a_ukf = resample(Accel_UKF, t_plot).Data; end
if PLOT_RLS_DATA, a_rls = resample(Accel_RLS, t_plot).Data; end

% --- CALCULATE METRICS (RMSE) ---
err_nom = a_real - a_nom;
rmse_nom_axis = rms(err_nom, 1);
rmse_nom_lin_total = rms(vecnorm(err_nom(:,1:3), 2, 2));
rmse_nom_ang_total = rms(vecnorm(err_nom(:,4:6), 2, 2));

if PLOT_RLS_DATA
    err_rls = a_real - a_rls;
    rmse_rls_axis = rms(err_rls, 1);
    rmse_rls_lin_total = rms(vecnorm(err_rls(:,1:3), 2, 2));
    rmse_rls_ang_total = rms(vecnorm(err_rls(:,4:6), 2, 2));
    
    % Improvement Factor (%)
    % Formula: (1 - RLS/Nominal) * 100. Positive = RLS is better.
    improv_axis = (1 - (rmse_rls_axis ./ rmse_nom_axis)) * 100;
    improv_lin_total = (1 - (rmse_rls_lin_total / rmse_nom_lin_total)) * 100;
    improv_ang_total = (1 - (rmse_rls_ang_total / rmse_nom_ang_total)) * 100;
end

% --- DISPLAY METRICS ---
fprintf('\n===========================================================\n');
fprintf('             ESTIMATION PERFORMANCE METRICS (RMSE)           \n');
fprintf('===========================================================\n');
fprintf('%-10s | %-12s | %-12s | %-12s\n', 'AXIS', 'NOMINAL', 'RLS', 'IMPROVEMENT');
fprintf('-----------------------------------------------------------\n');
labels_short = {'a_x', 'a_y', 'a_z', 'alpha_x', 'alpha_y', 'alpha_z'};
for k = 1:6
    if PLOT_RLS_DATA
        fprintf('%-10s | %-12.4f | %-12.4f | %+.2f%%\n', ...
            labels_short{k}, rmse_nom_axis(k), rmse_rls_axis(k), improv_axis(k));
    else
        fprintf('%-10s | %-12.4f | %-12s | %-12s\n', ...
            labels_short{k}, rmse_nom_axis(k), 'N/A', 'N/A');
    end
end
fprintf('-----------------------------------------------------------\n');
if PLOT_RLS_DATA
    fprintf('%-10s | %-12.4f | %-12.4f | %+.2f%%\n', 'TOT LIN', rmse_nom_lin_total, rmse_rls_lin_total, improv_lin_total);
    fprintf('%-10s | %-12.4f | %-12.4f | %+.2f%%\n', 'TOT ANG', rmse_nom_ang_total, rmse_rls_ang_total, improv_ang_total);
else
    fprintf('Total metrics require RLS data.\n');
end
fprintf('===========================================================\n\n');


% --- PLOTTING ---
labels = {'a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)', '\alpha_x (rad/s^2)', '\alpha_y (rad/s^2)', '\alpha_z (rad/s^2)'};

% Figure 1: Acceleration Comparison
figure('Name', 'Acceleration Estimation Comparison', 'Position', [100 100 1200 800]);
t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:6
    nexttile;
    plot(t_plot, a_real(:, k), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True'); hold on;
    plot(t_plot, a_nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.5, 'DisplayName', 'Nom');
    
    if PLOT_UKF_DATA
        plot(t_plot, a_ukf(:, k), '-', 'Color', c.ukf, 'LineWidth', 1.5, 'DisplayName', 'UKF');
    end
    
    rls_title_str = '';
    if PLOT_RLS_DATA
        plot(t_plot, a_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS');
        rls_title_str = sprintf(' | RLS RMSE: %.3f (Imp: %+.0f%%)', rmse_rls_axis(k), improv_axis(k));
    end
    
    ylabel(labels{k});
    title(sprintf('%s%s', labels_short{k}, rls_title_str));
    grid on;
    xlim([t_plot(1), t_plot(end)]);
    if k == 1, legend('Location', 'best'); end
end
title(t, sprintf('Acceleration Estimation (Mass=%.2fkg)', Uav_Mass), 'Interpreter', 'none');

% Figure 2: Errors
figure('Name', 'Acceleration Errors', 'Position', [150 150 1200 800]);
t2 = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:6
    nexttile;
    yline(0, 'k-', 'Alpha', 0.3); hold on;
    plot(t_plot, err_nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.0, 'DisplayName', 'Nom Err');
    if PLOT_RLS_DATA
        plot(t_plot, err_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS Err');
    end
    ylabel(['Error ' labels{k}]);
    grid on;
    xlim([t_plot(1), t_plot(end)]);
    if k == 1, legend('Location', 'best'); end
end
title(t2, 'Acceleration Errors (True - Estimated)');
fprintf('Plotting Complete.\n');