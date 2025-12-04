%% ANALYZE SINGLE SAMPLE (UKF & RLS) - WRENCH DOMAIN
%
% This script selects a SINGLE UKF simulation file and its corresponding
% RLS file, calculates the estimated wrenches, and plots the timeseries
% comparison (True vs Nominal vs UKF vs RLS).
%
% Comparison:
%   - Nominal: Based on initial B_matrix guess.
%   - UKF: Mean estimate from last 5s of simulation.
%   - RLS: Estimate sampled at t = 17s.
%
clear; clc; close all;

%% --- CONFIGURATION ---
% FLAGS: Enable/Disable specific data plots
PLOT_UKF_DATA = true;   % Set true to plot UKF data
PLOT_RLS_DATA = true;   % Set true to plot RLS data (and select RLS file)
RLS_SAMPLE_TIME = 17.0; % Time in seconds to sample the RLS data
TRIM_SECONDS = 5.0;     % For plotting: trim start/end artifacts

% Plot Colors
c.nom = [0.6350 0.0780 0.1840]; % Red (Nominal)
c.ukf = [0.0000 0.4470 0.7410]; % Blue (UKF)
c.rls = [0.4660 0.6740 0.1880]; % Green (RLS)
c.true = [0 0 0];               % Black (True)

%% --- 1. Select Files ---
% 1.1 Select Single UKF File (Mandatory for Ground Truth & Nominal)
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

% 1.2 Find/Select RLS File (Only if enabled)
rlsFileName = 'Skipped';
if PLOT_RLS_DATA
    % Try to auto-detect based on "SampleXXX"
    sampleIdMatch = regexp(ukfFileName, 'Sample\d+', 'match');
    if ~isempty(sampleIdMatch)
        sampleID = sampleIdMatch{1};
        fprintf('Step 2: Select the matching RLS result file (Sample ID: %s)...\n', sampleID);
        [rlsFileName, rlsPath] = uigetfile(fullfile(ukfPath, ['*' sampleID '*.mat']), 'Select RLS File');
    else
        fprintf('Step 2: Select the matching RLS result file...\n');
        [rlsFileName, rlsPath] = uigetfile(fullfile(ukfPath, '*.mat'), 'Select RLS File');
    end
    
    if isequal(rlsFileName, 0)
        disp('Cancelled RLS selection. Proceeding without RLS.'); 
        PLOT_RLS_DATA = false; % Disable if user cancelled
    else
        rlsFullFile = fullfile(rlsPath, rlsFileName);
    end
end

fprintf('Processing:\n  UKF: %s\n  RLS: %s\n', ukfFileName, rlsFileName);

%% --- 2. Process UKF Data ---
S_ukf = load(ukfFullFile);
if ~isfield(S_ukf, 'simOut') || ~isprop(S_ukf.simOut, 'UKFData')
    error('UKF file missing simOut.UKFData');
end
ukfData = S_ukf.simOut.UKFData;
Omega = ukfData.Omega;
Real_Wrench = ukfData.Real_Wrench;
B_nom = S_ukf.B_matrix_nominal;
N_ROTORS = S_ukf.Uav.N_ROTORS;

% UKF Stats (Last 5s) - Calculate even if not plotting to ensure struct validity
endTime = ukfData.UKF_DATA.Time(end);
startTime = max(ukfData.UKF_DATA.Time(1), endTime - 3.0);
ts_subset = getsampleusingtime(ukfData.UKF_DATA, startTime, endTime);

parametersWindow = ts_subset.Data(:, 14+N_ROTORS:end);
est_params_avg = mean(parametersWindow, 1);

idx_B_end = 6 * N_ROTORS;
B_vector = est_params_avg(1 : idx_B_end);
B_Matrix_UKF = reshape(B_vector, [6, N_ROTORS]);

%% --- 3. Process RLS Data ---
if PLOT_RLS_DATA
    S_rls = load(rlsFullFile);
    if ~isprop(S_rls.simOut, 'RLSData')
        error('RLS file missing simOut.RLSData');
    end
    rlsData = S_rls.simOut.RLSData;
    motorParamsTs = rlsData.MotorParams;
    forceEffTs = rlsData.ForceEffectiveness;
    torqueEffTs = rlsData.TorqueEffectiveness;
    
    % Find Index for Sample Time
    [~, time_idx] = min(abs(motorParamsTs.Time - RLS_SAMPLE_TIME));
    if motorParamsTs.Time(end) < (RLS_SAMPLE_TIME - 1.0)
        time_idx = length(motorParamsTs.Time);
        warning('RLS data shorter than requested sample time. Using end value.');
    end
    
    % Extract Matrices
    Est_ForceEff = forceEffTs.Data(:, :, time_idx);
    Est_TorqueEff = torqueEffTs.Data(:, :, time_idx);
    B_Matrix_RLS = [Est_ForceEff; Est_TorqueEff];
end

%% --- 4. Calculate Predictions ---
Omega_sq = Omega.Data .^ 2;

% Nominal
Wrench_Nom_Data = (B_nom * Omega_sq')';
Wrench_Nom = timeseries(Wrench_Nom_Data, Omega.Time);

% UKF
if PLOT_UKF_DATA
    Wrench_UKF_Data = (B_Matrix_UKF * Omega_sq')';
    Wrench_UKF = timeseries(Wrench_UKF_Data, Omega.Time);
end

% RLS
if PLOT_RLS_DATA
    Wrench_RLS_Data = (B_Matrix_RLS * Omega_sq')';
    Wrench_RLS = timeseries(Wrench_RLS_Data, Omega.Time);
end

%% --- 5. Plotting (Timeseries) ---
% Determine Common Time Vector based on enabled plots
start_times = [Real_Wrench.Time(1)];
end_times   = [Real_Wrench.Time(end)];

if PLOT_UKF_DATA
    start_times(end+1) = Wrench_UKF.Time(1);
    end_times(end+1)   = Wrench_UKF.Time(end);
end
if PLOT_RLS_DATA
    start_times(end+1) = Wrench_RLS.Time(1);
    end_times(end+1)   = Wrench_RLS.Time(end);
end

t_start = max(start_times) + TRIM_SECONDS;
t_end   = min(end_times) - TRIM_SECONDS;

if t_end <= t_start
    error('Data too short for trim settings.');
end

time_vec = Real_Wrench.Time;
mask = (time_vec >= t_start) & (time_vec <= t_end);
t_plot = time_vec(mask);

% Resample/Extract for plotting
w_real = resample(Real_Wrench, t_plot).Data;
w_nom  = resample(Wrench_Nom, t_plot).Data;
if PLOT_UKF_DATA
    w_ukf = resample(Wrench_UKF, t_plot).Data;
end
if PLOT_RLS_DATA
    w_rls = resample(Wrench_RLS, t_plot).Data;
end

% --- CALCULATE METRICS (RMSE & IMPROVEMENT) ---
err_nom = w_real - w_nom;
rmse_nom_axis = rms(err_nom, 1);
rmse_nom_force_total = rms(vecnorm(err_nom(:,1:3), 2, 2)); % Total Force Error
rmse_nom_torque_total = rms(vecnorm(err_nom(:,4:6), 2, 2)); % Total Torque Error

% FIX: Calculate UKF Error if UKF is enabled
if PLOT_UKF_DATA
    err_ukf = w_real - w_ukf;
end

if PLOT_RLS_DATA
    err_rls = w_real - w_rls;
    rmse_rls_axis = rms(err_rls, 1);
    rmse_rls_force_total = rms(vecnorm(err_rls(:,1:3), 2, 2));
    rmse_rls_torque_total = rms(vecnorm(err_rls(:,4:6), 2, 2));
    
    % Improvement Factor (%)
    % Formula: (1 - RLS/Nominal) * 100. Positive = RLS is better.
    improv_axis = (1 - (rmse_rls_axis ./ rmse_nom_axis)) * 100;
    improv_force_total = (1 - (rmse_rls_force_total / rmse_nom_force_total)) * 100;
    improv_torque_total = (1 - (rmse_rls_torque_total / rmse_nom_torque_total)) * 100;
end

% --- DISPLAY METRICS TABLE ---
fprintf('\n===========================================================\n');
fprintf('             WRENCH ESTIMATION METRICS (RMSE)              \n');
fprintf('===========================================================\n');
fprintf('%-10s | %-12s | %-12s | %-12s\n', 'AXIS', 'NOMINAL', 'RLS', 'IMPROVEMENT');
fprintf('-----------------------------------------------------------\n');
labels_short = {'F_x', 'F_y', 'F_z', 'Tau_x', 'Tau_y', 'Tau_z'};

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
    fprintf('%-10s | %-12.4f | %-12.4f | %+.2f%%\n', 'TOT FORCE', rmse_nom_force_total, rmse_rls_force_total, improv_force_total);
    fprintf('%-10s | %-12.4f | %-12.4f | %+.2f%%\n', 'TOT TORQ', rmse_nom_torque_total, rmse_rls_torque_total, improv_torque_total);
else
    fprintf('Total metrics require RLS data.\n');
end
fprintf('===========================================================\n\n');

labels = {'F_x (N)', 'F_y (N)', 'F_z (N)', '\tau_x (Nm)', '\tau_y (Nm)', '\tau_z (Nm)'};

% Figure 1: Tracking Comparison
figure('Name', 'Single Sample Wrench Estimation', 'Position', [100 100 1200 800]);
t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:6
    nexttile;
    plot(t_plot, w_real(:, k), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True'); hold on;
    plot(t_plot, w_nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.5, 'DisplayName', 'Nominal');
    
    if PLOT_UKF_DATA
        plot(t_plot, w_ukf(:, k), '-', 'Color', c.ukf, 'LineWidth', 1.5, 'DisplayName', 'UKF');
    end
    
    rls_title_str = '';
    if PLOT_RLS_DATA
        plot(t_plot, w_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS');
        rls_title_str = sprintf(' | RLS RMSE: %.3f (Imp: %+.0f%%)', rmse_rls_axis(k), improv_axis(k));
    end
    
    ylabel(labels{k});
    title(sprintf('%s%s', labels_short{k}, rls_title_str));
    grid on;
    xlim([t_plot(1), t_plot(end)]);
    
    if k == 1
        legend('Location', 'best');
    end
    if k > 3
        xlabel('Time (s)');
    end
end
title(t, sprintf('Wrench Estimation Comparison (File: %s)', ukfFileName), 'Interpreter', 'none');

% Figure 2: Estimation Errors
figure('Name', 'Estimation Errors', 'Position', [150 150 1200 800]);
t2 = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:6
    nexttile;
    yline(0, 'k-', 'Alpha', 0.3); hold on;
    plot(t_plot, err_nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.0, 'DisplayName', 'Nom Err');
    
    if PLOT_UKF_DATA
        plot(t_plot, err_ukf(:, k), '-', 'Color', c.ukf, 'LineWidth', 1.5, 'DisplayName', 'UKF Err');
    end
    
    if PLOT_RLS_DATA
        plot(t_plot, err_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS Err');
    end
    
    ylabel(['Error ' labels{k}]);
    grid on;
    xlim([t_plot(1), t_plot(end)]);
    
    if k == 1
        legend('Location', 'best');
    end
    if k > 3
        xlabel('Time (s)');
    end
end
title(t2, 'Estimation Errors vs Time');

%% --- 6. Plot RLS Parameter Convergence ---
if PLOT_RLS_DATA
    % Configuration for Convergence Plot
    CONV_TRUNCATE_TIME = RLS_SAMPLE_TIME + 2.0; 
    Y_LIMIT_LOOKBACK = 1.0; % Look at the last 1 second to determine Y-scale
    
    % Access raw Force Effectiveness Data [3 x N_Rotors x Time]
    % Row 1 = X-axis specific force (N / rad^2 per rotor)
    rls_time_all = rlsData.ForceEffectiveness.Time;
    
    % Create Mask to truncate data shortly after sample time
    mask_conv = rls_time_all <= CONV_TRUNCATE_TIME;
    t_conv = rls_time_all(mask_conv);
    
    % Extract X-Axis data for all rotors (Row 1)
    % Resulting Size: [N_ROTORS x Time_Steps]
    raw_data_x = rlsData.ForceEffectiveness.Data(1, 1:N_ROTORS, mask_conv);
    x_param_hist = squeeze(raw_data_x); 
    
    % Correct dimensions if squeeze rotated it (specifically for single rotor case)
    if N_ROTORS == 1 && size(x_param_hist, 1) > 1
        x_param_hist = x_param_hist'; 
    end
    
    % Visualization
    figure('Name', 'RLS Convergence: X-Axis Thrust', 'Position', [200 200 800 400]);
    plot(t_conv, x_param_hist, 'LineWidth', 1.5); hold on;
    
    % Add Sample Time Line
    xline(RLS_SAMPLE_TIME, 'r--', 'Sampling Point', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);
    
    % --- DYNAMIC Y-SCALING LOGIC ---
    % Find the indices corresponding to the last few seconds (steady state)
    idx_tail = t_conv >= (t_conv(end) - Y_LIMIT_LOOKBACK);
    
    if any(idx_tail)
        % Get data only from the "settled" region
        data_tail = x_param_hist(:, idx_tail);
        
        % Calculate Min/Max of the tail
        y_min = min(data_tail(:));
        y_max = max(data_tail(:));
        
        % Add 20% padding so lines aren't touching the plot edges
        y_range = y_max - y_min;
        if y_range == 0, y_range = abs(y_max) * 0.1; end % Handle flat lines
        if y_range == 0, y_range = 1.0; end % Handle pure zero lines
        
        padding = 0.2 * y_range;
        ylim([y_min - padding, y_max + padding]);
    end
    % -------------------------------
    
    % Styling
    grid on;
    xlabel('Time (s)');
    ylabel('Thrust Coeff (X)');
    title(sprintf('RLS Convergence: X-Axis Force Coeff (Scaled to Final Values)'));
    
    % Create Legend
    leg_str = cell(1, N_ROTORS);
    for i = 1:N_ROTORS
        leg_str{i} = sprintf('Rotor %d', i);
    end
    legend(leg_str, 'Location', 'best');
    xlim([0, CONV_TRUNCATE_TIME]);
end

fprintf('Plotting Complete.\n');

%% --- 8. Debug Plot: Rotor Speeds Squared (Omega^2) ---
% This is useful to see the raw input driving the wrench models.
figure('Name', 'Debug: Omega Squared', 'Position', [250 250 800 400]);

% Extract Data
omega_sq_data = Omega.Data .^ 2;
time_omega = Omega.Time;

% Plot
plot(time_omega, omega_sq_data, 'LineWidth', 1.2);
grid on;

% Styling
xlabel('Time (s)');
ylabel('\Omega^2 (rad/s)^2');
title('Input Input: Rotor Speeds Squared');

% Dynamic Legend based on N_ROTORS
leg_str = cell(1, N_ROTORS);
for i = 1:N_ROTORS
    leg_str{i} = sprintf('Rotor %d', i);
end
legend(leg_str, 'Location', 'best');

% Align X-Axis with the other plots
xlim([t_plot(1), t_plot(end)]);

fprintf('Debug Plot Added: Omega^2\n');