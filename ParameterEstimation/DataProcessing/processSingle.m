%% ANALYZE SINGLE SAMPLE (UKF & RLS) - WRENCH DOMAIN
%
% This script selects a SINGLE UKF simulation file and its corresponding
% RLS file, calculates the estimated wrenches, and plots the timeseries
% comparison (True vs Nominal vs UKF vs RLS).
%
% Comparison:
%   - Nominal: Based on initial B_matrix guess.
%   - UKF: Mean estimate from last 3s (default) of simulation.
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

% UKF Stats (Last 3s Window)
endTime = ukfData.UKF_DATA.Time(end);
ukf_avg_window_start = max(ukfData.UKF_DATA.Time(1), endTime - 3.0); % 3.0s window
ts_subset = getsampleusingtime(ukfData.UKF_DATA, ukf_avg_window_start, endTime);

% Extract Parameters
% Params usually start after states (14 + N_Rotors)
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

% UKF Metrics
if PLOT_UKF_DATA
    err_ukf = w_real - w_ukf;
    rmse_ukf_axis = rms(err_ukf, 1);
    rmse_ukf_force_total = rms(vecnorm(err_ukf(:,1:3), 2, 2));
    rmse_ukf_torque_total = rms(vecnorm(err_ukf(:,4:6), 2, 2));

    % UKF Improvement Factor (%)
    improv_ukf_axis = (1 - (rmse_ukf_axis ./ rmse_nom_axis)) * 100;
    improv_ukf_force_total = (1 - (rmse_ukf_force_total / rmse_nom_force_total)) * 100;
    improv_ukf_torque_total = (1 - (rmse_ukf_torque_total / rmse_nom_torque_total)) * 100;
end

% RLS Metrics
if PLOT_RLS_DATA
    err_rls = w_real - w_rls;
    rmse_rls_axis = rms(err_rls, 1);
    rmse_rls_force_total = rms(vecnorm(err_rls(:,1:3), 2, 2));
    rmse_rls_torque_total = rms(vecnorm(err_rls(:,4:6), 2, 2));
    
    % RLS Improvement Factor (%)
    improv_rls_axis = (1 - (rmse_rls_axis ./ rmse_nom_axis)) * 100;
    improv_rls_force_total = (1 - (rmse_rls_force_total / rmse_nom_force_total)) * 100;
    improv_rls_torque_total = (1 - (rmse_rls_torque_total / rmse_nom_torque_total)) * 100;
end

% --- DISPLAY METRICS TABLE ---
fprintf('\n========================================================================================\n');
fprintf('                           WRENCH ESTIMATION METRICS (RMSE)                             \n');
fprintf('========================================================================================\n');
fprintf('%-6s | %-10s | %-10s | %-10s | %-12s | %-12s\n', 'AXIS', 'NOMINAL', 'UKF', 'RLS', 'UKF IMP(%)', 'RLS IMP(%)');
fprintf('----------------------------------------------------------------------------------------\n');
labels_short = {'F_x', 'F_y', 'F_z', 'Tau_x', 'Tau_y', 'Tau_z'};

for k = 1:6
    % Prepare strings for UKF
    if PLOT_UKF_DATA
        s_ukf = sprintf('%-10.4f', rmse_ukf_axis(k));
        s_ukf_imp = sprintf('%+.2f%%', improv_ukf_axis(k));
    else
        s_ukf = 'N/A'; s_ukf_imp = 'N/A';
    end

    % Prepare strings for RLS
    if PLOT_RLS_DATA
        s_rls = sprintf('%-10.4f', rmse_rls_axis(k));
        s_rls_imp = sprintf('%+.2f%%', improv_rls_axis(k));
    else
        s_rls = 'N/A'; s_rls_imp = 'N/A';
    end

    fprintf('%-6s | %-10.4f | %-10s | %-10s | %-12s | %-12s\n', ...
            labels_short{k}, rmse_nom_axis(k), s_ukf, s_rls, s_ukf_imp, s_rls_imp);
end

fprintf('----------------------------------------------------------------------------------------\n');

% Summary Rows (Total Force/Torque)
if PLOT_UKF_DATA
    s_ukf_f = sprintf('%-10.4f', rmse_ukf_force_total); s_ukf_imp_f = sprintf('%+.2f%%', improv_ukf_force_total);
    s_ukf_t = sprintf('%-10.4f', rmse_ukf_torque_total); s_ukf_imp_t = sprintf('%+.2f%%', improv_ukf_torque_total);
else
    s_ukf_f = 'N/A'; s_ukf_imp_f = 'N/A';
    s_ukf_t = 'N/A'; s_ukf_imp_t = 'N/A';
end

if PLOT_RLS_DATA
    s_rls_f = sprintf('%-10.4f', rmse_rls_force_total); s_rls_imp_f = sprintf('%+.2f%%', improv_rls_force_total);
    s_rls_t = sprintf('%-10.4f', rmse_rls_torque_total); s_rls_imp_t = sprintf('%+.2f%%', improv_rls_torque_total);
else
    s_rls_f = 'N/A'; s_rls_imp_f = 'N/A';
    s_rls_t = 'N/A'; s_rls_imp_t = 'N/A';
end

fprintf('%-6s | %-10.4f | %-10s | %-10s | %-12s | %-12s\n', 'TOT F', rmse_nom_force_total, s_ukf_f, s_rls_f, s_ukf_imp_f, s_rls_imp_f);
fprintf('%-6s | %-10.4f | %-10s | %-10s | %-12s | %-12s\n', 'TOT T', rmse_nom_torque_total, s_ukf_t, s_rls_t, s_ukf_imp_t, s_rls_imp_t);
fprintf('========================================================================================\n\n');

labels = {'F_x (N)', 'F_y (N)', 'F_z (N)', '\tau_x (Nm)', '\tau_y (Nm)', '\tau_z (Nm)'};

% Figure 1: Tracking Comparison
figure('Name', 'Single Sample Wrench Estimation', 'Position', [100 100 1200 800]);
t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:6
    nexttile;
    plot(t_plot, w_real(:, k), 'k-', 'LineWidth', 1.5, 'DisplayName', 'True'); hold on;
    plot(t_plot, w_nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.5, 'DisplayName', 'Nominal');
    
    title_parts = {};
    
    % UKF Plot & Title
    if PLOT_UKF_DATA
        plot(t_plot, w_ukf(:, k), '-', 'Color', c.ukf, 'LineWidth', 1.5, 'DisplayName', 'UKF');
        title_parts{end+1} = sprintf('UKF: %.3f (%+.0f%%)', rmse_ukf_axis(k), improv_ukf_axis(k));
    end
    
    % RLS Plot & Title
    if PLOT_RLS_DATA
        plot(t_plot, w_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS');
        title_parts{end+1} = sprintf('RLS: %.3f (%+.0f%%)', rmse_rls_axis(k), improv_rls_axis(k));
    end
    
    % Construct Title
    if isempty(title_parts)
        full_title = labels_short{k};
    else
        % Join metrics with separator
        full_title = sprintf('%s\n%s', labels_short{k}, strjoin(title_parts, ' | '));
    end
    
    ylabel(labels{k});
    title(full_title, 'FontSize', 9);
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
    Y_LIMIT_LOOKBACK = 1.0; 
    
    rls_time_all = rlsData.ForceEffectiveness.Time;
    mask_conv = rls_time_all <= CONV_TRUNCATE_TIME;
    t_conv = rls_time_all(mask_conv);
    
    % Extract X-Axis data (Row 1)
    raw_data_x = rlsData.ForceEffectiveness.Data(1, 1:N_ROTORS, mask_conv);
    x_param_hist = squeeze(raw_data_x); 
    if N_ROTORS == 1 && size(x_param_hist, 1) > 1, x_param_hist = x_param_hist'; end
    
    figure('Name', 'RLS Convergence: X-Axis Thrust', 'Position', [200 200 800 400]);
    plot(t_conv, x_param_hist, 'LineWidth', 1.5); hold on;
    xline(RLS_SAMPLE_TIME, 'r--', 'Sampling Point', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);
    
    % Scale Y-Axis based on settled region
    idx_tail = t_conv >= (t_conv(end) - Y_LIMIT_LOOKBACK);
    if any(idx_tail)
        data_tail = x_param_hist(:, idx_tail);
        y_min = min(data_tail(:)); y_max = max(data_tail(:));
        y_range = y_max - y_min;
        if y_range == 0, y_range = 1.0; end
        padding = 0.2 * y_range;
        ylim([y_min - padding, y_max + padding]);
    end
    
    grid on; xlabel('Time (s)'); ylabel('Thrust Coeff (X)');
    title('RLS Convergence: X-Axis Force Coeff');
    leg_str = cell(1, N_ROTORS);
    for i = 1:N_ROTORS, leg_str{i} = sprintf('Rotor %d', i); end
    legend(leg_str, 'Location', 'best');
    xlim([0, CONV_TRUNCATE_TIME]);
end

%% --- 7. Plot UKF Parameter Convergence (Z-AXIS | ZOOMED) ---
if PLOT_UKF_DATA
    figure('Name', 'UKF Convergence: Z-Axis Thrust', 'Position', [220 220 800 400]);
    
    % 1. Get raw UKF parameter time history
    ukf_time_all = ukfData.UKF_DATA.Time;
    ukf_params_all = ukfData.UKF_DATA.Data(:, 14+N_ROTORS:end);
    
    % 2. Extract Z-Axis coefficients (Row 3 of B Matrix)
    % Indices: 3, 9, 15... (3rd element of every 6-row block)
    indices_z = 3:6:(6*N_ROTORS);
    ukf_z_params = ukf_params_all(:, indices_z);
    
    % 3. Plot Trajectories
    plot(ukf_time_all, ukf_z_params, 'LineWidth', 1.5); hold on;
    
    % 4. Visualize Averaging Window
    yl_initial = ylim; % Capture initial just for the patch creation (will override later)
    patch([ukf_avg_window_start, endTime, endTime, ukf_avg_window_start], ...
          [yl_initial(1), yl_initial(1), yl_initial(2), yl_initial(2)], ...
          'k', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    xline(ukf_avg_window_start, 'k--', 'Averaging Start', 'LabelVerticalAlignment', 'bottom');
    
    % 5. Scaling (STRICT ZOOM) - Based ONLY on the averaging window
    % This ensures initial startup transients do not skew the plot
    mask_scale = ukf_time_all >= ukf_avg_window_start;
    
    if any(mask_scale)
        data_window = ukf_z_params(mask_scale, :);
        
        y_min = min(data_window(:));
        y_max = max(data_window(:));
        y_range = y_max - y_min;
        
        % Safety: If converged to a perfect flat line, give it some room
        if y_range < 1e-7
            y_range = max(abs(y_max) * 0.05, 1e-6); 
        end
        
        padding = 0.8 * y_range;
        
        % Force the Y-limits to "Zoom In" on the steady state
        ylim([y_min - padding, y_max + padding]);
        
        % Update Patch Y-coords to match new limits so it covers the whole height
        % (Matlab patches don't auto-update Y when YLim changes)
        yl_new = [y_min - padding, y_max + padding];
        all_patches = findobj(gca, 'Type', 'Patch');
        if ~isempty(all_patches)
            all_patches(1).Vertices(1,2) = yl_new(1);
            all_patches(1).Vertices(2,2) = yl_new(1);
            all_patches(1).Vertices(3,2) = yl_new(2);
            all_patches(1).Vertices(4,2) = yl_new(2);
        end
    end
    
    grid on;
    xlabel('Time (s)');
    ylabel('Thrust Coeff (Z)');
    title(sprintf('UKF Convergence: Z-Axis Force Coeff (Zoomed on Final %.1fs)', endTime - ukf_avg_window_start));
    
    % Legend
    leg_str = cell(1, N_ROTORS);
    for i = 1:N_ROTORS
        leg_str{i} = sprintf('Rotor %d', i);
    end
    legend(leg_str, 'Location', 'best');
    xlim([0, endTime]);
end

%% --- 8. Debug Plot: Rotor Speeds Squared (Omega^2) ---
% This is useful to see the raw input driving the wrench models.
figure('Name', 'Debug: Omega Squared', 'Position', [250 250 800 400]);
omega_sq_data = Omega.Data .^ 2;
time_omega = Omega.Time;
plot(time_omega, omega_sq_data, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\Omega^2 (rad/s)^2');
title('Input Input: Rotor Speeds Squared');
leg_str = cell(1, N_ROTORS);
for i = 1:N_ROTORS, leg_str{i} = sprintf('Rotor %d', i); end
legend(leg_str, 'Location', 'best');
xlim([t_plot(1), t_plot(end)]);

fprintf('Plotting Complete.\n');