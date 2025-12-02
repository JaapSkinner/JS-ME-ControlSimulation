%% ANALYZE SINGLE SAMPLE (UKF & RLS)
%
% This script selects a SINGLE UKF simulation file and its corresponding
% RLS file, calculates the estimated wrenches, and plots the timeseries
% comparison (True vs Nominal vs UKF vs RLS).
%
% Comparison:
%   - Nominal: Based on initial B_matrix guess.
%   - UKF: Mean estimate from last 5s of simulation.
%   - RLS: Estimate sampled at t = 50s.
%
clear; clc; close all;

%% --- CONFIGURATION ---
% FLAGS: Enable/Disable specific data plots
PLOT_UKF_DATA = false;   % Set true to plot UKF data
PLOT_RLS_DATA = true;   % Set true to plot RLS data (and select RLS file)

RLS_SAMPLE_TIME = 50.0; % Time in seconds to sample the RLS data
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
    
    if PLOT_RLS_DATA
        plot(t_plot, w_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS');
    end
    
    ylabel(labels{k});
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

err_nom = w_real - w_nom;
if PLOT_UKF_DATA, err_ukf = w_real - w_ukf; end
if PLOT_RLS_DATA, err_rls = w_real - w_rls; end

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

fprintf('Plotting Complete.\n');