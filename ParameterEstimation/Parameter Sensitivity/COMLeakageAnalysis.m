% =========================================================================
% TORQUE LEAKAGE SENSITIVITY ANALYSIS (v2 - With Normalization)
% =========================================================================
% This script analyzes the relationship between the Center of Gravity
% Z-axis deviation and the resulting unwanted leakage torques when
% commanding pure horizontal thrust. The outputs are now normalized.
%
clear; clc; close all;

% --- User Settings ---
% =========================================================================
% >>> COPY AND PASTE the norm_params struct from the calculateMaxWrench.m script here. <<<
norm_params = struct();
norm_params.max_authority = [4.0416; 4.0416; 40.6237; 1.1631; 1.1631; 1.1290];
norm_params.min_thrust_offset = 0.5312;
% =========================================================================

% --- 1. Load Data ---
fprintf('Step 1: Loading all result files...\n');
if ~exist('Motor', 'var')
    run('ParameterEstimationBaseOL.m'); 
end
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end
resultsPath = uigetdir(startPath, 'Select the Folder Containing Monte Carlo Results');
if isequal(resultsPath, 0), disp('User selected Cancel.'); return; end
resultFiles = dir(fullfile(resultsPath, 'simResult_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No simResult files found in the selected folder.'); end

% --- 2. Identify Key Setpoints ---
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
idx_Fx_test = 1; % Hard-coded based on your setpoint matrix
idx_Fy_test = 2; % Hard-coded based on your setpoint matrix
fprintf('Analyzing Fx thrust test using Setpoint #%d: [%s]\n', idx_Fx_test, num2str(setpoints(idx_Fx_test,:)));
fprintf('Analyzing Fy thrust test using Setpoint #%d: [%s]\n', idx_Fy_test, num2str(setpoints(idx_Fy_test,:)));

% --- 3. Aggregate Leakage Data into a Table ---
varNames = {'CoM_Z_Dev', 'LeakageTx_FxCase', 'LeakageTy_FxCase', 'LeakageTz_FxCase', ...
            'LeakageTx_FyCase', 'LeakageTy_FyCase', 'LeakageTz_FyCase'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [numFiles, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);
max_torques = norm_params.max_authority(4:6); % Extract [max_Tx, max_Ty, max_Tz] for normalization

for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    % Store the input variable (cause)
    results_table.CoM_Z_Dev(i) = data.Sampled_features.COM.per_axis_deviation(3);
    
    % --- Process Fx Thrust Case ---
    w_des_fx = data.setpoints(idx_Fx_test, :)';
    w_actual_fx = data.wrenches(:, idx_Fx_test);
    error_raw_fx = w_actual_fx - w_des_fx;
    leakage_torques_raw_fx = error_raw_fx(4:6); % [Tx, Ty, Tz] in Nm
    % Normalize the leakage torques
    results_table{i, {'LeakageTx_FxCase', 'LeakageTy_FxCase', 'LeakageTz_FxCase'}} = (leakage_torques_raw_fx ./ max_torques)';
    
    % --- Process Fy Thrust Case ---
    w_des_fy = data.setpoints(idx_Fy_test, :)';
    w_actual_fy = data.wrenches(:, idx_Fy_test);
    error_raw_fy = w_actual_fy - w_des_fy;
    leakage_torques_raw_fy = error_raw_fy(4:6); % [Tx, Ty, Tz] in Nm
    % Normalize the leakage torques
    results_table{i, {'LeakageTx_FyCase', 'LeakageTy_FyCase', 'LeakageTz_FyCase'}} = (leakage_torques_raw_fy ./ max_torques)';
    
    if mod(i, 500) == 0 || i == numFiles
        fprintf('... Progress: %.0f%% (%d / %d files loaded)\n', (i/numFiles)*100, i, numFiles);
    end
end
fprintf('Data aggregation complete. Generating plots...\n\n');

% --- 4. Generate Leakage Sensitivity Plots ---
y_label_str = 'Normalized Leakage Torque';

% Figure 1: Leakage for Fx Command vs. CoM Z-Dev
figure('Name', 'Leakage Sensitivity for Fx Command', 'Position', [100, 200, 1500, 500]);
sgtitle(sprintf('Torque Leakage vs. CoM Z-Deviation for Fx Thrust Command (Setpoint #%d)', idx_Fx_test), 'FontSize', 14, 'FontWeight', 'bold');
x_data = results_table.CoM_Z_Dev;

% Subplot 1: Tx Leakage
subplot(1, 3, 1);
y_data_tx = results_table.LeakageTx_FxCase;
scatter(x_data, y_data_tx, 15, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
p = polyfit(x_data, y_data_tx, 1);
plot(x_data, polyval(p, x_data), 'r-', 'LineWidth', 2);
grid on;
title('Roll Torque Leakage (Tx)');
xlabel('CoM Z-Deviation (m)');
ylabel(y_label_str);
set(gca, 'FontSize', 11);

% Subplot 2: Ty Leakage
subplot(1, 3, 2);
y_data_ty = results_table.LeakageTy_FxCase;
scatter(x_data, y_data_ty, 15, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
p = polyfit(x_data, y_data_ty, 1);
plot(x_data, polyval(p, x_data), 'r-', 'LineWidth', 2);
grid on;
title('Pitch Torque Leakage (Ty)');
xlabel('CoM Z-Deviation (m)');
ylabel(y_label_str);
set(gca, 'FontSize', 11);

% Subplot 3: Tz Leakage
subplot(1, 3, 3);
y_data_tz = results_table.LeakageTz_FxCase;
scatter(x_data, y_data_tz, 15, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
p = polyfit(x_data, y_data_tz, 1);
plot(x_data, polyval(p, x_data), 'r-', 'LineWidth', 2);
grid on;
title('Yaw Torque Leakage (Tz)');
xlabel('CoM Z-Deviation (m)');
ylabel(y_label_str);
set(gca, 'FontSize', 11);

% Figure 2: Leakage for Fy Command vs. CoM Z-Dev
figure('Name', 'Leakage Sensitivity for Fy Command', 'Position', [150, 250, 1500, 500]);
sgtitle(sprintf('Torque Leakage vs. CoM Z-Deviation for Fy Thrust Command (Setpoint #%d)', idx_Fy_test), 'FontSize', 14, 'FontWeight', 'bold');

% Subplot 1: Tx Leakage
subplot(1, 3, 1);
y_data_tx = results_table.LeakageTx_FyCase;
scatter(x_data, y_data_tx, 15, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
p = polyfit(x_data, y_data_tx, 1);
plot(x_data, polyval(p, x_data), 'r-', 'LineWidth', 2);
grid on;
title('Roll Torque Leakage (Tx)');
xlabel('CoM Z-Deviation (m)');
ylabel(y_label_str);
set(gca, 'FontSize', 11);

% Subplot 2: Ty Leakage
subplot(1, 3, 2);
y_data_ty = results_table.LeakageTy_FyCase;
scatter(x_data, y_data_ty, 15, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
p = polyfit(x_data, y_data_ty, 1);
plot(x_data, polyval(p, x_data), 'r-', 'LineWidth', 2);
grid on;
title('Pitch Torque Leakage (Ty)');
xlabel('CoM Z-Deviation (m)');
ylabel(y_label_str);
set(gca, 'FontSize', 11);

% Subplot 3: Tz Leakage
subplot(1, 3, 3);
y_data_tz = results_table.LeakageTz_FyCase;
scatter(x_data, y_data_tz, 15, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
p = polyfit(x_data, y_data_tz, 1);
plot(x_data, polyval(p, x_data), 'r-', 'LineWidth', 2);
grid on;