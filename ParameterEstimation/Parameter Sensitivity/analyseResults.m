% =========================================================================
% ANALYZE MONTE CARLO PARAMETER SENSITIVITY (v7 - Final w/ Progress Bar)
% =========================================================================
% This script performs a sensitivity analysis on the Monte Carlo results.
%
% VERSION 7 REFINEMENTS:
% - Added an infrequent progress indicator to the main processing loop.
%
clear; clc; close all;

% --- User Settings ---
verbose = false; % Set to 'true' to see detailed progress and summaries in the console

% --- 1. Get Maximum Wrench Authority for Normalization ---
if verbose, fprintf('Step 1: Calculating Maximum Wrench Authority...\n'); end
run('ParameterEstimationBaseOL.m'); 
Motor_nom = Motor; Uav_nom = Uav; N_motors = Uav.N_ROTORS;
params_nom = struct();
u_min_nom = [-1.0;-1.0;0.0;-1.0;-1.0;-1.0]; u_max_nom = [1.0;1.0;1.0;1.0;1.0;1.0];
alpha_Tx_nom = Motor.CommandMixing(2)/10000; alpha_Ty_nom = Motor.CommandMixing(3)/10000; alpha_Tz_nom = Motor.CommandMixing(4)/10000;
P_nom = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 1 0 0 0; 1 0 0 0 0 0; 0 1 0 0 0 0];
A_scale_nom = diag([alpha_Tx_nom, alpha_Ty_nom, alpha_Tz_nom, 1, 1, 1]);
params_nom.A_eff = A_scale_nom * P_nom;
params_nom.u_min = u_min_nom; params_nom.u_max = u_max_nom; params_nom.M = Motor.mixingMatrix; params_nom.N = N_motors;
params_nom.Kt=Motor_nom.K_T; params_nom.Ke=Motor_nom.K_E; params_nom.R=Motor_nom.R; params_nom.C_tau=Motor_nom.C_TAU;
params_nom.mv=Motor_nom.volt_slope; params_nom.V_off=Motor_nom.Volt_offset; params_nom.I0=Motor_nom.I_0;
kf_vec_nom=0.5*Uav_nom.RHO_AIR*Aero.Cz3P.coefs(1)*(Uav_nom.D_PROP.^2).*Uav_nom.A_PROP; km_vec_nom=Motor_nom.C_TAU;
B_matrix_nom=zeros(6,N_motors);
for m=1:N_motors
    t_m = [0; 0; -kf_vec_nom(m)];
    tau_m = [0; 0; -Uav_nom.ROTOR_DIRECTION(m) * km_vec_nom(m)];
    R_m_b = Uav_nom.R_MOTOR_TO_BODY(:,:,m);
    T_body_i=R_m_b*t_m; tau_body_i=R_m_b*tau_m; r_i=Uav_nom.MotorLoc(m,1:3)';
    tau_from_thrust=cross(r_i,T_body_i); B_matrix_nom(:,m)=[T_body_i;tau_body_i+tau_from_thrust];
end
params_nom.B_matrix = B_matrix_nom;
setpoints_max_test=[1,0,0.5,0,0,0;-1,0,0.5,0,0,0;0,1,0.5,0,0,0;0,-1,0.5,0,0,0;0,0,1,0,0,0;0,0,0,0,0,0;0,0,0.5,1,0,0;0,0,0.5,-1,0,0;0,0,0.5,0,1,0;0,0,0.5,0,-1,0;0,0,0.5,0,0,1;0,0,0.5,0,0,-1];
results_max=zeros(6,size(setpoints_max_test,1));
for i=1:size(setpoints_max_test,1), results_max(:,i)=computeWrench(setpoints_max_test(i,:)',params_nom); end
max_authority = max(abs(results_max), [], 2);
max_authority(max_authority < 1e-6) = 1; 

% --- 2. Select Results Folder and Load Data ---
if verbose, fprintf('\nStep 2: Loading all result files...\n'); end
resultsPath = uigetdir('', 'Select the Folder Containing Monte Carlo Results');
if isequal(resultsPath, 0), disp('User selected Cancel.'); return; end
resultFiles = dir(fullfile(resultsPath, 'simResult_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No simResult files found.'); end
fprintf('Found %d result files. Processing, please wait...\n', numFiles);

% --- 3. Aggregate Data into a Single Table ---
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
nSetpoints = size(setpoints, 1);
is_single_axis = (sum(setpoints~=0,2)==1) | (sum(setpoints~=0,2)==2 & setpoints(:,3)~=0);
single_axis_indices = find(is_single_axis);

varNames = {'Score_Error', 'Score_Leakage', ...
            'CoG_Dev_X', 'CoG_Dev_Y', 'CoG_Dev_Z', ...
            'StdDev_KV', 'Range_KV', 'StdDev_R', 'Range_R', ...
            'StdDev_I0', 'Range_I0', 'Mass', 'Inertia_Dev_Magnitude'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [numFiles, length(varNames)], ...
                      'VariableTypes', varTypes, 'VariableNames', varNames);

for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    current_run_errors = zeros(nSetpoints, 1);
    current_run_leakages = [];
    for sp = 1:nSetpoints
        w_des = data.setpoints(sp, :)';
        w_actual = data.wrenches(:, sp);
        error_raw = w_actual - w_des;
        current_run_errors(sp) = norm(error_raw ./ max_authority);
        if ismember(sp, single_axis_indices)
            off_axis_indices = find(w_des == 0);
            norm_leakage = norm(w_actual(off_axis_indices));
            norm_des = norm(w_des);
            if norm_des > 1e-6, current_run_leakages(end+1)=(norm_leakage/norm_des)*100; end
        end
    end
    results_table.Score_Error(i) = mean(current_run_errors);
    results_table.Score_Leakage(i) = mean(current_run_leakages);
    
    if isfield(data.Sampled_features, 'COM') && isfield(data.Sampled_features.COM, 'per_axis_deviation')
        cog_dev = data.Sampled_features.COM.per_axis_deviation;
        results_table.CoG_Dev_X(i) = cog_dev(1); results_table.CoG_Dev_Y(i) = cog_dev(2); results_table.CoG_Dev_Z(i) = cog_dev(3);
    end
    if isfield(data.Sampled_features, 'K_V')
        results_table.StdDev_KV(i) = data.Sampled_features.K_V.std_across_motors;
        results_table.Range_KV(i)  = data.Sampled_features.K_V.range;
    end
    if isfield(data.Sampled_features, 'R')
        results_table.StdDev_R(i) = data.Sampled_features.R.std_across_motors;
        results_table.Range_R(i)  = data.Sampled_features.R.range;
    end
    if isfield(data.Sampled_features, 'I_0')
        results_table.StdDev_I0(i) = data.Sampled_features.I_0.std_across_motors;
        results_table.Range_I0(i)  = data.Sampled_features.I_0.range;
    end
    if isfield(data.Sampled_features, 'M')
        results_table.Mass(i) = data.Sampled_features.M.mean_deviation;
    end
    if isfield(data.Sampled_features, 'I') && isfield(data.Sampled_features.I, 'per_axis_deviation')
        results_table.Inertia_Dev_Magnitude(i) = norm(data.Sampled_features.I.per_axis_deviation);
    else
        results_table.Inertia_Dev_Magnitude(i) = NaN;
    end
    
    % --- NEW: Infrequent Progress Indicator ---
    % This will print an update every 100 files, and for the very last file.
    if mod(i, 500) == 0 || i == numFiles
        fprintf('... Progress: %.0f%% (%d / %d files processed)\n', (i/numFiles)*100, i, numFiles);
    end
end
fprintf('Processing complete.\n\n');

% --- 4. Display Final Statistical Summary ---
fprintf('================ OVERALL STATISTICAL SUMMARY ================\n');
fprintf('Based on the average scores from %d unique parameter sets.\n\n', numFiles);
fprintf('--- Average Error Score Distribution ---\n');
fprintf('  Mean: %.4f | Std Dev: %.4f | Median: %.4f | Max: %.4f\n', ...
    mean(results_table.Score_Error, 'omitnan'), std(results_table.Score_Error, 'omitnan'), median(results_table.Score_Error, 'omitnan'), max(results_table.Score_Error));
fprintf('\n--- Average Leakage Score Distribution ---\n');
fprintf('  Mean: %.2f%% | Std Dev: %.2f%% | Median: %.2f%% | Max: %.2f%%\n', ...
    mean(results_table.Score_Leakage, 'omitnan'), std(results_table.Score_Leakage, 'omitnan'), median(results_table.Score_Leakage, 'omitnan'), max(results_table.Score_Leakage));
fprintf('===========================================================\n');


% --- 5. Generate Sensitivity Plots ---
parameters_to_plot = results_table.Properties.VariableNames(3:end);
num_params = length(parameters_to_plot);

% Plot 1: Sensitivity vs. Average Error Score
figure('Name', 'Sensitivity Analysis: Error Score', 'NumberTitle', 'off', 'Position', [50, 50, 1400, 800]);
sgtitle('Sensitivity Analysis: Impact of Parameter Variation on Average Error Score', 'FontSize', 16, 'FontWeight', 'bold');
for i = 1:num_params
    param_name = parameters_to_plot{i};
    x_data = results_table.(param_name);
    y_data = results_table.Score_Error;
    subplot(ceil(num_params/4), 4, i);
    scatter(x_data, y_data, 25, 'b', 'filled', 'MarkerFaceAlpha', 0.4);
    hold on;
    valid_indices = ~isnan(x_data) & ~isnan(y_data);
    if sum(valid_indices) > 1 
        p = polyfit(x_data(valid_indices), y_data(valid_indices), 1);
        y_fit = polyval(p, sort(x_data(valid_indices)));
        plot(sort(x_data(valid_indices)), y_fit, 'r-', 'LineWidth', 2);
        R = corrcoef(x_data(valid_indices), y_data(valid_indices));
        r_value = R(1,2);
        title(sprintf('%s (r = %.3f)', strrep(param_name, '_', ' '), r_value));
    else
        title(sprintf('%s (not enough data)', strrep(param_name, '_', ' ')));
    end
    grid on;
    xlabel(strrep(param_name, '_', ' '));
    ylabel('Avg Error Score');
end

% Plot 2: Sensitivity vs. Average Leakage Score
figure('Name', 'Sensitivity Analysis: Leakage Score', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 800]);
sgtitle('Sensitivity Analysis: Impact of Parameter Variation on Average Leakage Score', 'FontSize', 16, 'FontWeight', 'bold');
for i = 1:num_params
    param_name = parameters_to_plot{i};
    x_data = results_table.(param_name);
    y_data = results_table.Score_Leakage;
    subplot(ceil(num_params/4), 4, i);
    scatter(x_data, y_data, 25, 'b', 'filled', 'MarkerFaceAlpha', 0.4);
    hold on;
    valid_indices = ~isnan(x_data) & ~isnan(y_data);
    if sum(valid_indices) > 1
        p = polyfit(x_data(valid_indices), y_data(valid_indices), 1);
        y_fit = polyval(p, sort(x_data(valid_indices)));
        plot(sort(x_data(valid_indices)), y_fit, 'r-', 'LineWidth', 2);
        R = corrcoef(x_data(valid_indices), y_data(valid_indices));
        r_value = R(1,2);
        title(sprintf('%s (r = %.3f)', strrep(param_name, '_', ' '), r_value));
    else
        title(sprintf('%s (not enough data)', strrep(param_name, '_', ' ')));
    end
    grid on;
    xlabel(strrep(param_name, '_', ' '));
    ylabel('Avg Leakage Score (%)');
end