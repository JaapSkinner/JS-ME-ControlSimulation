% =========================================================================
% PER-SETPOINT STEPWISE SENSITIVITY ANALYSIS (v4 - Robust Data Aggregation)
% =========================================================================
% This script performs a rigorous, per-setpoint Multiple Linear Regression
% analysis using backward elimination.
%
% VERSION 4 REFINEMENTS:
% - Rewritten data aggregation loop to be more robust and guarantee all
%   parameters from 'varNames' are correctly processed.
%
clear; clc; close all;

% --- User Settings ---
P_VALUE_THRESHOLD = 0.05;
norm_params = struct();
norm_params.max_authority = [3.4015; 3.3103; 24.4154; 0.4242; 0.3990; 0.0555];
norm_params.min_thrust_offset = 6.7169;

% --- 1. Load Data and Build Master Data Table ---
fprintf('Step 1: Loading all result files to build master data table...\n');
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
if numFiles == 0, error('No simResult files found.'); end

firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
nSetpoints = size(setpoints, 1);
num_total_rows = numFiles * nSetpoints;

varNames = {'SampleID', 'SetpointID', ...
            'CoM_AbsDev_X','CoM_AbsDev_Y','CoM_AbsDev_Z',...
            'MeanDev_KV','StdDev_KV', 'MeanDev_KE','StdDev_KE', 'MeanDev_CTAU','StdDev_CTAU', ...
            'MeanDev_B','StdDev_B', 'MeanDev_Voltoffset','StdDev_Voltoffset', ...
            'MeanDev_voltslope','StdDev_voltslope', 'MeanDev_R','StdDev_R', ...
            'MeanDev_I0','StdDev_I0', 'MeanDev_DPROP','StdDev_DPROP', ...
            'MeanDev_DUAV', 'MeanDev_M', 'MeanDev_RHOAIR', 'MeanDev_RPROP', ...
            'MeanDev_AUAV', 'MeanDev_APROP', 'MeanDev_ZETA', 'Inertia_AbsDev_Mag',...
            'ErrorScore', 'LeakageScore'};
varTypes = repmat({'double'}, 1, length(varNames));
mega_table = table('Size', [num_total_rows, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);

max_authority = norm_params.max_authority;
min_thrust_offset = norm_params.min_thrust_offset;

row_idx = 1;
for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    features = data.Sampled_features;
    
    % --- NEW (MORE ROBUST) FEATURE EXTRACTION ---
    temp_features = table();
    cog_dev = getfield_safe(features, 'COM.per_axis_deviation', [NaN, NaN, NaN]);
    temp_features.CoM_AbsDev_X = abs(cog_dev(1));
    temp_features.CoM_AbsDev_Y = abs(cog_dev(2));
    temp_features.CoM_AbsDev_Z = abs(cog_dev(3));
    temp_features.Inertia_AbsDev_Mag = norm(getfield_safe(features, 'I.per_axis_deviation', [NaN, NaN, NaN]));
    
    per_motor_list = {'K_V', 'K_E', 'C_TAU', 'B', 'Volt_offset', 'volt_slope', 'R', 'I_0', 'D_PROP'};
    for p = per_motor_list
        p_name = p{:};
        temp_features.(['MeanDev_', strrep(p_name, '_', '')]) = getfield_safe(features, [p_name, '.mean_deviation'], NaN);
        temp_features.(['StdDev_', strrep(p_name, '_', '')]) = getfield_safe(features, [p_name, '.std_across_motors'], NaN);
    end
    
    scalar_list = {'D_UAV', 'M', 'RHO_AIR', 'R_PROP', 'A_UAV', 'A_PROP', 'ZETA'};
    for p = scalar_list
        p_name = p{:};
        temp_features.(['MeanDev_', strrep(p_name, '_', '')]) = getfield_safe(features, [p_name, '.mean_deviation'], NaN);
    end
    
    for sp = 1:nSetpoints
        mega_table.SampleID(row_idx) = i;
        mega_table.SetpointID(row_idx) = sp;
        
        for var = temp_features.Properties.VariableNames
            mega_table.(var{:})(row_idx) = temp_features.(var{:});
        end
        
        w_des = data.setpoints(sp, :)'; w_actual = data.wrenches(:, sp);
        error_raw = w_actual - w_des;
        error_normalized = error_raw;
        error_normalized([1,2,4,5,6]) = error_normalized([1,2,4,5,6]) ./ max_authority([1,2,4,5,6]);
        thrust_range = max_authority(3) - min_thrust_offset;
        if thrust_range > 0, error_normalized(3) = error_normalized(3) / thrust_range; end
        mega_table.ErrorScore(row_idx) = norm(error_normalized);
        
        is_single_axis = (sum(w_des~=0)==1) || (sum(w_des~=0)==2 && w_des(3)~=0);
        if is_single_axis
            off_axis_indices=find(w_des==0); norm_leakage=norm(w_actual(off_axis_indices)); norm_des=norm(w_des);
            if norm_des > 1e-6, mega_table.LeakageScore(row_idx)=(norm_leakage/norm_des)*100; else, mega_table.LeakageScore(row_idx)=NaN; end
        else, mega_table.LeakageScore(row_idx) = NaN; end
        
        row_idx = row_idx + 1;
    end
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files aggregated)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Master data table created. Performing analysis...\n\n');

% --- 2. Perform Per-Setpoint Stepwise Analysis ---
initial_predictors = varNames(3:end-2);
analysis_results = cell(nSetpoints, 1);
for sp = 1:nSetpoints
    fprintf('======================================================================================\n');
    fprintf('           Stepwise Regression for Setpoint #%d: [%s]\n', sp, num2str(setpoints(sp,:)));
    fprintf('======================================================================================\n');
    
    subset_table = mega_table(mega_table.SetpointID == sp, :);
    
    fprintf('\n--- MODEL 1: Predicting Normalized Error Score ---\n');
    [final_mdl_error, removed_log_error] = performBackwardElimination(subset_table, 'ErrorScore', initial_predictors, P_VALUE_THRESHOLD);
    
    fprintf('\n--- MODEL 2: Predicting Leakage Score (%%) ---\n');
    if ~all(isnan(subset_table.LeakageScore))
        [final_mdl_leakage, removed_log_leakage] = performBackwardElimination(subset_table, 'LeakageScore', initial_predictors, P_VALUE_THRESHOLD);
    else
        fprintf('Leakage analysis skipped for this setpoint (not a single-axis command).\n');
        final_mdl_leakage = []; removed_log_leakage = {};
    end
    
    sp_results = struct();
    sp_results.SetpointID = sp;
    sp_results.SetpointVector = setpoints(sp, :);
    sp_results.ErrorModel = final_mdl_error;
    sp_results.ErrorModelRemovedLog = removed_log_error;
    sp_results.LeakageModel = final_mdl_leakage;
    sp_results.LeakageModelRemovedLog = removed_log_leakage;
    analysis_results{sp} = sp_results;
end

% --- 3. Save Final Results to File ---
output_filename = fullfile(resultsPath, '/../stepwise_analysis_results.mat');
save(output_filename, 'analysis_results', 'initial_predictors', 'P_VALUE_THRESHOLD');
fprintf('\n============================================================\n');
fprintf('Analysis complete. All simplified models saved to:\n%s\n', output_filename);
fprintf('You can now run the ''generateSensitivityReport.m'' script.\n');

% --- Helper function for Backward Elimination ---
function [final_mdl, removed_log] = performBackwardElimination(data_table, response_var, initial_predictors, p_threshold)
    current_predictors = initial_predictors;
    removed_log = {};
    nan_cols = all(isnan(data_table{:, current_predictors}));
    current_predictors(nan_cols) = [];
    if isempty(current_predictors) || all(isnan(data_table.(response_var)))
        final_mdl = []; 
        return;
    end
    
    while true
        formula = [response_var ' ~ ' strjoin(current_predictors, ' + ')];
        mdl = fitlm(data_table, formula);
        p_values = mdl.Coefficients.pValue(2:end);
        [max_p, max_p_idx] = max(p_values);
        if max_p > p_threshold && length(current_predictors) > 1
            removed_predictor = current_predictors{max_p_idx};
            removed_log{end+1} = sprintf('Removed ''%s'' (p = %.3f)', removed_predictor, max_p);
            current_predictors(max_p_idx) = [];
        else, break; end
    end
    
    if ~isempty(removed_log), fprintf('Backward Elimination Steps:\n');
        for k = 1:length(removed_log), fprintf('  %d. %s\n', k, removed_log{k}); end
    else, fprintf('No predictors were removed from the full model.\n'); end
    fprintf('\n--- FINAL MODEL ---\n');
    disp(mdl);
    final_mdl = mdl;
end

% --- Helper function to safely get data from nested structs ---
function val = getfield_safe(s, field, default)
    try
        val = eval(['s.' field]);
    catch
        val = default;
    end
end