% =========================================================================
% PER-SETPOINT MULTIVARIABLE SENSITIVITY ANALYSIS
% =========================================================================
% This script performs a rigorous, per-setpoint Multiple Linear Regression
% analysis to determine the independent effect of each parameter variation
% on performance for each specific maneuver.
%
clear; clc; close all;

% --- User Settings ---
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

% --- Define the structure of our master data table ---
varNames = {'SampleID', 'SetpointID', ...
            'CoM_AbsDev_X','CoM_AbsDev_Y','CoM_AbsDev_Z',...
            'MeanDev_KV','StdDev_KV', ...
            'MeanDev_CTAU','StdDev_CTAU', ...
            'MeanDev_R','StdDev_R', ...
            'MeanDev_I0','StdDev_I0', ...
            'Mass_AbsDev','Inertia_AbsDev_Mag',...
            'ErrorScore', 'LeakageScore'};
varTypes = repmat({'double'}, 1, length(varNames));
mega_table = table('Size', [num_total_rows, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);

% Extract norm_params for use in the loop
max_authority = norm_params.max_authority;
min_thrust_offset = norm_params.min_thrust_offset;

row_idx = 1;
for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    % --- Extract all parameter variation features for this sample ---
    features = data.Sampled_features;
    cog_dev = [NaN, NaN, NaN];
    if isfield(features, 'COM') && isfield(features.COM, 'per_axis_deviation'), cog_dev = features.COM.per_axis_deviation; end
    
    % Create a temporary struct to hold this sample's features
    sample_features = struct();
    sample_features.CoM_AbsDev_X = abs(cog_dev(1));
    sample_features.CoM_AbsDev_Y = abs(cog_dev(2));
    sample_features.CoM_AbsDev_Z = abs(cog_dev(3));
    
    motor_param_list = {'K_V', 'C_TAU', 'R', 'I_0'};
    for p_idx = 1:length(motor_param_list)
        p_name = motor_param_list{p_idx};
        mean_dev_name = ['MeanDev_', strrep(p_name, '_', '')]; % e.g., MeanDev_KV
        std_dev_name = ['StdDev_', strrep(p_name, '_', '')];   % e.g., StdDev_KV
        if isfield(features, p_name)
            sample_features.(mean_dev_name) = features.(p_name).mean_deviation;
            sample_features.(std_dev_name) = features.(p_name).std_across_motors;
        else
            sample_features.(mean_dev_name) = NaN;
            sample_features.(std_dev_name) = NaN;
        end
    end
    
    if isfield(features, 'M'), sample_features.Mass_AbsDev = abs(features.M.mean_deviation); else, sample_features.Mass_AbsDev = NaN; end
    if isfield(features, 'I'), sample_features.Inertia_AbsDev_Mag = norm(features.I.per_axis_deviation); else, sample_features.Inertia_AbsDev_Mag = NaN; end
    
    % --- Loop through setpoints for this sample ---
    for sp = 1:nSetpoints
        mega_table.SampleID(row_idx) = i;
        mega_table.SetpointID(row_idx) = sp;
        
        % Fill in the parameter variations (these are the same for all setpoints in a sample)
        fn = fieldnames(sample_features);
        for k=1:numel(fn)
            mega_table.(fn{k})(row_idx) = sample_features.(fn{k});
        end
        
        % Calculate and fill in the performance scores for this specific setpoint
        w_des = data.setpoints(sp, :)';
        w_actual = data.wrenches(:, sp);
        error_raw = w_actual - w_des;
        
        error_normalized = error_raw;
        error_normalized([1,2,4,5,6]) = error_normalized([1,2,4,5,6]) ./ max_authority([1,2,4,5,6]);
        thrust_range = max_authority(3) - min_thrust_offset;
        if thrust_range > 0
            error_normalized(3) = error_normalized(3) / thrust_range;
        end
        mega_table.ErrorScore(row_idx) = norm(error_normalized);
        
        is_single_axis = (sum(w_des~=0)==1) || (sum(w_des~=0)==2 && w_des(3)~=0);
        if is_single_axis
            off_axis_indices=find(w_des==0);
            norm_leakage=norm(w_actual(off_axis_indices));
            norm_des=norm(w_des);
            if norm_des > 1e-6, mega_table.LeakageScore(row_idx)=(norm_leakage/norm_des)*100;
            else, mega_table.LeakageScore(row_idx)=NaN; end
        else
            mega_table.LeakageScore(row_idx) = NaN;
        end
        
        row_idx = row_idx + 1;
    end
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files aggregated)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Master data table created. Performing analysis...\n\n');

% --- 3. Perform Per-Setpoint Multivariable Analysis ---
predictor_vars = varNames(3:end-2); % All columns between IDs and Scores
formula_base = strjoin(predictor_vars, ' + ');

for sp = 1:nSetpoints
    fprintf('======================================================================================\n');
    fprintf('           MLR Results for Setpoint #%d: [%s]\n', sp, num2str(setpoints(sp,:)));
    fprintf('======================================================================================\n');
    
    % Get the data for only this setpoint
    subset_table = mega_table(mega_table.SetpointID == sp, :);
    
    % --- Build and Display Model for Error Score ---
    fprintf('\n--- MODEL 1: Predicting Normalized Error Score ---\n');
    formula_error = ['ErrorScore ~ ' formula_base];
    mdl_error = fitlm(subset_table, formula_error);
    disp(mdl_error);
    
    % --- Build and Display Model for Leakage Score ---
    fprintf('\n--- MODEL 2: Predicting Leakage Score (%%) ---\n');
    if ~all(isnan(subset_table.LeakageScore))
        formula_leakage = ['LeakageScore ~ ' formula_base];
        mdl_leakage = fitlm(subset_table, formula_leakage);
        disp(mdl_leakage);
    else
        fprintf('Leakage analysis skipped for this setpoint (not a single-axis command).\n');
    end
end
fprintf('Analysis complete.\n');