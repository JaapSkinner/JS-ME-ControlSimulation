% =========================================================================
% ADVANCED SENSITIVITY ANALYSIS (v2 - Standardized Normalization)
% =========================================================================
% This script uses Multiple Linear Regression to analyze the independent
% effect of each parameter variation on system performance, averaged across
% all setpoints.
%
% VERSION 2 REFINEMENTS:
% - Uses the standardized 'norm_params' struct.
% - Implements advanced normalization for the ErrorScore metric before averaging.
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
fprintf('Step 1: Loading all result files to build data table...\n');
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
fprintf('Found %d result files. Aggregating data for regression analysis...\n', numFiles);

% --- 2. Aggregate Data into a Table ---
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints; nSetpoints = size(setpoints, 1);
is_single_axis = (sum(setpoints~=0,2)==1) | (sum(setpoints~=0,2)==2 & setpoints(:,3)~=0);
single_axis_indices = find(is_single_axis);

varNames = {'Score_Error','Score_Leakage','CoG_AbsDev_X','CoG_AbsDev_Y','CoG_AbsDev_Z','StdDev_KV','Range_KV','StdDev_R','Range_R','StdDev_I0','Range_I0','Mass_AbsDev','Inertia_AbsDev_Mag'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [numFiles, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);

% Extract norm_params for use in the loop
max_authority = norm_params.max_authority;
min_thrust_offset = norm_params.min_thrust_offset;

for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    current_run_errors=zeros(nSetpoints,1); 
    current_run_leakages=[];
    
    for sp = 1:nSetpoints
        w_des=data.setpoints(sp,:)'; 
        w_actual=data.wrenches(:,sp);
        error_raw=w_actual-w_des;
        
        % --- MODIFIED: Advanced Per-Setpoint Error Normalization ---
        error_normalized = error_raw;
        % Normalize Fx, Fy, and all torques by their simple max authority
        error_normalized([1,2,4,5,6]) = error_normalized([1,2,4,5,6]) ./ max_authority([1,2,4,5,6]);
        % For Fz error, scale it by the effective thrust range
        thrust_range = max_authority(3) - min_thrust_offset;
        if thrust_range > 0
            error_normalized(3) = error_normalized(3) / thrust_range;
        end
        current_run_errors(sp) = norm(error_normalized);
        % -----------------------------------------------------------
        
        if ismember(sp,single_axis_indices)
            off_axis_indices=find(w_des==0); 
            norm_leakage=norm(w_actual(off_axis_indices)); 
            norm_des=norm(w_des);
            if norm_des > 1e-6, current_run_leakages(end+1)=(norm_leakage/norm_des)*100; end
        end
    end
    
    results_table.Score_Error(i)=mean(current_run_errors); 
    if ~isempty(current_run_leakages)
        results_table.Score_Leakage(i)=mean(current_run_leakages);
    else
        results_table.Score_Leakage(i) = NaN;
    end
    
    % (Feature extraction is unchanged)
    if isfield(data.Sampled_features,'COM')&&isfield(data.Sampled_features.COM,'per_axis_deviation'), cog_dev=data.Sampled_features.COM.per_axis_deviation; results_table.CoG_AbsDev_X(i)=abs(cog_dev(1)); results_table.CoG_AbsDev_Y(i)=abs(cog_dev(2)); results_table.CoG_AbsDev_Z(i)=abs(cog_dev(3)); end
    if isfield(data.Sampled_features,'K_V'), results_table.StdDev_KV(i)=data.Sampled_features.K_V.std_across_motors; results_table.Range_KV(i)=data.Sampled_features.K_V.range; end
    if isfield(data.Sampled_features,'R'), results_table.StdDev_R(i)=data.Sampled_features.R.std_across_motors; results_table.Range_R(i)=data.Sampled_features.R.range; end
    if isfield(data.Sampled_features,'I_0'), results_table.StdDev_I0(i)=data.Sampled_features.I_0.std_across_motors; results_table.Range_I0(i)=data.Sampled_features.I_0.range; end
    if isfield(data.Sampled_features,'M'), results_table.Mass_AbsDev(i)=abs(data.Sampled_features.M.mean_deviation); end
    if isfield(data.Sampled_features,'I')&&isfield(data.Sampled_features.I,'per_axis_deviation'), results_table.Inertia_AbsDev_Mag(i)=norm(data.Sampled_features.I.per_axis_deviation); else, results_table.Inertia_AbsDev_Mag(i)=NaN; end
    
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files loaded)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Data aggregation complete.\n\n');

% --- 3. Perform Multiple Linear Regression ---
fprintf('Step 2: Building linear models to determine parameter sensitivity...\n\n');

mdl_error = fitlm(results_table, 'Score_Error ~ CoG_AbsDev_X + CoG_AbsDev_Y + CoG_AbsDev_Z + StdDev_KV + Range_KV + StdDev_R + Range_R + StdDev_I0 + Range_I0 + Mass_AbsDev + Inertia_AbsDev_Mag');
disp('----------------- MODEL 1: AVERAGE ERROR SCORE -----------------');
disp(mdl_error);

if ~all(isnan(results_table.LeakageScore))
    mdl_leakage = fitlm(results_table, 'Score_Leakage ~ CoG_AbsDev_X + CoG_AbsDev_Y + CoG_AbsDev_Z + StdDev_KV + Range_KV + StdDev_R + Range_R + StdDev_I0 + Range_I0 + Mass_AbsDev + Inertia_AbsDev_Mag');
    disp('----------------- MODEL 2: AVERAGE LEAKAGE SCORE -----------------');
    disp(mdl_leakage);
else
    mdl_leakage = [];
end

% --- 4. Generate Partial Regression Plots ---
fprintf('Step 3: Generating partial regression plots to visualize independent effects...\n');
predictor_names = mdl_error.PredictorNames; 
num_predictors = length(predictor_names);

figure('Name', 'Partial Regression: Error Score', 'NumberTitle', 'off', 'Position', [50, 50, 1400, 800]);
sgtitle('Independent Effect of Each Parameter on Average Error Score', 'FontSize', 16, 'FontWeight', 'bold');
for i = 1:num_predictors
    subplot(ceil(num_predictors/4), 4, i);
    plotAdded(mdl_error, predictor_names{i});
    title(strrep(predictor_names{i}, '_', ' '));
    xlabel(['Effect of ', strrep(predictor_names{i}, '_', ' ')]);
    ylabel('Effect on Avg Error Score');
    grid on;
end

if ~isempty(mdl_leakage)
    figure('Name', 'Partial Regression: Leakage Score', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 800]);
    sgtitle('Independent Effect of Each Parameter on Average Leakage Score', 'FontSize', 16, 'FontWeight', 'bold');
    for i = 1:num_predictors
        subplot(ceil(num_predictors/4), 4, i);
        plotAdded(mdl_leakage, predictor_names{i});
        title(strrep(predictor_names{i}, '_', ' '));
        xlabel(['Effect of ', strrep(predictor_names{i}, '_', ' ')]);
        ylabel('Effect on Avg Leakage (%)');
        grid on;
    end
end
fprintf('Analysis complete.\n');