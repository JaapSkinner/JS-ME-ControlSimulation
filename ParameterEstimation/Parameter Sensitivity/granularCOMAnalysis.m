% =========================================================================
% GRANULAR SENSITIVITY ANALYSIS: CoM vs Wrench Components (Combined & Normalized)
% =========================================================================
% This script performs a highly specific analysis to visualize how CoM
% deviation affects wrench error, combining data from both Fx and Fy
% horizontal thrust tests to find more general trends.
%
% VERSION 2 REFINEMENTS:
% - Uses the standardized 'norm_params' struct.
% - Normalizes the granular error components before plotting.
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
if numFiles == 0, error('No simResult files found.'); end

% --- 2. Aggregate Granular Data for COMBINED SETPOINTS ---
setpoints_to_analyze = [1, 2]; % Analyze Fx (1) and Fy (2) tests
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
fprintf('Analyzing and combining performance for Setpoints #%d and #%d\n', ...
    setpoints_to_analyze(1), setpoints_to_analyze(2));

num_total_points = numFiles * length(setpoints_to_analyze);
varNames = {'CoM_AbsDev_X', 'CoM_AbsDev_Y', 'CoM_AbsDev_Z', ...
            'Error_Fx', 'Error_Fy', 'Error_Fz', ...
            'Error_Tx', 'Error_Ty', 'Error_Tz'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [num_total_points, length(varNames)], ...
                      'VariableTypes', varTypes, 'VariableNames', varNames);

% Extract norm_params for use in the loop
max_authority = norm_params.max_authority;
min_thrust_offset = norm_params.min_thrust_offset;
                      
table_row_idx = 1; % Initialize a counter for the table rows
for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    cog_dev = [NaN, NaN, NaN];
    if isfield(data.Sampled_features, 'COM') && isfield(data.Sampled_features.COM, 'per_axis_deviation')
        cog_dev = data.Sampled_features.COM.per_axis_deviation;
    end
    
    for sp = setpoints_to_analyze
        w_des = data.setpoints(sp, :)';
        w_actual = data.wrenches(:, sp);
        error_raw = w_actual - w_des;
        
        % --- MODIFIED: Normalize the raw error vector ---
        error_normalized = error_raw;
        error_normalized([1,2,4,5,6]) = error_normalized([1,2,4,5,6]) ./ max_authority([1,2,4,5,6]);
        thrust_range = max_authority(3) - min_thrust_offset;
        if thrust_range > 0
            error_normalized(3) = error_normalized(3) / thrust_range;
        end
        % ---------------------------------------------
        
        results_table.CoM_AbsDev_X(table_row_idx) = abs(cog_dev(1));
        results_table.CoM_AbsDev_Y(table_row_idx) = abs(cog_dev(2));
        results_table.CoM_AbsDev_Z(table_row_idx) = abs(cog_dev(3));
        
        % Store the magnitude of the NORMALIZED error for each component
        results_table.Error_Fx(table_row_idx) = abs(error_normalized(1));
        results_table.Error_Fy(table_row_idx) = abs(error_normalized(2));
        results_table.Error_Fz(table_row_idx) = abs(error_normalized(3));
        results_table.Error_Tx(table_row_idx) = abs(error_normalized(4));
        results_table.Error_Ty(table_row_idx) = abs(error_normalized(5));
        results_table.Error_Tz(table_row_idx) = abs(error_normalized(6));
        
        table_row_idx = table_row_idx + 1;
    end
    
    if mod(i, 500) == 0 || i == numFiles
        fprintf('... Progress: %.0f%% (%d / %d files processed)\n', (i/numFiles)*100, i, numFiles);
    end
end
fprintf('Data aggregation complete. Generating plots...\n\n');

% --- 3. Generate Granular Sensitivity Plots ---
causes = {'CoM_AbsDev_X', 'CoM_AbsDev_Y', 'CoM_AbsDev_Z'};
effects = {'Error_Fx', 'Error_Fy', 'Error_Fz', 'Error_Tx', 'Error_Ty', 'Error_Tz'};

% --- MODIFIED: Update plot labels for normalized data ---
y_label_str = '|Normalized Error|';

figure('Name', sprintf('Granular CoM Sensitivity | Setpoints %d & %d', setpoints_to_analyze(1), setpoints_to_analyze(2)), ...
       'Position', [10, 50, 1800, 900]);
sgtitle(sprintf('Granular Sensitivity (Combined Fx & Fy Thrust Setpoints #%d & #%d)', setpoints_to_analyze(1), setpoints_to_analyze(2)), ...
        'FontSize', 16, 'FontWeight', 'bold');

plot_idx = 1;
for c = 1:length(causes)
    for e = 1:length(effects)
        subplot(3, 6, plot_idx);
        
        x_data = results_table.(causes{c});
        y_data = results_table.(effects{e});
        
        scatter(x_data, y_data, 15, 'b', 'filled', 'MarkerFaceAlpha', 0.2);
        hold on;
        
        valid_indices = ~isnan(x_data) & ~isnan(y_data);
        if sum(valid_indices) > 1
            p = polyfit(x_data(valid_indices), y_data(valid_indices), 1);
            y_fit = polyval(p, sort(x_data(valid_indices)));
            plot(sort(x_data(valid_indices)), y_fit, 'r-', 'LineWidth', 2);
            R = corrcoef(x_data(valid_indices), y_data(valid_indices));
            r_value = R(1,2);
            title_str = sprintf('%s vs %s (r=%.2f)', effects{e}, causes{c}, r_value);
        else
            title_str = sprintf('%s vs %s (no data)', effects{e}, causes{c});
        end
        
        grid on;
        title(strrep(title_str, '_', ' '));
        xlabel(['|', strrep(causes{c}, '_', ' '), '| (m)']);
        ylabel(y_label_str);
        
        plot_idx = plot_idx + 1;
    end
end
fprintf('Analysis complete.\n');