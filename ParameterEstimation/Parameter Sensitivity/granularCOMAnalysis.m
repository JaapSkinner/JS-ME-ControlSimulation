% =========================================================================
% GRANULAR SENSITIVITY ANALYSIS: CoM vs Wrench Components
% =========================================================================
% This script performs a highly specific analysis to visualize how the three
% axes of Center of Gravity (CoM) deviation independently affect the six
% individual components of the wrench error for a single, chosen setpoint.
%
clear; clc; close all;

% --- User Settings ---
% =========================================================================
% >>> CHOOSE THE SETPOINT TO ANALYZE HERE <<<
%
% SETPOINT GUIDE: [Fx, Fy, Fz, Tx, Ty, Tz] (Normalized Commands)
% --- Single-Axis Tests ---
%  1: Strong forward thrust (Fx=0.7)
%  2: Strong right thrust (Fy=0.7)
%  ... (and so on)
setpoint_to_analyze = 2; % <--- CHANGE THIS VALUE
% =========================================================================

% --- 1. Load Data ---
fprintf('Step 1: Loading all result files...\n');
% This section is simplified for brevity - assumes base parameters are in the workspace
% If not, copy the full setup from the previous script.
if ~exist('Motor', 'var')
    run('ParameterEstimationBaseOL.m'); 
end

resultsPath = uigetdir('', 'Select the Folder Containing Monte Carlo Results');
if isequal(resultsPath, 0), disp('User selected Cancel.'); return; end
resultFiles = dir(fullfile(resultsPath, 'simResult_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No simResult files found.'); end

% --- 2. Aggregate Granular Data for the CHOSEN SETPOINT ---
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
nSetpoints = size(setpoints, 1);
if setpoint_to_analyze > nSetpoints || setpoint_to_analyze < 1
    error('Invalid setpoint_to_analyze. Please choose an index between 1 and %d.', nSetpoints);
end
fprintf('Analyzing performance for Setpoint #%d: [%s]\n', ...
    setpoint_to_analyze, num2str(setpoints(setpoint_to_analyze, :)));

% Pre-allocate the new, more specific table
varNames = {'CoM_AbsDev_X', 'CoM_AbsDev_Y', 'CoM_AbsDev_Z', ...
            'Error_Fx', 'Error_Fy', 'Error_Fz', ...
            'Error_Tx', 'Error_Ty', 'Error_Tz'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [numFiles, length(varNames)], ...
                      'VariableTypes', varTypes, 'VariableNames', varNames);

for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    % Get data for the single chosen setpoint
    sp = setpoint_to_analyze;
    w_des = data.setpoints(sp, :)';
    w_actual = data.wrenches(:, sp);
    
    % Calculate the raw 6x1 error vector
    error_raw = w_actual - w_des;
    
    % Populate the table for this run
    if isfield(data.Sampled_features, 'COM') && isfield(data.Sampled_features.COM, 'per_axis_deviation')
        cog_dev = data.Sampled_features.COM.per_axis_deviation;
        results_table.CoM_AbsDev_X(i) = abs(cog_dev(1));
        results_table.CoM_AbsDev_Y(i) = abs(cog_dev(2));
        results_table.CoM_AbsDev_Z(i) = abs(cog_dev(3));
    end
    
    % Store the magnitude of the error for each component
    results_table.Error_Fx(i) = abs(error_raw(1));
    results_table.Error_Fy(i) = abs(error_raw(2));
    results_table.Error_Fz(i) = abs(error_raw(3));
    results_table.Error_Tx(i) = abs(error_raw(4));
    results_table.Error_Ty(i) = abs(error_raw(5));
    results_table.Error_Tz(i) = abs(error_raw(6));
    
    if mod(i, 500) == 0 || i == numFiles
        fprintf('... Progress: %.0f%% (%d / %d files loaded)\n', (i/numFiles)*100, i, numFiles);
    end
end
fprintf('Data aggregation complete. Generating plots...\n\n');

% --- 3. Generate Granular Sensitivity Plots ---
causes = {'CoM_AbsDev_X', 'CoM_AbsDev_Y', 'CoM_AbsDev_Z'};
effects = {'Error_Fx', 'Error_Fy', 'Error_Fz', 'Error_Tx', 'Error_Ty', 'Error_Tz'};
effect_units = {'(N)', '(N)', '(N)', '(Nm)', '(Nm)', '(Nm)'};

figure('Name', sprintf('Granular CoM Sensitivity | Setpoint %d', setpoint_to_analyze), ...
       'Position', [10, 50, 1800, 900]);
sgtitle(sprintf('Granular Sensitivity: CoM Deviation vs. Wrench Error Components for Setpoint #%d', setpoint_to_analyze), ...
        'FontSize', 16, 'FontWeight', 'bold');

plot_idx = 1;
for c = 1:length(causes)
    for e = 1:length(effects)
        subplot(3, 6, plot_idx);
        
        x_data = results_table.(causes{c});
        y_data = results_table.(effects{e});
        
        scatter(x_data, y_data, 15, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
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
        ylabel(['|', strrep(effects{e}, '_', ' '), '| ', effect_units{e}]);
        
        plot_idx = plot_idx + 1;
    end
end

fprintf('Analysis complete.\n');