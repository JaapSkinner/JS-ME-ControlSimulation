% =========================================================================
% AUTOMATED SENSITIVITY TREND HUNTER
% =========================================================================
% This script systematically analyzes all Monte Carlo results to find
% statistically significant linear relationships between specific parameter
% variations ("causes") and specific wrench error components ("effects") for
% each maneuver ("setpoint").
%
% It generates a summary report of only the most significant trends.
%
clear; clc; close all;

% --- User Settings ---
% =========================================================================
% Define the threshold for what is considered a "significant" trend.
% The correlation coefficient 'r' ranges from -1 to 1. A value's
% magnitude |r| > 0.5 is generally considered a moderate-to-strong correlation.
SIGNIFICANCE_THRESHOLD = 0.5;

% Set to true to automatically save a plot for each significant trend found.
GENERATE_PLOTS_FOR_TRENDS = true;
% =========================================================================

% --- 1. Load Data and Build Master Data Table ---
fprintf('Step 1: Loading all result files to build master data table...\n');
run('ParameterEstimationBaseOL.m'); % Load nominals for context
resultsPath = uigetdir(fullfile(matlab.project.rootProject().RootFolder, 'Results', 'ParameterEstimation'), 'Select the Folder Containing Monte Carlo Results');
if isequal(resultsPath, 0), disp('User selected Cancel.'); return; end
resultFiles = dir(fullfile(resultsPath, 'simResult_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No simResult files found.'); end

firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
nSetpoints = size(setpoints, 1);
num_total_rows = numFiles * nSetpoints;

% Pre-allocate a "mega-table" to hold all data from all runs and setpoints
varNames = {'SampleID', 'SetpointID', ...
            'CoM_AbsDev_X','CoM_AbsDev_Y','CoM_AbsDev_Z',...
            'StdDev_KV','Range_KV','StdDev_R','Range_R','StdDev_I0','Range_I0',...
            'Mass_AbsDev','Inertia_AbsDev_Mag',...
            'Error_Fx','Error_Fy','Error_Fz','Error_Tx','Error_Ty','Error_Tz', ...
            'Leakage_Fx','Leakage_Fy','Leakage_Fz','Leakage_Tx','Leakage_Ty','Leakage_Tz'};
varTypes = repmat({'double'}, 1, length(varNames));
mega_table = table('Size', [num_total_rows, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);

row_idx = 1;
for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    % Get parameter variations for this sample (they are the same for all setpoints in this file)
    cog_dev = [NaN, NaN, NaN];
    if isfield(data.Sampled_features, 'COM') && isfield(data.Sampled_features.COM, 'per_axis_deviation'), cog_dev = data.Sampled_features.COM.per_axis_deviation; end
    
    for sp = 1:nSetpoints
        mega_table.SampleID(row_idx) = i;
        mega_table.SetpointID(row_idx) = sp;
        
        % Fill in parameter data
        mega_table.CoM_AbsDev_X(row_idx) = abs(cog_dev(1));
        mega_table.CoM_AbsDev_Y(row_idx) = abs(cog_dev(2));
        mega_table.CoM_AbsDev_Z(row_idx) = abs(cog_dev(3));
        if isfield(data.Sampled_features, 'K_V'), mega_table.StdDev_KV(row_idx) = data.Sampled_features.K_V.std_across_motors; mega_table.Range_KV(row_idx) = data.Sampled_features.K_V.range; end
        if isfield(data.Sampled_features, 'R'), mega_table.StdDev_R(row_idx) = data.Sampled_features.R.std_across_motors; mega_table.Range_R(row_idx) = data.Sampled_features.R.range; end
        if isfield(data.Sampled_features, 'I_0'), mega_table.StdDev_I0(row_idx) = data.Sampled_features.I_0.std_across_motors; mega_table.Range_I0(row_idx) = data.Sampled_features.I_0.range; end
        if isfield(data.Sampled_features, 'M'), mega_table.Mass_AbsDev(row_idx) = abs(data.Sampled_features.M.mean_deviation); end
        if isfield(data.Sampled_features, 'I') && isfield(data.Sampled_features.I, 'per_axis_deviation'), mega_table.Inertia_AbsDev_Mag(row_idx) = norm(data.Sampled_features.I.per_axis_deviation); else, mega_table.Inertia_AbsDev_Mag(row_idx) = NaN; end

        % Calculate and fill in error/leakage data
        w_des = data.setpoints(sp, :)';
        w_actual = data.wrenches(:, sp);
        error_raw = w_actual - w_des;
        
        mega_table{row_idx, 'Error_Fx':'Error_Tz'} = abs(error_raw');
        
        leakage = w_actual;
        leakage(w_des ~= 0) = 0; % Zero out the on-axis components
        mega_table{row_idx, 'Leakage_Fx':'Leakage_Tz'} = abs(leakage');
        
        row_idx = row_idx + 1;
    end
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files aggregated)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Master data table created. Searching for significant trends...\n\n');

% --- 2. Systematically Test All Permutations ---
causes = varNames(3:13); % Names of all parameter variation columns
effects = varNames(14:end); % Names of all error/leakage columns

significant_trends = {}; % Use a cell array to store findings

for sp = 1:nSetpoints
    subset_table = mega_table(mega_table.SetpointID == sp, :);
    
    for c = 1:length(causes)
        for e = 1:length(effects)
            x_data = subset_table.(causes{c});
            y_data = subset_table.(effects{e});
            
            valid_indices = ~isnan(x_data) & ~isnan(y_data);
            if sum(valid_indices) < 10, continue; end % Skip if not enough data
            
            R = corrcoef(x_data(valid_indices), y_data(valid_indices));
            r_value = R(1,2);
            
            if abs(r_value) >= SIGNIFICANCE_THRESHOLD
                % Trend found! Store it.
                trend = struct();
                trend.SetpointID = sp;
                trend.SetpointVector = setpoints(sp, :);
                trend.Cause = causes{c};
                trend.Effect = effects{e};
                trend.R_Value = r_value;
                significant_trends{end+1} = trend;
            end
        end
    end
end

% --- 3. Generate Final Report ---
fprintf('================= SIGNIFICANT TRENDS REPORT =================\n');
fprintf('Threshold for correlation |r| >= %.2f\n', SIGNIFICANCE_THRESHOLD);
if isempty(significant_trends)
    fprintf('\nNo significant trends found at this threshold.\n');
else
    fprintf('\nFound %d significant trends:\n\n', length(significant_trends));
    for i = 1:length(significant_trends)
        trend = significant_trends{i};
        fprintf('--- Trend #%d ---\n', i);
        fprintf('    Setpoint #%d: [%s]\n', trend.SetpointID, num2str(trend.SetpointVector));
        fprintf('    CAUSE:  %s\n', strrep(trend.Cause, '_', ' '));
        fprintf('    EFFECT: %s\n', strrep(trend.Effect, '_', ' '));
        fprintf('    CORRELATION (r): %.3f (%s)\n', trend.R_Value, ...
            iif(trend.R_Value > 0, 'Positive', 'Negative'));
        fprintf('\n');
    end
end
fprintf('=============================================================\n');

% --- 4. (Optional) Generate Plots for Found Trends ---
if GENERATE_PLOTS_FOR_TRENDS && ~isempty(significant_trends)
    plotFolder = fullfile(resultsPath, 'Significant_Trend_Plots');
    if ~exist(plotFolder, 'dir'), mkdir(plotFolder); end
    fprintf('\nGenerating plots for significant trends in folder:\n%s\n', plotFolder);
    
    for i = 1:length(significant_trends)
        trend = significant_trends{i};
        
        subset_table = mega_table(mega_table.SetpointID == trend.SetpointID, :);
        x_data = subset_table.(trend.Cause);
        y_data = subset_table.(trend.Effect);
        
        fig = figure('Visible', 'off');
        scatter(x_data, y_data, 25, 'b', 'filled', 'MarkerFaceAlpha', 0.4);
        hold on;
        
        valid_indices = ~isnan(x_data) & ~isnan(y_data);
        p = polyfit(x_data(valid_indices), y_data(valid_indices), 1);
        y_fit = polyval(p, sort(x_data(valid_indices)));
        plot(sort(x_data(valid_indices)), y_fit, 'r-', 'LineWidth', 2);
        
        grid on;
        title(sprintf('Setpoint #%d: %s vs. %s', trend.SetpointID, ...
            strrep(trend.Effect, '_', ' '), strrep(trend.Cause, '_', ' ')), 'FontSize', 10);
        xlabel(strrep(trend.Cause, '_', ' '));
        ylabel(strrep(trend.Effect, '_', ' '));
        
        filename = sprintf('Trend_%02d_SP%d_%s_vs_%s.png', i, trend.SetpointID, trend.Effect, trend.Cause);
        saveas(fig, fullfile(plotFolder, filename));
        close(fig);
    end
end

fprintf('Analysis complete.\n');