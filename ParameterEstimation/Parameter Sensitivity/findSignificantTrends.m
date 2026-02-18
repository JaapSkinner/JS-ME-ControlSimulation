% =========================================================================
% AUTOMATED SENSITIVITY TREND HUNTER (v7 - C_TAU Added)
% =========================================================================
% This script systematically analyzes all Monte Carlo results to find
% statistically significant linear relationships.
%
% VERSION 7 REFINEMENTS:
% - Added motor torque constant (C_TAU) variation to the analysis.
%
clear; clc; close all;

% --- User Settings ---
SIGNIFICANCE_THRESHOLD = 0.1;
GENERATE_PLOTS_FOR_TRENDS = true;
SHOW_INSIGNIFICANT_TRENDS = true; 

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

% --- MODIFIED: Added StdDev_CTAU and Range_CTAU ---
varNames = {'SampleID', 'SetpointID', ...
            'CoM_AbsDev_X','CoM_AbsDev_Y','CoM_AbsDev_Z',...
            'StdDev_KV','Range_KV',...
            'StdDev_CTAU', 'Range_CTAU',...
            'StdDev_R','Range_R','StdDev_I0','Range_I0',...
            'Mass_AbsDev','Inertia_AbsDev_Mag',...
            'Error_Fx','Error_Fy','Error_Fz','Error_Tx','Error_Ty','Error_Tz', ...
            'Leakage_Fx','Leakage_Fy','Leakage_Fz','Leakage_Tx','Leakage_Ty','Leakage_Tz'};
varTypes = repmat({'double'}, 1, length(varNames));
mega_table = table('Size', [num_total_rows, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);
max_authority = norm_params.max_authority;
min_thrust_offset = norm_params.min_thrust_offset;

row_idx = 1;
for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    cog_dev = [NaN, NaN, NaN];
    if isfield(data.Sampled_features, 'COM') && isfield(data.Sampled_features.COM, 'per_axis_deviation'), cog_dev = data.Sampled_features.COM.per_axis_deviation; end
    
    for sp = 1:nSetpoints
        mega_table.SampleID(row_idx) = i; mega_table.SetpointID(row_idx) = sp;
        mega_table.CoM_AbsDev_X(row_idx) = abs(cog_dev(1)); mega_table.CoM_AbsDev_Y(row_idx) = abs(cog_dev(2)); mega_table.CoM_AbsDev_Z(row_idx) = abs(cog_dev(3));
        if isfield(data.Sampled_features, 'K_V'), mega_table.StdDev_KV(row_idx) = data.Sampled_features.K_V.std_across_motors; mega_table.Range_KV(row_idx) = data.Sampled_features.K_V.range; end
        
        % --- ADDED: Extract C_TAU features ---
        if isfield(data.Sampled_features, 'C_TAU')
            mega_table.StdDev_CTAU(row_idx) = data.Sampled_features.C_TAU.std_across_motors;
            mega_table.Range_CTAU(row_idx) = data.Sampled_features.C_TAU.range;
        end
        % ------------------------------------

        if isfield(data.Sampled_features, 'R'), mega_table.StdDev_R(row_idx) = data.Sampled_features.R.std_across_motors; mega_table.Range_R(row_idx) = data.Sampled_features.R.range; end
        if isfield(data.Sampled_features, 'I_0'), mega_table.StdDev_I0(row_idx) = data.Sampled_features.I_0.std_across_motors; mega_table.Range_I0(row_idx) = data.Sampled_features.I_0.range; end
        if isfield(data.Sampled_features, 'M'), mega_table.Mass_AbsDev(row_idx) = abs(data.Sampled_features.M.mean_deviation); end
        if isfield(data.Sampled_features, 'I') && isfield(data.Sampled_features.I, 'per_axis_deviation'), mega_table.Inertia_AbsDev_Mag(row_idx) = norm(data.Sampled_features.I.per_axis_deviation); else, mega_table.Inertia_AbsDev_Mag(row_idx) = NaN; end
        
        w_des = data.setpoints(sp, :)'; w_actual = data.wrenches(:, sp);
        error_raw = w_actual - w_des; leakage_raw = w_actual; leakage_raw(w_des ~= 0) = 0;
        error_normalized = error_raw; leakage_normalized = leakage_raw;
        thrust_range = max_authority(3) - min_thrust_offset;
        error_normalized([1,2,4,5,6]) = error_normalized([1,2,4,5,6]) ./ max_authority([1,2,4,5,6]);
        leakage_normalized([1,2,4,5,6]) = leakage_normalized([1,2,4,5,6]) ./ max_authority([1,2,4,5,6]);
        if thrust_range > 0, error_normalized(3) = error_normalized(3) / thrust_range; leakage_normalized(3) = leakage_normalized(3) / thrust_range; end
        error_vars = {'Error_Fx','Error_Fy','Error_Fz','Error_Tx','Error_Ty','Error_Tz'};
        mega_table{row_idx, error_vars} = abs(error_normalized');
        leakage_vars = {'Leakage_Fx','Leakage_Fy','Leakage_Fz','Leakage_Tx','Leakage_Ty','Leakage_Tz'};
        mega_table{row_idx, leakage_vars} = abs(leakage_normalized');
        row_idx = row_idx + 1;
    end
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files aggregated)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Master data table created. Searching for significant trends...\n\n');

% --- 2. Systematically Test All Permutations ---
% --- MODIFIED: Updated index ranges for causes and effects ---
causes = varNames(3:15);  % Now 13 causesTx
effects = varNames(16:end); % Effects start after the causes
% -----------------------------------------------------------

significant_trends = {};
all_r_values = cell(1, nSetpoints); 
for sp = 1:nSetpoints
    subset_table = mega_table(mega_table.SetpointID == sp, :);
    r_table_sp = array2table(NaN(length(effects), length(causes)), 'RowNames', effects, 'VariableNames', causes);
    
    for c = 1:length(causes)
        for e = 1:length(effects)
            x_data = subset_table.(causes{c});
            y_data = subset_table.(effects{e});
            valid_indices = ~isnan(x_data) & ~isnan(y_data);
            if sum(valid_indices) < 10, continue; end
            if range(x_data(valid_indices)) <= eps, continue; end
            
            R = corrcoef(x_data(valid_indices), y_data(valid_indices));
            r_value = R(1,2);
            
            r_table_sp.(causes{c})(effects{e}) = r_value;
            
            if abs(r_value) >= SIGNIFICANCE_THRESHOLD
                trend = struct('SetpointID', sp, 'SetpointVector', setpoints(sp, :), ...
                               'Cause', causes{c}, 'Effect', effects{e}, 'R_Value', r_value);
                significant_trends{end+1} = trend;
            end
        end
    end
    all_r_values{sp} = r_table_sp;
end

% --- 3. Generate Grouped Final Report ---
% (This section is unchanged and will work with the new data)
fprintf('================= SIGNIFICANT TRENDS REPORT =================\n');
fprintf('Threshold for correlation |r| >= %.2f\n', SIGNIFICANCE_THRESHOLD);
if isempty(significant_trends)
    fprintf('\nNo significant trends found at this threshold.\n');
else
    trends_table = struct2table([significant_trends{:}]);
    sorted_trends = sortrows(trends_table, {'SetpointID', 'Effect'});
    fprintf('\nFound %d significant trends:\n', height(sorted_trends));
    current_sp = -1; current_effect = '';
    for i = 1:height(sorted_trends)
        trend = sorted_trends(i, :);
        if trend.SetpointID ~= current_sp
            fprintf('\n============================================================\n');
            fprintf('Significant Trends for Setpoint #%d: [%s]\n', trend.SetpointID, num2str(trend.SetpointVector));
            fprintf('------------------------------------------------------------\n');
            current_sp = trend.SetpointID; current_effect = '';
        end
        if ~strcmp(trend.Effect, current_effect)
            fprintf('\n--> EFFECT: %s\n', strrep(trend.Effect{:}, '_', ' '));
            current_effect = trend.Effect;
        end
        if trend.R_Value > 0, direction = 'Positive'; else, direction = 'Negative'; end
        fprintf('    - CAUSE: %s (r = %.3f, %s)\n', strrep(trend.Cause{:}, '_', ' '), trend.R_Value, direction);
    end
end
fprintf('\n=============================================================\n');

% --- 4. Display Full R-Value Tables for Debugging ---
% (This section is unchanged and will work with the new data)
if SHOW_INSIGNIFICANT_TRENDS
    fprintf('\n\n=============== FULL R-VALUE CORRELATION TABLES ===============\n');
    fprintf('Reading tables: Rows are effects, Columns are causes.\n');
    for sp = 1:nSetpoints
        fprintf('\n--- Correlation Table for Setpoint #%d: [%s] ---\n', sp, num2str(setpoints(sp,:)));
        disp(all_r_values{sp});
    end
    fprintf('=============================================================\n');
end

% --- 5. (Optional) Generate Grouped Plots for Found Trends ---
% (This section is unchanged and will work with the new data)
if GENERATE_PLOTS_FOR_TRENDS && ~isempty(significant_trends)
    plotFolder = fullfile(resultsPath, '/../Significant_Trend_Plots_Grouped');
    if ~exist(plotFolder, 'dir'), mkdir(plotFolder); end
    fprintf('\nGenerating grouped plots for significant trends in folder:\n%s\n', plotFolder);
    
    [unique_groups, ~, group_idx] = unique(sorted_trends(:, {'SetpointID', 'Effect'}), 'rows');
    
    for g = 1:height(unique_groups)
        current_sp = unique_groups.SetpointID(g);
        current_effect = unique_groups.Effect{g};
        
        trends_for_this_plot = sorted_trends(group_idx == g, :);
        num_causes = height(trends_for_this_plot);
        
        fig = figure('Name', sprintf('SP %d - Causes of %s', current_sp, current_effect), ...
                     'Position', [50, 50, 450 * ceil(sqrt(num_causes)), 400 * ceil(sqrt(num_causes))], ...
                     'Visible', 'off');
        sgtitle(sprintf('Significant Causes of %s for Setpoint #%d: [%s]', ...
            strrep(current_effect, '_', ' '), current_sp, num2str(setpoints(current_sp,:))), ...
            'FontSize', 14, 'FontWeight', 'bold');
        
        for c = 1:num_causes
            trend = trends_for_this_plot(c, :);
            current_cause = trend.Cause{:};
            
            subplot(ceil(sqrt(num_causes)), ceil(sqrt(num_causes)), c);
            subset_table = mega_table(mega_table.SetpointID == current_sp, :);
            x_data = subset_table.(current_cause);
            y_data = subset_table.(current_effect);
            
            scatter(x_data, y_data, 25, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
            hold on;
            valid_indices = ~isnan(x_data) & ~isnan(y_data);
            if sum(valid_indices) > 1 && range(x_data(valid_indices)) > eps
                p = polyfit(x_data(valid_indices), y_data(valid_indices), 1);
                y_fit = polyval(p, sort(x_data(valid_indices)));
                plot(sort(x_data(valid_indices)), y_fit, 'r-', 'LineWidth', 2);
            end
            
            grid on;
            title(sprintf('%s (r=%.2f)', strrep(current_cause, '_', ' '), trend.R_Value), 'FontSize', 10);
            xlabel(strrep(current_cause, '_', ' '));
            ylabel(['Normalized ', strrep(current_effect, '_', ' ')]);
        end
        
        filename = sprintf('Group_SP%d_%s.png', current_sp, current_effect);
        saveas(fig, fullfile(plotFolder, filename));
        close(fig);
    end
end

fprintf('Analysis complete.\n');