% =========================================================================
% GENERATE OVERALL SENSITIVITY REPORT (v6.3 - Split Plots & Sorted by Leakage)
% =========================================================================
% This script loads the results from the stepwise analysis and creates a
% high-level summary report and optional detailed plots.
%
% VERSION 6.3 REFINEMENTS:
% - Split summary into two separate figures (Frequency vs Strength).
% - Sorted summary plots based on Leakage Strength (Avg t-Stat).
% - Renamed legends to 'Wrench Error' and 'Leakage'.
% - Saves summary plots as .fig files.
%
clear; clc; close all;

% --- User Settings ---
HIDE_ZERO_SIGNIFICANCE_PARAMS_IN_PLOT = true;
GENERATE_MODEL_PLOTS = true;
SETPOINTS_TO_PLOT = [1, 3, 15];
P_VALUE_THRESHOLD = 0.05; % Defined for print output context

% --- 1. Load Analysis Results File ---
fprintf('Loading analysis results...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end

[fileName, pathName] = uigetfile(fullfile(startPath, '*.mat'), 'Select the stepwise_analysis_results.mat file');
if isequal(fileName, 0), disp('User selected Cancel.'); return; end
load(fullfile(pathName, fileName));

% --- 2. Tally Parameter Significance and Strength ---
nSetpoints = length(analysis_results);
all_predictors = initial_predictors;

% Initialize count table
predictor_counts = table('Size', [length(all_predictors), 5], ...
                         'VariableTypes', {'cell', 'double', 'double', 'double', 'double'}, ...
                         'VariableNames', {'Parameter', 'ErrorModelCount', 'LeakageModelCount', 'Sum_Abs_tStat_Error', 'Sum_Abs_tStat_Leakage'});
predictor_counts.Parameter = all_predictors';
predictor_counts.ErrorModelCount = zeros(height(predictor_counts), 1);
predictor_counts.LeakageModelCount = zeros(height(predictor_counts), 1);
predictor_counts.Sum_Abs_tStat_Error = zeros(height(predictor_counts), 1);
predictor_counts.Sum_Abs_tStat_Leakage = zeros(height(predictor_counts), 1);

% Loop through setpoints
for i = 1:nSetpoints
    sp_result = analysis_results{i};
    
    % Process Error Model
    if ~isempty(sp_result.ErrorModel)
        mdl = sp_result.ErrorModel;
        surviving_predictors = mdl.PredictorNames;
        for p_idx = 1:length(surviving_predictors)
            p_name = surviving_predictors{p_idx};
            row_idx = strcmp(predictor_counts.Parameter, p_name);
            if any(row_idx)
                predictor_counts.ErrorModelCount(row_idx) = predictor_counts.ErrorModelCount(row_idx) + 1;
                tStat = mdl.Coefficients.tStat(p_name);
                predictor_counts.Sum_Abs_tStat_Error(row_idx) = predictor_counts.Sum_Abs_tStat_Error(row_idx) + abs(tStat);
            end
        end
    end
    
    % Process Leakage Model
    if ~isempty(sp_result.LeakageModel)
        mdl = sp_result.LeakageModel;
        surviving_predictors = mdl.PredictorNames;
        for p_idx = 1:length(surviving_predictors)
            p_name = surviving_predictors{p_idx};
            row_idx = strcmp(predictor_counts.Parameter, p_name);
            if any(row_idx)
                predictor_counts.LeakageModelCount(row_idx) = predictor_counts.LeakageModelCount(row_idx) + 1;
                tStat = mdl.Coefficients.tStat(p_name);
                predictor_counts.Sum_Abs_tStat_Leakage(row_idx) = predictor_counts.Sum_Abs_tStat_Leakage(row_idx) + abs(tStat);
            end
        end
    end
end

% Calculate Totals and Averages
num_error_models = sum(cellfun(@(x) ~isempty(x.ErrorModel), analysis_results));
num_leakage_models = sum(cellfun(@(x) ~isempty(x.LeakageModel), analysis_results));

if num_error_models > 0
    predictor_counts.ErrorModelPercent = (predictor_counts.ErrorModelCount / num_error_models) * 100;
else
    predictor_counts.ErrorModelPercent = zeros(height(predictor_counts),1);
end

if num_leakage_models > 0
    predictor_counts.LeakageModelPercent = (predictor_counts.LeakageModelCount / num_leakage_models) * 100;
else
    predictor_counts.LeakageModelPercent = zeros(height(predictor_counts),1);
end

predictor_counts.Avg_Abs_tStat_Error = predictor_counts.Sum_Abs_tStat_Error ./ predictor_counts.ErrorModelCount;
predictor_counts.Avg_Abs_tStat_Leakage = predictor_counts.Sum_Abs_tStat_Leakage ./ predictor_counts.LeakageModelCount;

% Handle NaNs (divide by zero)
predictor_counts.Avg_Abs_tStat_Error(isnan(predictor_counts.Avg_Abs_tStat_Error)) = 0;
predictor_counts.Avg_Abs_tStat_Leakage(isnan(predictor_counts.Avg_Abs_tStat_Leakage)) = 0;

% --- 3. Display Final Report (Text) ---
fprintf('\n===================================================================\n');
fprintf('           OVERALL PARAMETER SENSITIVITY REPORT\n');
fprintf('===================================================================\n');
fprintf('Significance is defined as a parameter remaining in the final\n');
fprintf('model after backward elimination (p <= %.2f).\n\n', P_VALUE_THRESHOLD);

% Text Report is still useful to see sorted by Error for one section
sorted_error = sortrows(predictor_counts, 'ErrorModelCount', 'descend');
fprintf('--- Significance in PREDICTING ERROR SCORE ---\n');
fprintf('%-22s | %-16s | %s\n', 'Parameter', 'Significant In...', 'Avg Strength');
fprintf('%-22s | %-16s | %s\n', '----------------------', '----------------', '--------------');
for i = 1:height(sorted_error)
    row = sorted_error(i,:);
    fprintf('%-22s | %2d / %2d models (%3.0f%%) | Avg |tStat| = %5.2f\n', strrep(row.Parameter{1}, '_', ' '), row.ErrorModelCount, num_error_models, row.ErrorModelPercent, row.Avg_Abs_tStat_Error);
end

sorted_leakage = sortrows(predictor_counts, 'LeakageModelCount', 'descend');
fprintf('\n--- Significance in PREDICTING LEAKAGE SCORE ---\n');
fprintf('%-22s | %-16s | %s\n', 'Parameter', 'Significant In...', 'Avg Strength');
fprintf('%-22s | %-16s | %s\n', '----------------------', '----------------', '--------------');
for i = 1:height(sorted_leakage)
    row = sorted_leakage(i,:);
    fprintf('%-22s | %2d / %2d models (%3.0f%%) | Avg |tStat| = %5.2f\n', strrep(row.Parameter{1}, '_', ' '), row.LeakageModelCount, num_leakage_models, row.LeakageModelPercent, row.Avg_Abs_tStat_Leakage);
end
fprintf('\n');

% --- 4. Generate Summary Plots (Split & Sorted by Leakage Strength) ---

% SORTING LOGIC: Sort primarily by Leakage Strength (Avg_Abs_tStat_Leakage)
plot_data = sortrows(predictor_counts, 'Avg_Abs_tStat_Leakage', 'descend');

if HIDE_ZERO_SIGNIFICANCE_PARAMS_IN_PLOT
    rows_to_keep = (plot_data.ErrorModelCount > 0) | (plot_data.LeakageModelCount > 0);
    plot_data = plot_data(rows_to_keep, :);
    fprintf('NOTE: Plots filtered to show only %d parameters significant in at least one model.\n', height(plot_data));
end

if ~isempty(plot_data)
    num_plot_params = height(plot_data);
    param_labels = strrep(plot_data.Parameter, '_', ' ');

    % --- FIGURE 1: Frequency of Significance ---
    fig1 = figure('Name', 'Overall Significance Frequency', 'Position', [100, 200, 1000, 500]);
    bar_data_freq = [plot_data.ErrorModelPercent, plot_data.LeakageModelPercent];
    
    b1 = bar(bar_data_freq, 'grouped');
    set(gca, 'XTick', 1:num_plot_params, 'XTickLabel', param_labels);
    xtickangle(45);
    title('Frequency of Significance (Sorted by Leakage Strength)', 'FontSize', 14);
    ylabel('Significant in X% of Models');
    legend('Wrench Error', 'Leakage', 'Location', 'northeast'); % UPDATED LEGEND
    grid on;
    
    % Save Figure 1
    save_path_freq = fullfile(pathName, 'Summary_Frequency.fig');
    savefig(fig1, save_path_freq);
    fprintf('Saved Frequency Plot to: %s\n', save_path_freq);

    % --- FIGURE 2: Average Strength ---
    fig2 = figure('Name', 'Overall Average Strength', 'Position', [150, 150, 1000, 500]);
    bar_data_strength = [plot_data.Avg_Abs_tStat_Error, plot_data.Avg_Abs_tStat_Leakage];
    
    b2 = bar(bar_data_strength, 'grouped');
    set(gca, 'XTick', 1:num_plot_params, 'XTickLabel', param_labels);
    xtickangle(45);
    title('Average Strength of Effect (Sorted by Leakage Strength)', 'FontSize', 14);
    ylabel('Average |t-statistic|');
    legend('Wrench Error', 'Leakage', 'Location', 'northeast'); % UPDATED LEGEND
    grid on;

    % Save Figure 2
    save_path_str = fullfile(pathName, 'Summary_Strength.fig');
    savefig(fig2, save_path_str);
    fprintf('Saved Strength Plot to: %s\n', save_path_str);
end

% --- 5. (Optional) Generate Detailed Model Coefficient Plots ---
if GENERATE_MODEL_PLOTS
    fprintf('\nGenerating detailed coefficient plots for specified setpoints...\n');
    
    plot_folder = fullfile(pathName, 'Coefficient_Plots_v63');
    if ~exist(plot_folder, 'dir'), mkdir(plot_folder); end
    
    for sp_to_plot = SETPOINTS_TO_PLOT
        if sp_to_plot > nSetpoints || sp_to_plot < 1
            warning('Setpoint #%d is out of range. Skipping plot.', sp_to_plot);
            continue;
        end
        
        sp_result = analysis_results{sp_to_plot};
        fig = figure('Name', sprintf('Coefficient Plot for Setpoint #%d', sp_to_plot), 'Position', [300, 300, 800, 700]);
        
        t = tiledlayout(2, 1, 'TileSpacing', 'compact');
        
        % Plot Wrench Error
        ax1 = nexttile;
        plotCoefficients(sp_result.ErrorModel, ax1, 'Wrench Error Score');
        
        % Plot Leakage
        ax2 = nexttile;
        plotCoefficients(sp_result.LeakageModel, ax2, 'Leakage Score');
        
        sgtitle(sprintf('Final Model Coefficients for Setpoint #%d: [%s]', ...
            sp_to_plot, num2str(sp_result.SetpointVector)), 'FontSize', 14, 'FontWeight', 'bold');
        
        % Save as Image (PNG) for quick viewing
        saveas(fig, fullfile(plot_folder, sprintf('Coefficients_SP%02d.png', sp_to_plot)));
        % Save as FIG for editing
        savefig(fig, fullfile(plot_folder, sprintf('Coefficients_SP%02d.fig', sp_to_plot)));
    end
    fprintf('Detailed coefficient plots saved to folder: %s\n', plot_folder);
end

% --- Helper function for plotting coefficients ---
function plotCoefficients(mdl, ax, model_title)
    if isempty(mdl)
        title(ax, [model_title, ': No significant predictors found']);
        set(ax, 'XLim', [0 1], 'YLim', [0 1]);
        text(0.5, 0.5, 'N/A', 'HorizontalAlignment', 'center', 'FontSize', 14);
        return;
    end
    
    coeffs = mdl.Coefficients;
    
    % Remove Intercept if present (usually the first row)
    if strcmp(coeffs.Properties.RowNames{1}, '(Intercept)')
        coeffs = coeffs(2:end, :);
    end
    
    if height(coeffs) == 0
        title(ax, [model_title, ': No significant predictors found']);
        return;
    end
    
    % Sort by Estimate Magnitude for the detailed plot
    [~, sort_idx] = sort(abs(coeffs.Estimate), 'ascend');
    coeffs = coeffs(sort_idx, :);
    
    param_names = strrep(coeffs.Properties.RowNames, '_', ' ');
    y_cats = categorical(param_names, param_names); 
    
    estimates = coeffs.Estimate;
    
    % Robust Confidence Intervals
    ci_matrix = coefCI(mdl);
    % Remove intercept from CI matrix
    ci_predictors = ci_matrix(2:end, :); 
    
    % Reorder CI to match sorted coefficients
    ci_predictors_sorted = ci_predictors(sort_idx, :); 
    ci_lower = ci_predictors_sorted(:, 1);
    ci_upper = ci_predictors_sorted(:, 2);
    
    err_lower = estimates - ci_lower;
    err_upper = ci_upper - estimates;
    
    errorbar(ax, estimates, y_cats, err_lower, err_upper, 'horizontal', 'o', 'MarkerSize', 8, 'LineWidth', 1.5, 'CapSize', 8);
    hold(ax, 'on');
    xline(ax, 0, 'r--', 'LineWidth', 1);
    grid(ax, 'on');
    title(ax, [model_title, ' Model Coefficients']);
    xlabel(ax, 'Effect Size (Estimate)');
    ylabel(ax, 'Significant Predictors');
end