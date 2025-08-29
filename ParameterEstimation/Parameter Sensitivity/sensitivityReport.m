% =========================================================================
% GENERATE OVERALL SENSITIVITY REPORT (v5.1 - Plot Label Fix)
% =========================================================================
% This script loads the results from the stepwise analysis and creates a
% high-level summary report.
%
% VERSION 5.1 REFINEMENTS:
% - Fixed a plotting issue where MATLAB would hide some x-axis labels.
%   The script now forces all parameter labels to be displayed on the chart.
%
clear; clc; close all;

% --- User Settings ---
HIDE_ZERO_SIGNIFICANCE_PARAMS_IN_PLOT = true;

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
predictor_counts = table('Size', [length(all_predictors), 5], ...
                         'VariableTypes', {'cell', 'double', 'double', 'double', 'double'}, ...
                         'VariableNames', {'Parameter', 'ErrorModelCount', 'LeakageModelCount', 'Sum_Abs_tStat_Error', 'Sum_Abs_tStat_Leakage'});
predictor_counts.Parameter = all_predictors';
predictor_counts.ErrorModelCount = zeros(height(predictor_counts), 1);
predictor_counts.LeakageModelCount = zeros(height(predictor_counts), 1);
predictor_counts.Sum_Abs_tStat_Error = zeros(height(predictor_counts), 1);
predictor_counts.Sum_Abs_tStat_Leakage = zeros(height(predictor_counts), 1);

for i = 1:nSetpoints
    sp_result = analysis_results{i};
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

num_error_models = sum(cellfun(@(x) ~isempty(x.ErrorModel), analysis_results));
num_leakage_models = sum(cellfun(@(x) ~isempty(x.LeakageModel), analysis_results));
if num_error_models > 0, predictor_counts.ErrorModelPercent = (predictor_counts.ErrorModelCount / num_error_models) * 100; else, predictor_counts.ErrorModelPercent = zeros(height(predictor_counts),1); end
if num_leakage_models > 0, predictor_counts.LeakageModelPercent = (predictor_counts.LeakageModelCount / num_leakage_models) * 100; else, predictor_counts.LeakageModelPercent = zeros(height(predictor_counts),1); end
predictor_counts.Avg_Abs_tStat_Error = predictor_counts.Sum_Abs_tStat_Error ./ predictor_counts.ErrorModelCount;
predictor_counts.Avg_Abs_tStat_Leakage = predictor_counts.Sum_Abs_tStat_Leakage ./ predictor_counts.LeakageModelCount;
predictor_counts.Avg_Abs_tStat_Error(isnan(predictor_counts.Avg_Abs_tStat_Error)) = 0;
predictor_counts.Avg_Abs_tStat_Leakage(isnan(predictor_counts.Avg_Abs_tStat_Leakage)) = 0;

% --- 3. Display Final Report (Always complete) ---
fprintf('\n===================================================================\n');
fprintf('           OVERALL PARAMETER SENSITIVITY REPORT\n');
fprintf('===================================================================\n');
fprintf('Significance is defined as a parameter remaining in the final\n');
fprintf('model after backward elimination (p <= %.2f).\n\n', P_VALUE_THRESHOLD);
sorted_error = sortrows(predictor_counts, 'ErrorModelCount', 'descend');
fprintf('--- Significance in PREDICTING ERROR SCORE ---\n');
fprintf('%-22s | %-16s | %s\n', 'Parameter', 'Significant In...', 'Avg Strength');
fprintf('%-22s | %-16s | %s\n', '----------------------', '----------------', '--------------');
for i = 1:height(sorted_error)
    row = sorted_error(i,:);
    fprintf('%-22s | %2d / %2d models (%3.0f%%) | Avg |tStat| = %5.2f\n', ...
        strrep(row.Parameter{1}, '_', ' '), row.ErrorModelCount, num_error_models, row.ErrorModelPercent, row.Avg_Abs_tStat_Error);
end
sorted_leakage = sortrows(predictor_counts, 'LeakageModelCount', 'descend');
fprintf('\n--- Significance in PREDICTING LEAKAGE SCORE ---\n');
fprintf('%-22s | %-16s | %s\n', 'Parameter', 'Significant In...', 'Avg Strength');
fprintf('%-22s | %-16s | %s\n', '----------------------', '----------------', '--------------');
for i = 1:height(sorted_leakage)
    row = sorted_leakage(i,:);
    fprintf('%-22s | %2d / %2d models (%3.0f%%) | Avg |tStat| = %5.2f\n', ...
        strrep(row.Parameter{1}, '_', ' '), row.LeakageModelCount, num_leakage_models, row.LeakageModelPercent, row.Avg_Abs_tStat_Leakage);
end
fprintf('\n');

% --- 4. Generate Summary Plot ---
plot_data = sorted_error; 
if HIDE_ZERO_SIGNIFICANCE_PARAMS_IN_PLOT
    rows_to_keep = (plot_data.ErrorModelCount > 0) | (plot_data.LeakageModelCount > 0);
    plot_data = plot_data(rows_to_keep, :);
    fprintf('NOTE: Plot is filtered to show only the %d parameters that were significant in at least one model.\n\n', height(plot_data));
end

figure('Name', 'Overall Parameter Significance Summary', 'Position', [200, 200, 1200, 600]);
t = tiledlayout(1, 2);
num_plot_params = height(plot_data);

% Subplot 1: Frequency of Significance
ax1 = nexttile;
bar_data_freq = [plot_data.ErrorModelPercent, plot_data.LeakageModelPercent];
bar(ax1, bar_data_freq, 'grouped');
% --- FIXED: Force all ticks and labels to be displayed ---
ax1.XTick = 1:num_plot_params;
set(ax1, 'XTickLabel', strrep(plot_data.Parameter, '_', ' '));
xtickangle(ax1, 45);
title(ax1, 'Frequency of Significance', 'FontSize', 14);
ylabel(ax1, 'Significant in X% of Models');
legend(ax1, 'Error Score Models', 'Leakage Score Models', 'Location', 'northeast');
grid(ax1, 'on');

% Subplot 2: Average Strength of Effect
ax2 = nexttile;
bar_data_strength = [plot_data.Avg_Abs_tStat_Error, plot_data.Avg_Abs_tStat_Leakage];
bar(ax2, bar_data_strength, 'grouped');
% --- FIXED: Force all ticks and labels to be displayed ---
ax2.XTick = 1:num_plot_params;
set(ax2, 'XTickLabel', strrep(plot_data.Parameter, '_', ' '));
xtickangle(ax2, 45);
title(ax2, 'Average Strength of Effect (when significant)', 'FontSize', 14);
ylabel(ax2, 'Average |t-statistic|');
legend(ax2, 'Error Score Models', 'Leakage Score Models', 'Location', 'northeast');
grid(ax2, 'on');

sgtitle('Overall Parameter Significance Summary', 'FontSize', 16, 'FontWeight', 'bold');