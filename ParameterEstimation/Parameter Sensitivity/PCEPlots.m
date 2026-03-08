% =========================================================================
% SCRIPT 3: SPARSE PCE VALIDATION PLOT (Fixes the Curse of Dimensionality)
% =========================================================================
clear; clc; close all;

% --- 1. Load the Sobol Results ---
[sobol_file, sobol_path] = uigetfile('*.mat', '1. Select your UQLab Sobol Results File');
if isequal(sobol_file, 0), return; end
load(fullfile(sobol_path, sobol_file), 'initial_predictors');

if ~exist('initial_predictors', 'var')
    error('Could not find ''initial_predictors'' in the Sobol file.');
end

% --- 2. Load the Raw Data (mega_table) ---
[raw_file, raw_path] = uigetfile('*.mat', '2. Select the .mat file containing mega_table');
if isequal(raw_file, 0), return; end
data_loaded = load(fullfile(raw_path, raw_file));

if isfield(data_loaded, 'mega_table')
    mega_table = data_loaded.mega_table;
else
    error('The selected file does not contain ''mega_table''.');
end

fprintf('Building Advanced Sparse PCE (LARS) for Setpoint 1...\n');
sp = 1; 

% --- 3. Extract Data ---
subset_table = mega_table(mega_table.SetpointID == sp, :);
X = subset_table{:, initial_predictors};
Y_true = subset_table.ErrorScore;

valid_idx = ~any(isnan(X), 2) & ~isnan(Y_true);
X = X(valid_idx, :); 
Y_true = Y_true(valid_idx);

% --- 4. Build the LARS PCE Model ---
clear InputOpts MetaOpts;
for i = 1:size(X, 2)
    InputOpts.Marginals(i).Type = 'Uniform';
    InputOpts.Marginals(i).Parameters = [min(X(:,i)), max(X(:,i))];
end
myInput = uq_createInput(InputOpts, '-private');

MetaOpts.Type = 'Metamodel';
MetaOpts.MetaType = 'PCE';
MetaOpts.Input = myInput;
MetaOpts.ExpDesign.X = X;
MetaOpts.ExpDesign.Y = Y_true;

% -- THE MAGIC FIX: SPARSE PCE SETTINGS --
MetaOpts.Method = 'LARS';             % Use Least Angle Regression for sparsity
MetaOpts.Degree = 1:3;                % Lower degree to prevent matrix crashing
MetaOpts.TruncOptions.qNorm = 0.75;   % Hyperbolic truncation (favors main/2-way effects)
MetaOpts.Display = 'quiet';

myPCE = uq_createModel(MetaOpts, '-private');

% --- 5. Evaluate the Model ---
Y_pred = uq_evalModel(myPCE, X);

% --- 6. Calculate R-squared ---
SS_res = sum((Y_true - Y_pred).^2);
SS_tot = sum((Y_true - mean(Y_true)).^2);
R_squared = 1 - (SS_res / SS_tot);

% --- 7. Generate the Plot ---
fig_scatter = figure('Color', 'w', 'Position', [200, 200, 700, 600]);
scatter(Y_true, Y_pred, 30, [0.2 0.4 0.7], 'filled', 'MarkerFaceAlpha', 0.5);
hold on;

min_val = min([Y_true; Y_pred]);
max_val = max([Y_true; Y_pred]);
plot([min_val, max_val], [min_val, max_val], 'k--', 'LineWidth', 2);

xlabel('True Simulation Error', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Sparse PCE Predicted Error', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('Surrogate Model Validation (Setpoint %d)', sp), 'FontSize', 15);
subtitle('Sparse Polynomial Chaos Expansion (LARS) Accuracy', 'FontSize', 11);
legend({'Monte Carlo Samples', 'Perfect Fit (y = x)'}, 'Location', 'northwest', 'FontSize', 11);
grid on; set(gca, 'GridAlpha', 0.15);
axis equal; 
xlim([min_val max_val]); ylim([min_val max_val]);

txt = sprintf('R^2 = %.4f', R_squared);
text(max_val*0.65, min_val + (max_val-min_val)*0.1, txt, ...
    'FontSize', 14, 'BackgroundColor', 'w', 'EdgeColor', 'k');

% --- 8. Save the Plot ---
out_dir = uigetdir(raw_path, 'Select Folder to Save the Scatter Plot');
if ~isequal(out_dir, 0)
    out_name = fullfile(out_dir, sprintf('PCE_Validation_Scatter_SP%d.png', sp));
    exportgraphics(fig_scatter, out_name, 'Resolution', 300);
    fprintf('  Saved: %s\n', out_name);
end

fprintf('\nSuccess! R-squared achieved with LARS: %.4f\n', R_squared);