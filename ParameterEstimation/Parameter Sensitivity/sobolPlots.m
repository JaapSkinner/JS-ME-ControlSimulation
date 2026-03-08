% =========================================================================
% "PAZAZZ" PLOTS: BUBBLE MATRIX & 3D LANDSCAPE
% =========================================================================
clear; clc; close all;

% --- 1. Load Data ---
[file, path] = uigetfile('*.mat', 'Select the UQLab Sobol Results File');
if isequal(file, 0), return; end
load(fullfile(path, file));

if ~exist('sobol_results', 'var') || ~exist('initial_predictors', 'var')
    error('Incorrect file selected. Missing required variables.');
end

out_dir = uigetdir(path, 'Select Folder to Save the PNG Plots');
if isequal(out_dir, 0), return; end

fprintf('Generating showstopper plots...\n');

nSetpoints = length(sobol_results);
num_vars = length(initial_predictors);

total_order_matrix = zeros(num_vars, nSetpoints);
first_order_matrix = zeros(num_vars, nSetpoints);
setpoint_labels = cell(1, nSetpoints);

for sp = 1:nSetpoints
    if isempty(sobol_results{sp}), continue; end
    total_order_matrix(:, sp) = sobol_results{sp}.SobolTotalOrder(:);
    first_order_matrix(:, sp) = sobol_results{sp}.SobolFirstOrder(:);
    setpoint_labels{sp} = sprintf('SP %d', sp);
end

% Filter out the noise (Keep if Max Total Order > 2%)
keep_idx = max(total_order_matrix, [], 2) > 0.02;

filtered_Total = total_order_matrix(keep_idx, :);
filtered_First = first_order_matrix(keep_idx, :);
filtered_Interaction = filtered_Total - filtered_First; % The Non-linear coupling!
filtered_vars = initial_predictors(keep_idx);
filtered_vars = strrep(filtered_vars, '_', '\_'); % Clean text for plots

% =========================================================================
% PLOT 1: THE INTERACTION BUBBLE MATRIX
% =========================================================================
fig_bubble = figure('Color', 'w', 'Position', [100, 100, 950, 650], 'Visible', 'off');

% Create coordinates for the grid
[X_grid, Y_grid] = meshgrid(1:nSetpoints, 1:length(filtered_vars));

% Bubble Size = Total Order (scaled for visibility)
bubble_sizes = max(filtered_Total(:) * 2000, 1); 
% Bubble Color = Interaction Effect
bubble_colors = filtered_Interaction(:);

% Generate Bubble Chart (Requires MATLAB R2020b or newer)
b = bubblechart(X_grid(:), Y_grid(:), bubble_sizes, bubble_colors, 'MarkerFaceAlpha', 0.80);

% Formatting
colormap(hot); % 'hot' makes high interactions glow bright yellow/white
c = colorbar;
c.Label.String = 'Interaction Magnitude (Total - First Order)';
c.Label.FontSize = 11;
c.Label.FontWeight = 'bold';

% Lock axes and labels
xticks(1:nSetpoints);
xticklabels(setpoint_labels);
yticks(1:length(filtered_vars));
yticklabels(filtered_vars);
xlim([0.5, nSetpoints + 0.5]);
ylim([0.5, length(filtered_vars) + 0.5]);

title('Non-Linear Parameter Interactions Across Maneuvers', 'FontSize', 15);
subtitle('Bubble Size = Total Impact | Color = Dependency on Parameter Coupling', 'FontSize', 11);
grid on; set(gca, 'GridAlpha', 0.15);

bubble_filename = fullfile(out_dir, 'Sobol_Interaction_Bubble.png');
exportgraphics(fig_bubble, bubble_filename, 'Resolution', 300);
close(fig_bubble);
fprintf('  Saved: Sobol_Interaction_Bubble.png\n');

% =========================================================================
% PLOT 2: THE 3D SENSITIVITY LANDSCAPE
% =========================================================================
fig_3d = figure('Color', 'w', 'Position', [150, 150, 1000, 700], 'Visible', 'off');

% Generate 3D Bar
b3 = bar3(filtered_Total);

% Color the bars based on their height (Z-value)
for k = 1:length(b3)
    zdata = b3(k).ZData;
    b3(k).CData = zdata;
    b3(k).FaceColor = 'interp';
end

colormap(parula);
colorbar;

% Formatting
set(gca, 'XTickLabel', setpoint_labels, 'FontSize', 11);
set(gca, 'YTickLabel', filtered_vars, 'FontSize', 11);
zlabel('Total-Order Sensitivity', 'FontSize', 12, 'FontWeight', 'bold');
title('3D Sensitivity Landscape of UAV Errors', 'FontSize', 15);

% Tweak the camera angle for a dramatic, presentation-ready view
view(-45, 35); 
grid on;

bar3_filename = fullfile(out_dir, 'Sobol_3D_Landscape.png');
exportgraphics(fig_3d, bar3_filename, 'Resolution', 300);
close(fig_3d);
fprintf('  Saved: Sobol_3D_Landscape.png\n');

fprintf('\nPazazz successfully added! Plots saved.\n');

