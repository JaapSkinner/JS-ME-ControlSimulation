% =========================================================================
% VISUALIZE PARAMETER DISTRIBUTIONS
% =========================================================================
% This script runs the sampleParameters function in a loop and plots
% histograms of the resulting distributions for each varied parameter
% to verify the sampling process.
%
clear; clc; close all;

% --- CONFIG ---
N_samples = 1000;
N_motors  = 8;

% Load or define nominal structs
run('ParameterEstimationBaseOL.m'); % Use OL version for full Uav struct
Motor_nom = Motor;
Uav_nom   = Uav;
Uav_nom.COM = [0 0 0];

% --- MODIFIED: Updated for 18 parameters ---
variationPercent = 5 * ones(18,1);  % 5% variation everywhere

% --- Storage ---
features = cell(N_samples, 1);

% --- Sampling Loop ---
fprintf('Generating %d parameter samples...\n', N_samples);
for i = 1:N_samples
    [~, ~, f] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], N_motors);
    features{i} = f;
end
fprintf('Sampling complete.\n');

% Convert to struct array for easier processing
F = [features{:}];

% --- MODIFIED: Added C_TAU to the list ---
paramList = {
    'K_V', true;        % perMotor (std_across_motors + mean_deviation)
    'K_E', true;
    'C_TAU', true;      % <-- ADDED
    'B', true;
    'Volt_offset', true;
    'volt_slope', true;
    'R', true;
    'I_0', true;
    'D_UAV', false;
    'D_PROP', true;
    'M', false;         % scalar only
    'I', false;         % vector per_axis_deviation
    'RHO_AIR', false;
    'R_PROP', false;
    'A_UAV', false;
    'A_PROP', false;
    'ZETA', false;
    'COM', false;
};

% --- Plotting ---
figure('Position', [100 100 1200 900]);
% --- MODIFIED: Increased tile count to accommodate new plots ---
t = tiledlayout(11, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
plotIdx = 1;

for k = 1:size(paramList,1)
    field = paramList{k,1};
    isPerMotor = paramList{k,2};
    
    if isPerMotor
        % Plot std_across_motors
        nexttile(plotIdx);
        vals = arrayfun(@(f) f.(field).std_across_motors, F);
        histogram(vals, 50); % Use more bins for better visualization
        title([field ' std across motors']);
        xlabel('Std Dev'); ylabel('Count');
        plotIdx = plotIdx + 1;
        
        % Plot mean deviation
        nexttile(plotIdx);
        vals = arrayfun(@(f) f.(field).mean_deviation, F);
        histogram(vals, 50);
        title([field ' mean deviation']);
        xlabel('\Delta'); ylabel('Count');
        plotIdx = plotIdx + 1;
    else
        % Check if vector (per_axis_deviation)
        sample = F(1).(field);
        if isfield(sample, 'per_axis_deviation')
            % Determine if it's 3x1 (COM) or 3x3 (Inertia)
            if numel(sample.per_axis_deviation) == 3
                num_axes = 3;
                vals_mat = reshape([vertcat(F.(field)).per_axis_deviation], num_axes, [])';
            else % Assuming inertia matrix
                num_axes = 3; % We only care about the diagonal
                vals_mat = reshape([vertcat(F.(field)).per_axis_deviation], num_axes, [])';
            end

            for ax = 1:num_axes
                nexttile(plotIdx);
                histogram(vals_mat(:,ax), 50);
                title([field ' axis ' num2str(ax) ' deviation']);
                xlabel('\Delta'); ylabel('Count');
                plotIdx = plotIdx + 1;
            end
        else
            % Scalar only
            vals = arrayfun(@(f) f.(field).mean_deviation, F);
            nexttile(plotIdx);
            histogram(vals, 50);
            title([field ' mean deviation']);
            xlabel('\Delta'); ylabel('Count');
            plotIdx = plotIdx + 1;
        end
    end
end
sgtitle('Monte Carlo Parameter Variation Distributions', 'FontSize', 16, 'FontWeight', 'bold');