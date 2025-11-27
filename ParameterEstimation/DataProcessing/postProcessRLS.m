%% ANALYZE UKF vs RLS SIMULATION RESULTS
%
% This script loads preprocessed data containing both UKF and RLS estimates.
% It generates comparative plots for:
%   1. Single Run Tracking (Separate plots for UKF and RLS).
%   2. Aggregate Error Quantiles (Combined).
%   3. Statistical Box Plots (Grouped by Axis, Split Force/Torque).
%   4. Performance vs. System Variation Scatter (Combined).
%
clear; clc; close all;

%% --- CONFIGURATION ---
TRIM_SECONDS = 5.0; % Number of seconds to cut from start and end

% Color Palette
c.nom = [0.6350 0.0780 0.1840]; % Red (Nominal)
c.ukf = [0.0000 0.4470 0.7410]; % Blue (UKF)
c.rls = [0.4660 0.6740 0.1880]; % Green (RLS)
c.true = [0 0 0];               % Black (Ground Truth)

%% --- 1. Select Preprocessed Folder ---
fprintf('Step 1: Select the folder containing UKF_RLS_Preprocessed results...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end

preprocessedPath = uigetdir(startPath, 'Select Folder Containing Preprocessed Data');
if isequal(preprocessedPath, 0)
    disp('User selected Cancel. Script terminated.'); 
    return; 
end

[parentFolder, ~] = fileparts(preprocessedPath);
plotOutputPath = fullfile(parentFolder, 'UKF_RLS_Analysis_Plots'); 

if ~isfolder(plotOutputPath)
    mkdir(plotOutputPath);
    fprintf('Created output folder: %s\n', plotOutputPath);
end

resultFiles = dir(fullfile(preprocessedPath, 'preprocessed_estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0
    error('No preprocessed files found in: %s', preprocessedPath);
end

fprintf('Found %d files. Analyzing...\n', numFiles);

%% --- 2. Initialize Aggregation Structures ---
% We will store RMSEs for boxplots and scatter plots
% Structure: [NumFiles x 1]
aggData = struct('filename', {}, 'features', {}, ...
                 'rmse_dof_nom', {}, 'rmse_dof_ukf', {}, 'rmse_dof_rls', {}, ...
                 'rmse_total_nom', {}, 'rmse_total_ukf', {}, 'rmse_total_rls', {}, ...
                 'time_aligned', {}, ...
                 'error_nom', {}, 'error_ukf', {}, 'error_rls', {});

dof_labels = {'F_x', 'F_y', 'F_z', '\tau_x', '\tau_y', '\tau_z'};

%% --- 3. Process Files ---
validCount = 0;

for i = 1:numFiles
    fName = resultFiles(i).name;
    fullPath = fullfile(preprocessedPath, fName);
    
    try
        S = load(fullPath);
        
        % Check required fields
        if ~isfield(S, 'Predicted_Wrench_UKF') || ~isfield(S, 'Predicted_Wrench_RLS') || ...
           ~isfield(S, 'Real_Wrench')
            fprintf('Skipping %s (Missing data fields)\n', fName);
            continue;
        end
        
        % --- Time Synchronization & Trimming ---
        % Find common start/end times across all signals
        t_start_raw = max([S.Real_Wrench.Time(1), S.Predicted_Wrench_UKF.Time(1), S.Predicted_Wrench_RLS.Time(1)]);
        t_end_raw   = min([S.Real_Wrench.Time(end), S.Predicted_Wrench_UKF.Time(end), S.Predicted_Wrench_RLS.Time(end)]);
        
        t_trim_start = t_start_raw + TRIM_SECONDS;
        t_trim_end   = t_end_raw - TRIM_SECONDS;
        
        if t_trim_end <= t_trim_start
            continue; 
        end
        
        % Create common time vector based on Real_Wrench sampling
        raw_time = S.Real_Wrench.Time;
        time_common = raw_time(raw_time >= t_trim_start & raw_time <= t_trim_end);
        
        if length(time_common) < 10, continue; end
        
        % Resample all data to common time
        w_real = resample(S.Real_Wrench, time_common).Data;
        w_nom  = resample(S.Nom_Predicted_Wrench, time_common).Data;
        w_ukf  = resample(S.Predicted_Wrench_UKF, time_common).Data;
        w_rls  = resample(S.Predicted_Wrench_RLS, time_common).Data;
        
        % --- Error Calculation ---
        e_nom = w_real - w_nom;
        e_ukf = w_real - w_ukf;
        e_rls = w_real - w_rls;
        
        % --- Metrics ---
        % 1. RMSE per DOF (for Boxplots)
        rmse_dof_nom = rms(e_nom, 1);
        rmse_dof_ukf = rms(e_ukf, 1);
        rmse_dof_rls = rms(e_rls, 1);
        
        % 2. Total RMSE (Norm of all errors) for Scatter plot
        % Flatten the error matrix to get a single scalar score for the run
        rmse_total_nom = rms(vecnorm(e_nom, 2, 2));
        rmse_total_ukf = rms(vecnorm(e_ukf, 2, 2));
        rmse_total_rls = rms(vecnorm(e_rls, 2, 2));
        
        % --- Store ---
        validCount = validCount + 1;
        aggData(validCount).filename = fName;
        aggData(validCount).features = S.features_i;
        aggData(validCount).time_aligned = time_common - time_common(1); % Relative time 0 start
        
        aggData(validCount).rmse_dof_nom = rmse_dof_nom;
        aggData(validCount).rmse_dof_ukf = rmse_dof_ukf;
        aggData(validCount).rmse_dof_rls = rmse_dof_rls;
        
        aggData(validCount).rmse_total_nom = rmse_total_nom;
        aggData(validCount).rmse_total_ukf = rmse_total_ukf;
        aggData(validCount).rmse_total_rls = rmse_total_rls;
        
        aggData(validCount).error_nom = e_nom;
        aggData(validCount).error_ukf = e_ukf;
        aggData(validCount).error_rls = e_rls;
        
        % Save raw data for the FIRST valid file to use in Figure 1
        if validCount == 1
            exampleRun.time = time_common;
            exampleRun.real = w_real;
            exampleRun.nom  = w_nom;
            exampleRun.ukf  = w_ukf;
            exampleRun.rls  = w_rls;
        end
        
    catch ME
        fprintf('Error processing %s: %s\n', fName, ME.message);
    end
end
aggData = aggData(1:validCount);
fprintf('Successfully aggregated %d runs.\n', validCount);

%% --- 4. PREPARE PLOTTING DATA ---

% Matrix forms for boxplots: [N_files x 6]
all_rmse_nom = vertcat(aggData.rmse_dof_nom);
all_rmse_ukf = vertcat(aggData.rmse_dof_ukf);
all_rmse_rls = vertcat(aggData.rmse_dof_rls);

% Vectors for scatter plot: [N_files x 1]
vec_rmse_total_nom = [aggData.rmse_total_nom]';
vec_rmse_total_ukf = [aggData.rmse_total_ukf]';
vec_rmse_total_rls = [aggData.rmse_total_rls]';

% Feature Variation Calculation (Z-Score Sum) for Scatter Plot
all_features = [aggData.features];
field_names = fields(all_features);
feature_vals = zeros(validCount, 0);

% Extract scalar values from feature structure
for k = 1:length(field_names)
    fname = field_names{k};
    if isstruct(all_features(1).(fname))
        % Handle nested 'mean_deviation' if exists
        if isfield(all_features(1).(fname), 'mean_deviation')
            vals = [all_features.(fname)];
            feature_vals(:, end+1) = [vals.mean_deviation]';
        end
    elseif isnumeric(all_features(1).(fname))
        feature_vals(:, end+1) = [all_features.(fname)]';
    end
end
% Normalized variation score
z_feats = zscore(feature_vals);
variation_score = sum(abs(z_feats), 2);


%% --- FIGURE 1: Single Run Tracking (Separated) ---
% We create one figure for UKF and one for RLS to support report sections.

if exist('exampleRun', 'var')
    % --- Fig 1a: UKF vs Nominal vs True ---
    f1a = figure('Name', 'Single Run: UKF Tracking', 'Position', [100 100 1000 600]);
    t1a = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    for k = 1:6
        nexttile(t1a);
        plot(exampleRun.time, exampleRun.real(:, k), 'k-', 'LineWidth', 1.5); hold on;
        plot(exampleRun.time, exampleRun.nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.2);
        plot(exampleRun.time, exampleRun.ukf(:, k), '-', 'Color', c.ukf, 'LineWidth', 1.2);
        title(dof_labels{k}); grid on; xlim([exampleRun.time(1) exampleRun.time(end)]);
        if k==1, legend('True', 'Nominal', 'UKF', 'Location', 'best'); end
    end
    title(t1a, 'Single Run Tracking: UKF Estimate');
    saveas(f1a, fullfile(plotOutputPath, 'Fig1a_SingleRun_UKF.png'));
    
    % --- Fig 1b: RLS vs Nominal vs True ---
    f1b = figure('Name', 'Single Run: RLS Tracking', 'Position', [150 150 1000 600]);
    t1b = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    for k = 1:6
        nexttile(t1b);
        plot(exampleRun.time, exampleRun.real(:, k), 'k-', 'LineWidth', 1.5); hold on;
        plot(exampleRun.time, exampleRun.nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.2);
        plot(exampleRun.time, exampleRun.rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.2);
        title(dof_labels{k}); grid on; xlim([exampleRun.time(1) exampleRun.time(end)]);
        if k==1, legend('True', 'Nominal', 'RLS', 'Location', 'best'); end
    end
    title(t1b, 'Single Run Tracking: RLS Estimate');
    saveas(f1b, fullfile(plotOutputPath, 'Fig1b_SingleRun_RLS.png'));
end

%% --- FIGURE 2: Aggregate Error Quantiles (Combined) ---
% Using Median lines for all 3 to compare behavior over time

% 1. Interpolate all errors to a master time vector
all_time_vecs = {aggData.time_aligned};
durations = cellfun(@(t) t(end), all_time_vecs);
master_time = 0:0.1:min(durations);

if isempty(master_time), master_time = 0:0.1:10; end 
nSteps = length(master_time);

interp_err_nom = zeros(nSteps, 6, validCount);
interp_err_ukf = zeros(nSteps, 6, validCount);
interp_err_rls = zeros(nSteps, 6, validCount);

for i = 1:validCount
    t = aggData(i).time_aligned;
    interp_err_nom(:,:,i) = interp1(t, aggData(i).error_nom, master_time, 'linear', 0);
    interp_err_ukf(:,:,i) = interp1(t, aggData(i).error_ukf, master_time, 'linear', 0);
    interp_err_rls(:,:,i) = interp1(t, aggData(i).error_rls, master_time, 'linear', 0);
end

% 2. Calculate Medians
med_nom = median(interp_err_nom, 3);
med_ukf = median(interp_err_ukf, 3);
med_rls = median(interp_err_rls, 3);

f2 = figure('Name', 'Aggregate Error Dynamics', 'Position', [200 200 1000 600]);
t2 = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

for k = 1:6
    nexttile(t2);
    % Plot 0 line
    yline(0, 'k-', 'Alpha', 0.3); hold on;
    
    % Plot Medians (Thinner lines as requested)
    plot(master_time, med_nom(:, k), '--', 'Color', c.nom, 'LineWidth', 1.0);
    plot(master_time, med_ukf(:, k), '-', 'Color', c.ukf, 'LineWidth', 1.5);
    plot(master_time, med_rls(:, k), '-', 'Color', c.rls, 'LineWidth', 1.5);
    
    title(dof_labels{k}); grid on;
    xlim([master_time(1) master_time(end)]);
    if k==1, legend('Zero Ref', 'Nominal Median', 'UKF Median', 'RLS Median', 'Location', 'best'); end
    if k>3, xlabel('Time (s)'); end
    ylabel('Error');
end
title(t2, 'Aggregate Error Dynamics (Median across all runs)');
saveas(f2, fullfile(plotOutputPath, 'Fig2_ErrorDynamics_Combined.png'));


%% --- FIGURE 3: Comparative Box Plots (Grouped by Axis, Split Force/Torque) ---
% Desired Layout: 
% Subplot 1: Forces (Fx, Fy, Fz). Grouped: [Nom, UKF, RLS] per axis.
% Subplot 2: Torques (Tx, Ty, Tz). Grouped: [Nom, UKF, RLS] per axis.

f3 = figure('Name', 'Statistical Performance', 'Position', [100 100 1200 500]);
t3 = tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% -- Helper for Boxplot Data Construction --
% We construct a matrix where columns are: 
% [Fx_Nom, Fx_UKF, Fx_RLS, Gap, Fy_Nom, Fy_UKF, Fy_RLS, Gap, ...]
% The Gap column (NaNs) creates visual separation between axes.

% --- Subplot 1: FORCES (Cols 1, 2, 3) ---
nexttile(t3);
nan_col = nan(validCount, 1);
data_forces = [ ...
    all_rmse_nom(:,1), all_rmse_ukf(:,1), all_rmse_rls(:,1), nan_col, ... % Fx
    all_rmse_nom(:,2), all_rmse_ukf(:,2), all_rmse_rls(:,2), nan_col, ... % Fy
    all_rmse_nom(:,3), all_rmse_ukf(:,3), all_rmse_rls(:,3) ];            % Fz

% Positions for the boxes to ensure tight grouping
positions_F = [1 1.25 1.5,  2.5 2.75 3.0,  4.0 4.25 4.5];
group_labels_F = {'Nom','UKF','RLS', 'Nom','UKF','RLS', 'Nom','UKF','RLS'};

% Remove the NaN columns for the plot command, keep positions
plot_data_F = data_forces(:, [1:3, 5:7, 9:11]);

hB1 = boxplot(plot_data_F, 'Positions', positions_F, 'Widths', 0.2, 'Colors', 'k');
hold on;

% Color the boxes manually
h = findobj(gca,'Tag','Box');
% Note: boxplot creates handles in reverse order (last column first)
boxes_rls = h(1:3:end); 
boxes_ukf = h(2:3:end);
boxes_nom = h(3:3:end);

set(boxes_rls, 'Color', c.rls, 'LineWidth', 1.5);
set(boxes_ukf, 'Color', c.ukf, 'LineWidth', 1.5);
set(boxes_nom, 'Color', c.nom, 'LineWidth', 1.5);

set(gca, 'XTick', [1.25, 2.75, 4.25], 'XTickLabel', {'F_x', 'F_y', 'F_z'});
ylabel('RMSE (N)'); grid on;
title('Force Estimation Error');

% Dummy lines for legend
L1 = plot(nan, nan, 'Color', c.nom, 'LineWidth', 2);
L2 = plot(nan, nan, 'Color', c.ukf, 'LineWidth', 2);
L3 = plot(nan, nan, 'Color', c.rls, 'LineWidth', 2);
legend([L1, L2, L3], 'Nominal', 'UKF', 'RLS', 'Location', 'northwest');


% --- Subplot 2: TORQUES (Cols 4, 5, 6) ---
nexttile(t3);
data_torques = [ ...
    all_rmse_nom(:,4), all_rmse_ukf(:,4), all_rmse_rls(:,4), nan_col, ... % Tx
    all_rmse_nom(:,5), all_rmse_ukf(:,5), all_rmse_rls(:,5), nan_col, ... % Ty
    all_rmse_nom(:,6), all_rmse_ukf(:,6), all_rmse_rls(:,6) ];            % Tz

plot_data_T = data_torques(:, [1:3, 5:7, 9:11]);

hB2 = boxplot(plot_data_T, 'Positions', positions_F, 'Widths', 0.2, 'Colors', 'k');
hold on;

h2 = findobj(gca,'Tag','Box');
% Reverse order again
set(h2(1:3:end), 'Color', c.rls, 'LineWidth', 1.5);
set(h2(2:3:end), 'Color', c.ukf, 'LineWidth', 1.5);
set(h2(3:3:end), 'Color', c.nom, 'LineWidth', 1.5);

set(gca, 'XTick', [1.25, 2.75, 4.25], 'XTickLabel', {'\tau_x', '\tau_y', '\tau_z'});
ylabel('RMSE (Nm)'); grid on;
title('Torque Estimation Error');

saveas(f3, fullfile(plotOutputPath, 'Fig3_Comparative_Boxplots.png'));


%% --- FIGURE 4: Robustness Scatter (Variation vs RMSE) ---
% Combined plot with 3 data points per file (Nom, UKF, RLS)

f4 = figure('Name', 'Robustness Analysis', 'Position', [150 150 800 600]);

% 1. Plot Nominal
scatter(variation_score, vec_rmse_total_nom, 40, 's', 'MarkerEdgeColor', c.nom, 'LineWidth', 1.5); hold on;

% 2. Plot UKF
scatter(variation_score, vec_rmse_total_ukf, 40, 'o', 'MarkerEdgeColor', c.ukf, 'LineWidth', 1.5);

% 3. Plot RLS
scatter(variation_score, vec_rmse_total_rls, 40, 'd', 'MarkerEdgeColor', c.rls, 'LineWidth', 1.5);

% Add trend lines
p_nom = polyfit(variation_score, vec_rmse_total_nom, 1);
p_ukf = polyfit(variation_score, vec_rmse_total_ukf, 1);
p_rls = polyfit(variation_score, vec_rmse_total_rls, 1);

x_fit = linspace(min(variation_score), max(variation_score), 100);

plot(x_fit, polyval(p_nom, x_fit), '--', 'Color', c.nom, 'LineWidth', 1);
plot(x_fit, polyval(p_ukf, x_fit), '-', 'Color', c.ukf, 'LineWidth', 1.5);
plot(x_fit, polyval(p_rls, x_fit), '-', 'Color', c.rls, 'LineWidth', 1.5);

xlabel('System Variation Index (Sum of Feature Z-Scores)');
ylabel('Total RMSE (Norm)');
title('Estimator Robustness to System Variations');
grid on;
legend('Nominal', 'UKF', 'RLS', 'Location', 'best');

saveas(f4, fullfile(plotOutputPath, 'Fig4_Robustness_Scatter.png'));

fprintf('Analysis Complete. Figures saved to: %s\n', plotOutputPath);