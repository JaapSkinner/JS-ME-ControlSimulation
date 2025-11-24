%% ANALYZE PREPROCESSED SIMULATION DATA (UKF Version) - FINAL CLEAN + TRIMMED
%
% This script loads all files from a '..._Preprocessed' folder,
% aggregates data, calculates wrench prediction error metrics, and plots
% them against the simulation's ground-truth feature variations.
%
% COMPARISON: Estimated (UKF) vs. Nominal (Baseline Guess).
% DATA PROCESSING: Trims the first and last 5 seconds of data.
%
% Modifications:
% - TRIM: Removes 5s from start/end before metrics/plotting.
% - Outlier Detection: Identifies and lists high-error runs (IQR method).
% - Fig 4a: Visually highlights outliers with labels.
% - Fig 4b: Absolute Correlation for Estimated only.
%
clear; clc; close all;
%% --- CONFIGURATION ---
TRIM_SECONDS = 5.0; % Number of seconds to cut from start and end

%% --- 1. Select Preprocessed Folder & Create Plot Output Folder ---
fprintf('Step 1: Select the folder containing preprocessed result files (e.g., UKF_Preprocessed)...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end
preprocessedPath = uigetdir(startPath, 'Select the Folder Containing Preprocessed Results');
if isequal(preprocessedPath, 0), disp('User selected Cancel. Script terminated.'); return; end
[parentFolder, ~] = fileparts(preprocessedPath);

% Output folder name updated to indicate Trimmed status
plotOutputPath = fullfile(parentFolder, 'UKF_Analysis_Plots_Trimmed'); 
if ~isfolder(plotOutputPath)
    fprintf('Creating plot output directory: %s\n', plotOutputPath);
    mkdir(plotOutputPath);
else
    fprintf('Plot output directory already exists: %s\n', plotOutputPath);
end

resultFiles = dir(fullfile(preprocessedPath, 'preprocessed_estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No preprocessed_estimation_*.mat files found in: %s', preprocessedPath); end
fprintf('Found %d preprocessed files. Aggregating results (Trimming %.1fs)...\n', numFiles, TRIM_SECONDS);

%% --- 2. Initialize Results Aggregator & Plot Settings ---
results = struct('filename', {}, ...
                 'rmse_per_dof_est', {}, ...         
                 'rmse_overall_force_N_est', {}, ... 
                 'rmse_overall_torque_Nm_est', {}, ...
                 'wrench_error_est', {}, ...        
                 'rmse_per_dof_nom', {}, ...         
                 'rmse_overall_force_N_nom', {}, ... 
                 'rmse_overall_torque_Nm_nom', {}, ...
                 'wrench_error_nom', {}, ...        
                 'time', {}, ...                 
                 'features', {});               
results(numFiles).filename = [];

% Plotting Labels and Colors
dof_labels = {'F_x (N)', 'F_y (N)', 'F_z (N)', '\tau_x (N·m)', '\tau_y (N·m)', '\tau_z (N·m)'};
dof_labels_short = {'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'};
c.true = [0 0 0];                       % Black
c.est = [0.0 0.4470 0.7410];            % Blue (UKF)
c.nom = [0.8500 0.3250 0.0980];         % Red (Nominal)
c.outlier = [1 0 0];                    % Red for outliers

%% --- 3. Process Each File and Aggregate Results ---
processedCount = 0;
for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(preprocessedPath, currentFileName);
    
    try
        S = load(fullFilePath);
        % Check for required fields
        if ~isfield(S, 'Real_Wrench') || ~isfield(S, 'Predicted_Wrench') || ...
           ~isfield(S, 'Nom_Predicted_Wrench') || ~isfield(S, 'features_i')
            continue;
        end
        
        % --- Align Time & Trim ---
        % 1. Find intersection of raw times
        t_start_raw = max(S.Real_Wrench.Time(1), S.Predicted_Wrench.Time(1));
        t_end_raw = min(S.Real_Wrench.Time(end), S.Predicted_Wrench.Time(end));
        
        % 2. Define Trimmed Window
        t_trim_start = t_start_raw + TRIM_SECONDS;
        t_trim_end = t_end_raw - TRIM_SECONDS;
        
        % 3. Check if enough data remains
        if t_trim_end <= t_trim_start
            warning('File %s too short for %.1fs trim. Skipping.', currentFileName, TRIM_SECONDS);
            continue;
        end
        
        % 4. Create Common Time Vector (using Real Wrench sampling)
        % Only keep indices within the trimmed window
        raw_time_vec = S.Real_Wrench.Time;
        time_common = raw_time_vec(raw_time_vec >= t_trim_start & raw_time_vec <= t_trim_end);
        
        if length(time_common) < 10, continue; end
        
        % --- Resample Data to Trimmed Time ---
        real_wrench_data = resample(S.Real_Wrench, time_common).Data;
        est_pred_wrench_data = resample(S.Predicted_Wrench, time_common).Data;
        nom_pred_wrench_data = resample(S.Nom_Predicted_Wrench, time_common).Data;
        
        % --- Calculate Error ---
        wrenchError_est = real_wrench_data - est_pred_wrench_data; 
        wrenchError_nom = real_wrench_data - nom_pred_wrench_data; 
        
        % --- Metrics (Estimated - UKF) ---
        rmse_per_dof_est = rms(wrenchError_est, 1); 
        forceErrorMagnitude_est = vecnorm(wrenchError_est(:, 1:3), 2, 2);
        torqueErrorMagnitude_est = vecnorm(wrenchError_est(:, 4:6), 2, 2);
        
        % --- Metrics (Nominal) ---
        rmse_per_dof_nom = rms(wrenchError_nom, 1); 
        forceErrorMagnitude_nom = vecnorm(wrenchError_nom(:, 1:3), 2, 2);
        torqueErrorMagnitude_nom = vecnorm(wrenchError_nom(:, 4:6), 2, 2);
        
        % --- Store ---
        processedCount = processedCount + 1;
        results(processedCount).filename = currentFileName;
        results(processedCount).time = time_common;
        results(processedCount).features = S.features_i;
        
        results(processedCount).rmse_per_dof_est = rmse_per_dof_est;
        results(processedCount).rmse_overall_force_N_est = rms(forceErrorMagnitude_est);
        results(processedCount).rmse_overall_torque_Nm_est = rms(torqueErrorMagnitude_est);
        results(processedCount).wrench_error_est = wrenchError_est;
        
        results(processedCount).rmse_per_dof_nom = rmse_per_dof_nom;
        results(processedCount).rmse_overall_force_N_nom = rms(forceErrorMagnitude_nom);
        results(processedCount).rmse_overall_torque_Nm_nom = rms(torqueErrorMagnitude_nom);
        results(processedCount).wrench_error_nom = wrenchError_nom;
        
    catch
        % Silent skip
    end
end
results = results(1:processedCount);
numFiles = processedCount;
fprintf('Aggregation Complete. Processed %d files.\n', processedCount);

%% --- 4. Plotting Data Preparation ---
fprintf('Preparing aggregated data...\n');
all_time_cell = {results.time};

% Note: Times are now trimmed relative to their own start, so we align by index or relative time 
t_durations = cellfun(@(t) t(end) - t(1), all_time_cell);
min_duration = min(t_durations);

% Create a relative master time from 0 to min_duration
t_step = median(cellfun(@(t) median(diff(t)), all_time_cell));
master_time = (0:t_step:min_duration)'; 
nTimeSteps = length(master_time);

all_errors_interp_est = zeros(nTimeSteps, 6, numFiles);
all_errors_interp_nom = zeros(nTimeSteps, 6, numFiles);

for i = 1:numFiles
    % Shift this run's time to start at 0 for alignment
    t_shifted = results(i).time - results(i).time(1);
    
    all_errors_interp_est(:,:,i) = interp1(t_shifted, results(i).wrench_error_est, master_time, 'linear', NaN);
    all_errors_interp_nom(:,:,i) = interp1(t_shifted, results(i).wrench_error_nom, master_time, 'linear', NaN);
end

all_rmse_per_dof_est = vertcat(results.rmse_per_dof_est);
all_rmse_per_dof_nom = vertcat(results.rmse_per_dof_nom);
all_features = [results.features];

%% --- 5. OUTLIER ANALYSIS ---
% We identify outliers based on the Total RMSE of the Estimation.
% Method: Interquartile Range (IQR). Anything > Q3 + 1.5*IQR is an outlier.
fprintf('\n-------------------------------------------------\n');
fprintf('Analyzing for Outliers...\n');

% Calculate total RMSE for every run
total_rmse_est = vecnorm(all_rmse_per_dof_est, 2, 2); % [N x 1]

% Calculate Statistics
q25 = prctile(total_rmse_est, 25);
q75 = prctile(total_rmse_est, 75);
iqr_val = q75 - q25;
upper_fence = q75 + 1.5 * iqr_val;

% Find Outliers
outlier_indices = find(total_rmse_est > upper_fence);

if isempty(outlier_indices)
    fprintf('No significant outliers detected (using 1.5*IQR rule).\n');
else
    fprintf('** POTENTIAL OUTLIERS DETECTED: %d runs **\n', length(outlier_indices));
    fprintf('Threshold RMSE > %.4f\n', upper_fence);
    fprintf('-------------------------------------------------\n');
    fprintf('%-6s | %-10s | %s\n', 'Run ID', 'RMSE', 'Filename');
    fprintf('-------------------------------------------------\n');
    for k = 1:length(outlier_indices)
        idx = outlier_indices(k);
        fprintf('%-6d | %-10.4f | %s\n', idx, total_rmse_est(idx), results(idx).filename);
    end
    fprintf('-------------------------------------------------\n');
    fprintf('These runs are marked with red Xs in Figure 4a.\n');
end
fprintf('\n');

%% --- 6. Plot Generation ---
fprintf('Generating plots...\n');

% --- A. Illustrative Single-Run Example (Fig 1a, 1b) ---
% NOTE: Must apply the same TRIM logic here!
try
    i_run = 1;
    S_run1 = load(fullfile(preprocessedPath, results(i_run).filename));
    
    % 1. Calc raw intersection
    t_start_raw = max(S_run1.Real_Wrench.Time(1), S_run1.Predicted_Wrench.Time(1));
    t_end_raw = min(S_run1.Real_Wrench.Time(end), S_run1.Predicted_Wrench.Time(end));
    
    % 2. Apply Trim
    t_trim_start = t_start_raw + TRIM_SECONDS;
    t_trim_end = t_end_raw - TRIM_SECONDS;
    
    raw_time_vec = S_run1.Real_Wrench.Time;
    time_common_1 = raw_time_vec(raw_time_vec >= t_trim_start & raw_time_vec <= t_trim_end);
    
    real_data_1 = resample(S_run1.Real_Wrench, time_common_1).Data;
    pred_data_1 = resample(S_run1.Predicted_Wrench, time_common_1).Data;
    nom_data_1 = resample(S_run1.Nom_Predicted_Wrench, time_common_1).Data;
    
    error_data_1_est = real_data_1 - pred_data_1;
    error_data_1_nom = real_data_1 - nom_data_1;
    
    fig1a = figure('Name', 'Fig 1a: Single Run Wrench Tracking', 'Position', [100 100 900 600]);
    tLayout1a = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for k = 1:6
        nexttile(tLayout1a);
        plot(time_common_1, real_data_1(:,k), 'k-', 'LineWidth', 1.5); hold on;
        plot(time_common_1, pred_data_1(:,k), '-', 'Color', c.est, 'LineWidth', 1.5);
        plot(time_common_1, nom_data_1(:,k), '--', 'Color', c.nom, 'LineWidth', 1.5);
        title(dof_labels{k}); grid on;
        if k == 1, legend('True', 'Estimated (UKF)', 'Nominal (Guess)', 'Location', 'northwest'); end 
        if k >= 5, xlabel('Time (s)'); end
    end
    title(tLayout1a, sprintf('Figure 1a: True vs. Estimated (Trimmed %.1fs)', TRIM_SECONDS));
    saveas(fig1a, fullfile(plotOutputPath, 'Fig1a_Wrench_Tracking.png'));
    
    fig1b = figure('Name', 'Fig 1b: Single Run Error', 'Position', [150 150 900 600]);
    tLayout1b = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for k = 1:6
        nexttile(tLayout1b);
        plot(time_common_1, error_data_1_est(:,k), '-', 'Color', c.est, 'LineWidth', 1); hold on;
        plot(time_common_1, error_data_1_nom(:,k), '--', 'Color', c.nom, 'LineWidth', 1);
        title(dof_labels{k}); grid on;
        if k == 1, legend('Estimated (UKF) Error', 'Nominal (Guess) Error', 'Location', 'best'); end
        if k >= 5, xlabel('Time (s)'); end
        ylabel('Error');
    end
    title(tLayout1b, sprintf('Figure 1b: Instantaneous Error (Trimmed %.1fs)', TRIM_SECONDS));
    saveas(fig1b, fullfile(plotOutputPath, 'Fig1b_Error_Tracking.png'));
catch ME
    fprintf('Skipping Fig 1: %s\n', ME.message);
end

% --- B. Aggregate Temporal Behavior (Fig 2a, 2b) ---
try
    q_median_est = median(all_errors_interp_est, 3, 'omitnan'); 
    q_25_est = prctile(all_errors_interp_est, 25, 3);
    q_75_est = prctile(all_errors_interp_est, 75, 3);
    
    q_median_nom = median(all_errors_interp_nom, 3, 'omitnan');
    q_25_nom = prctile(all_errors_interp_nom, 25, 3);
    q_75_nom = prctile(all_errors_interp_nom, 75, 3);
    
    fig2a = figure('Name', 'Fig 2a: Error Quantile Bands', 'Position', [100 100 900 600]);
    tLayout2a = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for k = 1:6
        nexttile(tLayout2a);
        fill([master_time; flipud(master_time)], [q_25_nom(:,k); flipud(q_75_nom(:,k))], c.nom, 'EdgeColor', 'none', 'FaceAlpha', 0.3); hold on;
        fill([master_time; flipud(master_time)], [q_25_est(:,k); flipud(q_75_est(:,k))], c.est, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
        plot(master_time, q_median_est(:,k), '-', 'Color', c.est, 'LineWidth', 2);
        plot(master_time, q_median_nom(:,k), '-', 'Color', c.nom, 'LineWidth', 2);
        title(dof_labels{k}); grid on;
        if k == 1, legend('IQR (Nominal)', 'IQR (Estimated)', 'Median (Estimated)', 'Median (Nominal)'); end
        if k >= 5, xlabel('Time (s) [Relative to Trim Start]'); end
        ylabel('Error');
    end
    title(tLayout2a, 'Figure 2a: Aggregate Error Quantiles (Trimmed Data)');
    saveas(fig2a, fullfile(plotOutputPath, 'Fig2a_Quantile_Bands.png'));
catch ME
    fprintf('Skipping Fig 2a: %s\n', ME.message);
end

try
    rms_evolution_est = sqrt(mean(all_errors_interp_est.^2, 3, 'omitnan'));
    rms_evolution_nom = sqrt(mean(all_errors_interp_nom.^2, 3, 'omitnan'));
    
    fig2b = figure('Name', 'Fig 2b: RMS-Error Evolution', 'Position', [150 150 900 600]);
    tLayout2b = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for k=1:6
        nexttile(tLayout2b);
        plot(master_time, rms_evolution_est(:,k), '-', 'Color', c.est, 'LineWidth', 1.5); hold on;
        plot(master_time, rms_evolution_nom(:,k), '--', 'Color', c.nom, 'LineWidth', 1.5);
        title(dof_labels{k}); grid on;
        if k==1, legend('Estimated (UKF)', 'Nominal (Guess)'); end 
        if k>=5, xlabel('Time (s) [Relative]'); end
        ylabel('RMS Error');
    end
    title(tLayout2b, 'Figure 2b: RMS Error Evolution (Trimmed Data)');
    saveas(fig2b, fullfile(plotOutputPath, 'Fig2b_RMSE_Evolution.png'));
catch ME
    fprintf('Skipping Fig 2b: %s\n', ME.message);
end

% --- C. Statistical Summaries (Fig 3) ---
try
    fig3 = figure('Name', 'Fig 3: Final RMSE Distribution (All Runs)', 'Position', [100 100 1000 450]);
    
    ax1 = subplot(1, 2, 1);
    h_est = boxplot(all_rmse_per_dof_est, 'Labels', dof_labels_short, 'Colors', c.est, 'MedianStyle', 'target');
    set(h_est, {'linew'}, {1.5}); 
    title('Figure 3: Estimated (UKF) RMSE'); 
    ylabel('RMSE'); grid on;
    
    ax2 = subplot(1, 2, 2);
    h_nom = boxplot(all_rmse_per_dof_nom, 'Labels', dof_labels_short, 'Colors', c.nom, 'MedianStyle', 'target');
    set(h_nom, {'linew'}, {1.5}); 
    title('Figure 3: Nominal (Guess) RMSE');
    ylabel('RMSE'); grid on;
    
    max_y = max(max(get(ax1, 'YLim')), max(get(ax2, 'YLim')));
    set(ax1, 'YLim', [0 max_y*1.05]);
    set(ax2, 'YLim', [0 max_y*1.05]);
    saveas(fig3, fullfile(plotOutputPath, 'Fig3_RMSE_Boxplots.png'));
catch ME
    fprintf('Skipping Fig 3: %s\n', ME.message);
end

% --- D. Variation Sensitivity (Fig 4a, 4b) ---
try
    % Figure 4a: Scatter of total variation vs. total RMSE
    % Now includes OUTLIER labeling.
    
    % 1. Get feature fields & Build Table
    all_field_names = fields(all_features);
    feature_paths = {};
    for j = 1:length(all_field_names)
        if isfield(all_features(1).(all_field_names{j}), 'mean_deviation')
            feature_paths{end+1} = [all_field_names{j} '.mean_deviation'];
        end
    end
    
    raw_feature_table = zeros(numFiles, length(feature_paths));
    for i_file = 1:numFiles
        s = all_features(i_file);
        for j_feat = 1:length(feature_paths)
            f_path = feature_paths{j_feat};
            parts = strsplit(f_path, '.');
            val = s;
            for k = 1:length(parts)
                part = parts{k};
                array_match = regexp(part, '(\w+)\((\d+)\)', 'tokens');
                if ~isempty(array_match)
                    fieldName = array_match{1}{1};
                    idx = str2double(array_match{1}{2});
                    val = val.(fieldName)(idx);
                else
                    val = val.(part);
                end
            end
            raw_feature_table(i_file, j_feat) = val;
        end
    end
    
    % Z-score and Sum
    norm_feature_table = zscore(raw_feature_table);
    total_norm_var = sum(abs(norm_feature_table), 2);
    
    total_rmse_nom = vecnorm(all_rmse_per_dof_nom, 2, 2);
    
    fig4a = figure('Name', 'Fig 4a: Variation vs. RMSE', 'Position', [100 100 800 600]);
    
    % Plot Standard Points
    s1 = scatter(total_norm_var, total_rmse_est, 30, 'filled', 'MarkerFaceColor', c.est, 'MarkerFaceAlpha', 0.6); hold on;
    s2 = scatter(total_norm_var, total_rmse_nom, 30, 's', 'MarkerEdgeColor', c.nom, 'MarkerFaceAlpha', 0.6);
    
    % Plot Outliers (if any)
    if ~isempty(outlier_indices)
         s3 = plot(total_norm_var(outlier_indices), total_rmse_est(outlier_indices), ...
             'x', 'Color', 'r', 'MarkerSize', 12, 'LineWidth', 2);
         text(total_norm_var(outlier_indices), total_rmse_est(outlier_indices), ...
             string(outlier_indices), 'VerticalAlignment', 'bottom', 'Color', 'r', 'FontSize', 8);
    else
        s3 = [];
    end
    
    % Regressions (include all points for trend)
    p_est = polyfit(total_norm_var, total_rmse_est, 1);
    plot(total_norm_var, polyval(p_est, total_norm_var), '-', 'Color', c.est, 'LineWidth', 2);
    p_nom = polyfit(total_norm_var, total_rmse_nom, 1);
    plot(total_norm_var, polyval(p_nom, total_norm_var), '--', 'Color', c.nom, 'LineWidth', 2);
    
    title(sprintf('Figure 4a: Normalized Variation vs. RMSE (Trimmed %.1fs)', TRIM_SECONDS));
    xlabel('Normalized Total Variation (Sum of |Z-Scores|)');
    ylabel('Total RMSE (L2-Norm)');
    grid on;
    
    if isempty(s3)
        legend([s1 s2], 'Estimated (UKF)', 'Nominal (Guess)', 'Location', 'best');
    else
        legend([s1 s2 s3], 'Estimated (UKF)', 'Nominal (Guess)', 'Outliers', 'Location', 'best');
    end
    saveas(fig4a, fullfile(plotOutputPath, 'Fig4a_Variation_vs_RMSE.png'));
catch ME
    fprintf('Skipping Fig 4a: %s\n', ME.message);
end

try
    % Figure 4b: Feature Correlation Heatmap
    features_to_correlate = {
        'K_V.mean_deviation', 'K_E.mean_deviation', 'R.mean_deviation', ...
        'COM.per_axis_deviation(3)'
    };
    
    feature_data_table = zeros(numFiles, length(features_to_correlate));
    for i_file = 1:numFiles
        s = all_features(i_file);
        for j_feat = 1:length(features_to_correlate)
            f_path = features_to_correlate{j_feat};
            parts = strsplit(f_path, '.');
            val = s;
            for k = 1:length(parts)
                part = parts{k};
                array_match = regexp(part, '(\w+)\((\d+)\)', 'tokens');
                if ~isempty(array_match)
                    fieldName = array_match{1}{1};
                    idx = str2double(array_match{1}{2});
                    val = val.(fieldName)(idx);
                else
                    val = val.(part);
                end
            end
            feature_data_table(i_file, j_feat) = val;
        end
    end
    
    feature_labels = strrep(features_to_correlate, '.mean_deviation', '');
    feature_labels = strrep(feature_labels, '.per_axis_deviation', '');
    feature_labels = strrep(feature_labels, '(3)', '_Z');
    
    % Est Only, Absolute
    results_table = [all_rmse_per_dof_est, total_rmse_est];
    results_labels = [strcat(dof_labels_short, ' (Est)'), {'Total RMSE'}];
    
    corr_matrix = abs(corr(feature_data_table, results_table, 'Rows', 'complete'));
    
    fig4b = figure('Name', 'Fig 4b: Feature Correlation', 'Position', [100 100 800 500]);
    heatmap(results_labels, feature_labels, corr_matrix, ...
            'Title', 'Figure 4b: Feature Importance (Absolute Correlation)', ...
            'Colormap', parula, 'ColorLimits', [0 1]);
    saveas(fig4b, fullfile(plotOutputPath, 'Fig4b_Correlation_Heatmap.png'));
catch ME
    fprintf('Skipping Fig 4b: %s\n', ME.message);
end

% --- E. Cross-DOF Behavior (Fig 5) ---
try
    rmse_norm_cols_est = (all_rmse_per_dof_est - min(all_rmse_per_dof_est, [], 1)) ./ ...
                         (max(all_rmse_per_dof_est, [], 1) - min(all_rmse_per_dof_est, [], 1));
    
    row_magnitudes_est = vecnorm(rmse_norm_cols_est, 2, 2);
    [~, sort_idx] = sort(row_magnitudes_est, 'ascend');
    
    fig5 = figure('Name', 'Fig 5: Error Fingerprint', 'Position', [100 100 1100 500]);
    
    subplot(1, 2, 1);
    imagesc(rmse_norm_cols_est(sort_idx, :));
    title('Fig 5: Estimated (UKF) Fingerprint'); 
    xlabel('DOF'); ylabel('Simulation Run (Sorted)');
    set(gca, 'XTick', 1:6, 'XTickLabel', dof_labels_short, 'XTickLabelRotation', 30);
    colorbar; clim([0 1]);
    
    rmse_norm_cols_nom = (all_rmse_per_dof_nom - min(all_rmse_per_dof_nom, [], 1)) ./ ...
                         (max(all_rmse_per_dof_nom, [], 1) - min(all_rmse_per_dof_nom, [], 1));
    rmse_norm_cols_nom(isnan(rmse_norm_cols_nom)) = 0; 
    
    subplot(1, 2, 2);
    imagesc(rmse_norm_cols_nom(sort_idx, :));
    title('Fig 5: Nominal (Guess) Fingerprint');
    xlabel('DOF');
    set(gca, 'XTick', 1:6, 'XTickLabel', dof_labels_short, 'XTickLabelRotation', 30);
    colorbar; clim([0 1]);
    
    saveas(fig5, fullfile(plotOutputPath, 'Fig5_Error_Fingerprint.png'));
catch ME
    fprintf('Skipping Fig 5: %s\n', ME.message);
end

fprintf('\n-------------------------------------------------\n');
fprintf('All plotting complete. Figures saved to: %s\n', plotOutputPath);
fprintf('-------------------------------------------------\n');