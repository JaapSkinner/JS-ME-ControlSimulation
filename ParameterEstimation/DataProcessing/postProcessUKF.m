%% ANALYZE PREPROCESSED SIMULATION DATA
%
% This script loads all files from a '..._Preprocessed' folder,
% aggregates data, calculates wrench prediction error metrics, and plots
% them against the simulation's ground-truth feature variations.
%
% This version also loads the "Nominal" (pre-estimation guess) wrench
% prediction to compare the estimator's performance against the baseline.
%
% Steps:
%   1. Asks user to select the 'UKF_Preprocessed' folder.
%   2. Creates a 'Plots' folder to save all figures.
%   3. Initializes an empty 'results' struct array.
%   4. For each 'preprocessed_*.mat' file:
%      a. Loads the file.
%      b. Aligns Real_Wrench, Predicted_Wrench (Estimated), and
%         Nom_Predicted_Wrench (Nominal) timeseries.
%      c. Calculates the full error time-series for both 'Estimated' and 'Nominal'.
%      d. Calculates per-DOF RMSE and overall magnitude RMSE for both.
%      e. Stores all metrics, features, and error time-series in the
%         'results' struct.
%   5. After the loop, prepares aggregated data for plotting by
%      interpolating all error series onto a common time grid.
%   6. Generates a comprehensive set of plots comparing Estimator vs. Nominal
%      performance, saving each figure to the 'Plots' folder.
%
clear; clc; close all;

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

% Create a 'Plots' folder in the same directory as the preprocessedPath
[parentFolder, ~] = fileparts(preprocessedPath);
plotOutputPath = fullfile(parentFolder, 'UKF_Analysis_Plots2');
if ~isfolder(plotOutputPath)
    fprintf('Creating plot output directory: %s\n', plotOutputPath);
    mkdir(plotOutputPath);
else
    fprintf('Plot output directory already exists: %s\n', plotOutputPath);
end

resultFiles = dir(fullfile(preprocessedPath, 'preprocessed_estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No preprocessed_estimation_*.mat files found in: %s', preprocessedPath); end
fprintf('Found %d preprocessed files. Aggregating results...\n', numFiles);

%% --- 2. Initialize Results Aggregator & Plot Settings ---
% We store more data now, including the full error timeseries for each run
% for both the Estimated (UKF) and Nominal (Guess) predictions.
results = struct('filename', {}, ...
                 'rmse_per_dof_est', {}, ...         % [1x6] vector of RMSE for (Fx,Fy,Fz,Tx,Ty,Tz)
                 'rmse_overall_force_N_est', {}, ... % Scalar L2-norm RMSE for force
                 'rmse_overall_torque_Nm_est', {}, ...% Scalar L2-norm RMSE for torque
                 'wrench_error_est', {}, ...        % [Tx6] matrix
                 'rmse_per_dof_nom', {}, ...         % [1x6] vector
                 'rmse_overall_force_N_nom', {}, ... % Scalar L2-norm
                 'rmse_overall_torque_Nm_nom', {}, ...% Scalar L2-norm
                 'wrench_error_nom', {}, ...        % [Tx6] matrix
                 'time', {}, ...                 % [Tx1] vector
                 'features', {});               % Struct of ground-truth variations

% Pre-allocate for speed
results(numFiles).filename = [];

% Plotting Labels and Colors
dof_labels = {'F_x (N)', 'F_y (N)', 'F_z (N)', '\tau_x (N·m)', '\tau_y (N·m)', '\tau_z (N·m)'};
dof_labels_short = {'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'};

c.true = [0 0 0];                         % Black
c.est = [0.0 0.4470 0.7410];            % Blue
c.nom = [0.8500 0.3250 0.0980];         % Red
c.est_fill = [c.est, 0.2];               % Blue with alpha
c.nom_fill = [c.nom, 0.2];               % Red with alpha

%% --- 3. Process Each File and Aggregate Results ---
processedCount = 0;
for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(preprocessedPath, currentFileName);
    fprintf('Aggregating file (%d/%d): %s\n', i, numFiles, currentFileName);

    try
        S = load(fullFilePath);
        % Check for all required data, including new Nominal wrench
        if ~isfield(S, 'Real_Wrench') || ~isfield(S, 'Predicted_Wrench') || ...
           ~isfield(S, 'Nom_Predicted_Wrench') || ~isfield(S, 'features_i') || ~isfield(S, 'B_nom')
            warning('File %s is missing required data (Real/Predicted/Nominal Wrench or features_i/B_nom). Skipping.', currentFileName);
            continue;
        end
        
        % --- b. Align Time ---
        time_start = max(S.Real_Wrench.Time(1), S.Predicted_Wrench.Time(1));
        time_end = min(S.Real_Wrench.Time(end), S.Predicted_Wrench.Time(end));
        time_common = S.Real_Wrench.Time(S.Real_Wrench.Time >= time_start & S.Real_Wrench.Time <= time_end);
        
        if isempty(time_common) || length(time_common) < 10 % Skip if not enough data
            warning('File %s has no overlapping time data. Skipping.', currentFileName);
            continue;
        end
        
        real_wrench_data = resample(S.Real_Wrench, time_common).Data;
        est_pred_wrench_data = resample(S.Predicted_Wrench, time_common).Data;
        nom_pred_wrench_data = resample(S.Nom_Predicted_Wrench, time_common).Data;

        % --- c. Calculate Error (Estimated vs. Nominal) ---
        wrenchError_est = real_wrench_data - est_pred_wrench_data; % [Time x 6]
        wrenchError_nom = real_wrench_data - nom_pred_wrench_data; % [Time x 6]

        % --- d. Calculate Scalar Metrics (Estimated) ---
        forceError_N_est = wrenchError_est(:, 1:3);
        torqueError_Nm_est = wrenchError_est(:, 4:6);
        rmse_per_dof_est = rms(wrenchError_est, 1); % [1x6] vector
        forceErrorMagnitude_est = vecnorm(forceError_N_est, 2, 2);
        torqueErrorMagnitude_est = vecnorm(torqueError_Nm_est, 2, 2);
        rmseForceMag_est = rms(forceErrorMagnitude_est);   % Scalar
        rmseTorqueMag_est = rms(torqueErrorMagnitude_est); % Scalar
        
        % --- d. Calculate Scalar Metrics (Nominal) ---
        forceError_N_nom = wrenchError_nom(:, 1:3);
        torqueError_Nm_nom = wrenchError_nom(:, 4:6);
        rmse_per_dof_nom = rms(wrenchError_nom, 1); % [1x6] vector
        forceErrorMagnitude_nom = vecnorm(forceError_N_nom, 2, 2);
        torqueErrorMagnitude_nom = vecnorm(torqueError_Nm_nom, 2, 2);
        rmseForceMag_nom = rms(forceErrorMagnitude_nom);   % Scalar
        rmseTorqueMag_nom = rms(torqueErrorMagnitude_nom); % Scalar

        % --- e. Store in Aggregator ---
        results(i).filename = currentFileName;
        results(i).time = time_common;
        results(i).features = S.features_i;
        % Estimated
        results(i).rmse_per_dof_est = rmse_per_dof_est;
        results(i).rmse_overall_force_N_est = rmseForceMag_est;
        results(i).rmse_overall_torque_Nm_est = rmseTorqueMag_est;
        results(i).wrench_error_est = wrenchError_est;
        % Nominal
        results(i).rmse_per_dof_nom = rmse_per_dof_nom;
        results(i).rmse_overall_force_N_nom = rmseForceMag_nom;
        results(i).rmse_overall_torque_Nm_nom = rmseTorqueMag_nom;
        results(i).wrench_error_nom = wrenchError_nom;
        
        processedCount = processedCount + 1;
    catch ME
        fprintf(2, 'ERROR processing file %s: %s\n', currentFileName, ME.message);
        fprintf(2, 'Skipping this file.\n');
    end
end

% Clean up empty entries
results = results(1:processedCount);
if processedCount == 0, error('No files were successfully processed.'); end
numFiles = processedCount; % Update numFiles to reflect actual processed count

fprintf('\n-------------------------------------------------\n');
fprintf('Aggregation Complete. Processed %d files.\n', processedCount);
fprintf('The aggregated data is stored in the ''results'' struct array.\n');
fprintf('-------------------------------------------------\n');


%% --- 4. Plotting Data Preparation ---
% To create aggregate temporal plots (like Fig 2), we must interpolate all
% error time-series onto a single, common time grid.

fprintf('Preparing aggregated data for temporal plots...\n');
try
    all_time_cell = {results.time};
    
    % Find a master time grid. Use median start/end/step.
    t_start = median(cellfun(@(t) t(1), all_time_cell));
    t_end = median(cellfun(@(t) t(end), all_time_cell));
    t_step = median(cellfun(@(t) median(diff(t)), all_time_cell));
    
    master_time = (t_start:t_step:t_end)';
    nTimeSteps = length(master_time);

    % Create [Time x 6 DOFs x N Runs] tensors for both Est and Nom errors
    all_errors_interp_est = zeros(nTimeSteps, 6, numFiles);
    all_errors_interp_nom = zeros(nTimeSteps, 6, numFiles);
    
    for i = 1:numFiles
        % Interpolate this run's error onto the master time grid
        all_errors_interp_est(:,:,i) = interp1(results(i).time, results(i).wrench_error_est, master_time, 'linear', NaN);
        all_errors_interp_nom(:,:,i) = interp1(results(i).time, results(i).wrench_error_nom, master_time, 'linear', NaN);
    end
    
    % Get all per-DOF RMSEs into simple [N Runs x 6 DOFs] matrices
    all_rmse_per_dof_est = vertcat(results.rmse_per_dof_est);
    all_rmse_per_dof_nom = vertcat(results.rmse_per_dof_nom);
    
    % Get all features into a struct array
    all_features = [results.features];

    fprintf('Data preparation complete.\n');
catch ME
    fprintf(2, 'ERROR during data preparation: %s\n', ME.message);
    fprintf(2, 'Cannot generate temporal plots (Fig 1, 2).\n');
    % We can still try to generate Fig 3, 4, 5
    all_rmse_per_dof_est = vertcat(results.rmse_per_dof_est);
    all_rmse_per_dof_nom = vertcat(results.rmse_per_dof_nom);
    all_features = [results.features];
end


%% --- 5. Plot Generation: B-Matrix Estimation Analysis ---
fprintf('Generating plots...\n');

% --- A.1. Illustrative Single-Run Example (Fig 1a, 1b) ---
try
    fprintf('  Generating Figure 1a, 1b (Single-Run Example)...\n');
    % Load data for the *first* processed run
    i_run = 1;
    S_run1 = load(fullfile(preprocessedPath, results(i_run).filename));
    
    % Re-align this run's data
    time_start_1 = max([S_run1.Real_Wrench.Time(1), S_run1.Predicted_Wrench.Time(1), S_run1.Nom_Predicted_Wrench.Time(1)]);
    time_end_1 = min([S_run1.Real_Wrench.Time(end), S_run1.Predicted_Wrench.Time(end), S_run1.Nom_Predicted_Wrench.Time(end)]);
    time_common_1 = S_run1.Real_Wrench.Time(S_run1.Real_Wrench.Time >= time_start_1 & S_run1.Real_Wrench.Time <= time_end_1);
    
    real_data_1 = resample(S_run1.Real_Wrench, time_common_1).Data;
    pred_data_1 = resample(S_run1.Predicted_Wrench, time_common_1).Data;
    nom_data_1 = resample(S_run1.Nom_Predicted_Wrench, time_common_1).Data;
    
    error_data_1_est = real_data_1 - pred_data_1;
    error_data_1_nom = real_data_1 - nom_data_1;

    % Figure 1a: True vs. Estimated vs. Nominal Time-Series
    fig1a = figure('Name', 'Fig 1a: Single Run Wrench Tracking', 'Position', [100 100 900 600]);
    tLayout1a = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for k = 1:6
        nexttile(tLayout1a);
        plot(time_common_1, real_data_1(:,k), 'k-', 'LineWidth', 1.5); hold on;
        plot(time_common_1, pred_data_1(:,k), '-', 'Color', c.est, 'LineWidth', 1.5);
        plot(time_common_1, nom_data_1(:,k), '--', 'Color', c.nom, 'LineWidth', 1.5);
        title(dof_labels{k});
        grid on;
        if k == 1, legend('True', 'Estimated (UKF)', 'Nominal (Guess)', 'Location', 'northwest'); end
        if k >= 5, xlabel('Time (s)'); end
    end
    title(tLayout1a, 'Figure 1a: True vs. Estimated Wrench (Single Run)');
    saveas(fig1a, fullfile(plotOutputPath, 'Fig1a_Wrench_Tracking.png'));

    % Figure 1b: Error Time-Series (Estimated vs. Nominal)
    fig1b = figure('Name', 'Fig 1b: Single Run Error (True - Est)', 'Position', [150 150 900 600]);
    tLayout1b = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for k = 1:6
        nexttile(tLayout1b);
        plot(time_common_1, error_data_1_est(:,k), '-', 'Color', c.est, 'LineWidth', 1); hold on;
        plot(time_common_1, error_data_1_nom(:,k), '--', 'Color', c.nom, 'LineWidth', 1);
        title(dof_labels{k});
        grid on;
        if k == 1, legend('Estimated (UKF) Error', 'Nominal (Guess) Error', 'Location', 'best'); end
        if k >= 5, xlabel('Time (s)'); end
        ylabel('Error');
    end
    title(tLayout1b, 'Figure 1b: Instantaneous Error (Single Run)');
    saveas(fig1b, fullfile(plotOutputPath, 'Fig1b_Error_Tracking.png'));

catch ME
    fprintf(2, 'Could not generate Figure 1: %s\n', ME.message);
end


% --- A.2. Aggregate Temporal Behavior (Fig 2a, 2b, 2c) ---
% These plots depend on 'all_errors_interp' from Section 4
if exist('all_errors_interp_est', 'var')
    try
        fprintf('  Generating Figure 2a (Error Density)...\n');
        % Figure 2a: Error Density Plot (Estimated)
        fig2a_est = figure('Name', 'Fig 2a: Estimated Error Density', 'Position', [100 100 900 600]);
        tLayout2a_est = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
        for k = 1:6
            nexttile(tLayout2a_est);
            error_vec_k = reshape(all_errors_interp_est(:, k, :), [], 1);
            time_vec_k = repmat(master_time, numFiles, 1);
            nan_mask = isnan(error_vec_k);
            histogram2(time_vec_k(~nan_mask), error_vec_k(~nan_mask), ...
                'DisplayStyle', 'tile', 'ShowEmptyBins', 'off', 'Normalization', 'pdf');
            title(dof_labels{k});
            if k >= 5, xlabel('Time (s)'); end
            ylabel('Error'); colorbar;
        end
        title(tLayout2a_est, 'Figure 2a (Estimated): Error Density (All Runs)');
        saveas(fig2a_est, fullfile(plotOutputPath, 'Fig2a_Density_Estimated.png'));

        % Figure 2a (Nominal): Error Density Plot
        fig2a_nom = figure('Name', 'Fig 2a (Nominal): Nominal Error Density', 'Position', [150 150 900 600]);
        tLayout2a_nom = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
        for k = 1:6
            nexttile(tLayout2a_nom);
            error_vec_k = reshape(all_errors_interp_nom(:, k, :), [], 1);
            time_vec_k = repmat(master_time, numFiles, 1);
            nan_mask = isnan(error_vec_k);
            histogram2(time_vec_k(~nan_mask), error_vec_k(~nan_mask), ...
                'DisplayStyle', 'tile', 'ShowEmptyBins', 'off', 'Normalization', 'pdf');
            title(dof_labels{k});
            if k >= 5, xlabel('Time (s)'); end
            ylabel('Error'); colorbar;
        end
        title(tLayout2a_nom, 'Figure 2a (Nominal): Error Density (All Runs)');
        saveas(fig2a_nom, fullfile(plotOutputPath, 'Fig2a_Density_Nominal.png'));

    catch ME
        fprintf(2, 'Could not generate Figure 2a: %s\n', ME.message);
    end

    try
        fprintf('  Generating Figure 2b (Quantile Bands)...\n');
        % Figure 2b: Quantile Bands (Estimated vs. Nominal)
        q_median_est = median(all_errors_interp_est, 3, 'omitnan'); % [Time x 6]
        q_25_est = prctile(all_errors_interp_est, 25, 3);
        q_75_est = prctile(all_errors_interp_est, 75, 3);
        
        q_median_nom = median(all_errors_interp_nom, 3, 'omitnan');
        q_25_nom = prctile(all_errors_interp_nom, 25, 3);
        q_75_nom = prctile(all_errors_interp_nom, 75, 3);
        
        fig2b = figure('Name', 'Fig 2b: Error Quantile Bands', 'Position', [100 100 900 600]);
        tLayout2b = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
        for k = 1:6
            nexttile(tLayout2b);
            % Plot Nominal IQR (Reds)
            fill([master_time; flipud(master_time)], [q_25_nom(:,k); flipud(q_75_nom(:,k))], c.nom, 'EdgeColor', 'none', 'FaceAlpha', 0.3); hold on;
            % Plot Estimated IQR (Blues)
            fill([master_time; flipud(master_time)], [q_25_est(:,k); flipud(q_75_est(:,k))], c.est, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
            % Plot Medians
            plot(master_time, q_median_est(:,k), '-', 'Color', c.est, 'LineWidth', 2);
            plot(master_time, q_median_nom(:,k), '-', 'Color', c.nom, 'LineWidth', 2);
            title(dof_labels{k});
            grid on;
            if k == 1, legend('IQR (Nominal)', 'IQR (Estimated)', 'Median (Estimated)', 'Median (Nominal)'); end
            if k >= 5, xlabel('Time (s)'); end
            ylabel('Error');
        end
        title(tLayout2b, 'Figure 2b: Aggregate Error Quantiles (IQR & Median)');
        saveas(fig2b, fullfile(plotOutputPath, 'Fig2b_Quantile_Bands.png'));

    catch ME
        fprintf(2, 'Could not generate Figure 2b: %s\n', ME.message);
    end

    try
        fprintf('  Generating Figure 2c (RMS-Error Evolution)...\n');
        % Figure 2c: RMS-Error Evolution
        rms_evolution_est = sqrt(mean(all_errors_interp_est.^2, 3, 'omitnan'));
        rms_evolution_nom = sqrt(mean(all_errors_interp_nom.^2, 3, 'omitnan'));
        
        fig2c = figure('Name', 'Fig 2c: RMS-Error Evolution', 'Position', [150 150 900 600]);
        tLayout2c = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
        for k=1:6
            nexttile(tLayout2c);
            plot(master_time, rms_evolution_est(:,k), '-', 'Color', c.est, 'LineWidth', 1.5); hold on;
            plot(master_time, rms_evolution_nom(:,k), '--', 'Color', c.nom, 'LineWidth', 1.5);
            title(dof_labels{k});
            grid on;
            if k==1, legend('Estimated (UKF)', 'Nominal (Guess)'); end
            if k>=5, xlabel('Time (s)'); end
            ylabel('RMS Error');
        end
        title(tLayout2c, 'Figure 2c: RMS Error Evolution (Averaged Across All Runs)');
        saveas(fig2c, fullfile(plotOutputPath, 'Fig2c_RMSE_Evolution.png'));

    catch ME
        fprintf(2, 'Could not generate Figure 2c: %s\n', ME.message);
    end
else
    fprintf(2, 'Skipping aggregate temporal plots (Fig 2) due to data prep error.\n');
end


% --- A.3. Statistical Summaries (Fig 3a, 3b) ---
try
    fprintf('  Generating Figure 3a (RMSE Box Plot)...\n');
    % Figure 3a: Box plots of final RMSE per DOF
    fig3a = figure('Name', 'Fig 3a: Final RMSE Distribution (All Runs)', 'Position', [100 100 1000 450]);
    
    % Create two side-by-side plots for clarity
    ax1 = subplot(1, 2, 1);
    h_est = boxplot(all_rmse_per_dof_est, 'Labels', dof_labels_short, 'Colors', c.est, 'MedianStyle', 'target');
    set(h_est, {'linew'}, {1.5}); % Make lines thicker
    title('Figure 3a: Estimated (UKF) RMSE');
    ylabel('RMSE');
    grid on;
    
    ax2 = subplot(1, 2, 2);
    h_nom = boxplot(all_rmse_per_dof_nom, 'Labels', dof_labels_short, 'Colors', c.nom, 'MedianStyle', 'target');
    set(h_nom, {'linew'}, {1.5}); % Make lines thicker
    title('Figure 3a: Nominal (Guess) RMSE');
    ylabel('RMSE');
    grid on;
    
    % Get max Y-limit from both and apply to both for consistent scaling
    max_y = max(max(get(ax1, 'YLim')), max(get(ax2, 'YLim')));
    set(ax1, 'YLim', [0 max_y*1.05]);
    set(ax2, 'YLim', [0 max_y*1.05]);
    
    saveas(fig3a, fullfile(plotOutputPath, 'Fig3a_RMSE_Boxplots.png'));

catch ME
    fprintf(2, 'Could not generate Figure 3a: %s\n', ME.message);
end

try
    fprintf('  Generating Figure 3b (RMSE ECDFs)...\n');
    % Figure 3b: ECDFs of RMSE
    fig3b = figure('Name', 'Fig 3b: RMSE ECDF per DOF', 'Position', [150 150 700 500]);
    colors = lines(6);
    hold on;
    for k = 1:6
        % Plot Estimated
        [f_est, x_est] = ecdf(all_rmse_per_dof_est(:, k));
        plot(x_est, f_est, '-', 'Color', colors(k,:), 'LineWidth', 2);
        % Plot Nominal
        [f_nom, x_nom] = ecdf(all_rmse_per_dof_nom(:, k));
        plot(x_nom, f_nom, '--', 'Color', colors(k,:), 'LineWidth', 1.5);
    end
    hold off;
    
    % Create legend
    legend_labels = cell(6*2, 1);
    for k=1:6
        legend_labels{2*k-1} = [dof_labels_short{k} ' (Est)'];
        legend_labels{2*k} = [dof_labels_short{k} ' (Nom)'];
    end
    legend(legend_labels, 'Location', 'best');
    title('Figure 3b: Empirical CDF of RMSE per DOF (Solid=Est, Dashed=Nom)');
    xlabel('RMSE');
    ylabel('Cumulative Probability');
    ylim([0 1]); % Set Y-limit
    grid on;
    saveas(fig3b, fullfile(plotOutputPath, 'Fig3b_RMSE_ECDF.png'));

catch ME
    fprintf(2, 'Could not generate Figure 3b: %s\n', ME.message);
end


%% --- A.4. Variation Sensitivity (Fig 4a, 4b, 4c) ---

% NOTE: Removing problematic anonymous helper functions 'get_feature' and
% 'build_normalized_feature_table'. Replacing with explicit loops.

try
    fprintf('  Generating Figure 4a (Total Variation vs. RMSE)...\n');
    % Figure 4a: Scatter of total variation vs. total RMSE
    
    % Define "Total Variation" - sum of normalized (z-scored) deviations.
    % This treats variations in small (e.g., C_TAU) and large (e.g., K_V)
    % parameters more equally.
    
    % 1. Get all 'mean_deviation' field paths
    all_field_names = fields(all_features);
    feature_paths_to_normalize = {};
    for j = 1:length(all_field_names)
        if isfield(all_features(1).(all_field_names{j}), 'mean_deviation')
            feature_paths_to_normalize{end+1} = [all_field_names{j} '.mean_deviation'];
        end
    end
    
    % 2. Build the feature table (NOT normalized yet)
    % [numFiles x numFeatures]
    raw_feature_table = zeros(numFiles, length(feature_paths_to_normalize));
    for i_file = 1:numFiles
        s = all_features(i_file); % Get the struct for this run
        for j_feat = 1:length(feature_paths_to_normalize)
            f_path = feature_paths_to_normalize{j_feat};
            
            % Robustly access nested fields and array elements
            parts = strsplit(f_path, '.');
            val = s;
            for k = 1:length(parts)
                part = parts{k};
                array_match = regexp(part, '(\w+)\((\d+)\)', 'tokens');
                if ~isempty(array_match)
                    % Handle array access like 'per_axis_deviation(1)'
                    fieldName = array_match{1}{1};
                    idx = str2double(array_match{1}{2});
                    val = val.(fieldName)(idx);
                else
                    % Handle simple field access
                    val = val.(part);
                end
            end
            raw_feature_table(i_file, j_feat) = val;
        end
    end
    
    % 3. Normalize (z-score) the table
    norm_feature_table = zscore(raw_feature_table);
    
    % 4. "Total Variation" is the sum of these z-scores for each run
    % We sum the absolute values, as we care about magnitude of deviation
    total_norm_var = sum(abs(norm_feature_table), 2);
    
    % Total RMSE = L2 norm of the per-DOF RMSE vector
    total_rmse_est = vecnorm(all_rmse_per_dof_est, 2, 2); % [numFiles x 1]
    total_rmse_nom = vecnorm(all_rmse_per_dof_nom, 2, 2); % [numFiles x 1]

    fig4a = figure('Name', 'Fig 4a: Normalized Variation vs. Total RMSE', 'Position', [100 100 700 500]);
    
    % Plot Estimated
    scatter(total_norm_var, total_rmse_est, 30, 'filled', 'MarkerFaceColor', c.est, 'MarkerFaceAlpha', 0.6);
    hold on;
    % Plot Nominal
    scatter(total_norm_var, total_rmse_nom, 30, 's', 'MarkerEdgeColor', c.nom, 'MarkerFaceAlpha', 0.6);
    
    % Add regression lines
    p_est = polyfit(total_norm_var, total_rmse_est, 1);
    plot(total_norm_var, polyval(p_est, total_norm_var), '-', 'Color', c.est, 'LineWidth', 2);
    
    p_nom = polyfit(total_norm_var, total_rmse_nom, 1);
    plot(total_norm_var, polyval(p_nom, total_norm_var), '--', 'Color', c.nom, 'LineWidth', 2);
    
    title('Figure 4a: Normalized Ground-Truth Variation vs. Total Estimation RMSE');
    xlabel('Normalized Total Variation (Sum of |Z-Scores|)');
    ylabel('Total RMSE (L2-Norm of per-DOF RMSE)');
    grid on;
    legend('Estimated (UKF) Runs', 'Nominal (Guess) Runs', 'Estimated Fit', 'Nominal Fit', 'Location', 'best');
    saveas(fig4a, fullfile(plotOutputPath, 'Fig4a_Variation_vs_RMSE.png'));

catch ME
    fprintf(2, 'Could not generate Figure 4a: %s\n', ME.message);
end

try
    fprintf('  Generating Figure 4b (COM Deviation Landscape)...\n');
    % Figure 4b: 2D landscape (COM X-dev vs Z-dev)
    
    % FIX: Removed '.features' - all_features is the struct array
    com_x_dev = arrayfun(@(s) s.COM.per_axis_deviation(1), all_features);
    com_x_dev = com_x_dev(:); % Force to column vector
    com_z_dev = arrayfun(@(s) s.COM.per_axis_deviation(3), all_features);
    com_z_dev = com_z_dev(:); % Force to column vector
    
    total_rmse_est = vecnorm(all_rmse_per_dof_est, 2, 2); % This is already a column vector

    % Create an interpolant
    F = scatteredInterpolant(com_x_dev, com_z_dev, total_rmse_est, 'linear', 'none');
    
    % Create a grid to query the interpolant
    [xq, yq] = meshgrid(linspace(min(com_x_dev), max(com_x_dev), 50), ...
                        linspace(min(com_z_dev), max(com_z_dev), 50));
    zq = F(xq, yq);
    
    fig4b = figure('Name', 'Fig 4b: 2D Performance Landscape (COM)', 'Position', [150 150 700 500]);
    contourf(xq, yq, zq, 20, 'LineStyle', 'none');
    hold on;
    scatter(com_x_dev, com_z_dev, 20, 'k', 'filled'); % Show original points
    title('Figure 4b: Estimation RMSE vs. COM Variation');
    xlabel('COM X-Axis Deviation (m)');
    ylabel('COM Z-Axis Deviation (m)');
    colorbar;
    c = colorbar;
    c.Label.String = 'Total RMSE (Est)';
    clim([min(total_rmse_est), max(total_rmse_est)]);
    
    saveas(fig4b, fullfile(plotOutputPath, 'Fig4b_COM_Landscape.png'));
    
catch ME
    fprintf(2, 'Could not generate Figure 4b: %s\n', ME.message);
end

try
    fprintf('  Generating Figure 4c (Feature Importance)...\n');
    % Figure 4c: Feature Importance (Correlation Heatmap)
    
    % Define features to correlate, now including COM per-axis
    features_to_correlate = {
        'K_V.mean_deviation', 'K_E.mean_deviation', 'R.mean_deviation', ...
        'COM.per_axis_deviation(3)'
    };
    
    feature_data_table = zeros(numFiles, length(features_to_correlate));
    
    % FIX: Replaced problematic arrayfun/eval with a robust loop
    for i_file = 1:numFiles
        s = all_features(i_file); % Get struct for this run
        for j_feat = 1:length(features_to_correlate)
            f_path = features_to_correlate{j_feat};
            
            % Robustly access nested fields and array elements
            parts = strsplit(f_path, '.');
            val = s;
            for k = 1:length(parts)
                part = parts{k};
                array_match = regexp(part, '(\w+)\((\d+)\)', 'tokens');
                if ~isempty(array_match)
                    % Handle array access like 'per_axis_deviation(1)'
                    fieldName = array_match{1}{1};
                    idx = str2double(array_match{1}{2});
                    val = val.(fieldName)(idx);
                else
                    % Handle simple field access
                    val = val.(part);
                end
            end
            feature_data_table(i_file, j_feat) = val;
        end
    end
    
    % Clean up labels for heatmap
    feature_labels = strrep(features_to_correlate, '.mean_deviation', '');
    feature_labels = strrep(feature_labels, '.per_axis_deviation', '');
    feature_labels = strrep(feature_labels, '(1)', '_X');
    feature_labels = strrep(feature_labels, '(2)', '_Y');
    feature_labels = strrep(feature_labels, '(3)', '_Z');

    % Create a matrix of results to correlate against
    total_rmse_est = vecnorm(all_rmse_per_dof_est, 2, 2);
    total_rmse_nom = vecnorm(all_rmse_per_dof_nom, 2, 2);
    results_table = [all_rmse_per_dof_est, all_rmse_per_dof_nom, total_rmse_est, total_rmse_nom];
    
    results_labels_est = strcat(dof_labels_short, ' (Est)');
    results_labels_nom = strcat(dof_labels_short, ' (Nom)');
    results_labels = [results_labels_est, results_labels_nom, {'Total RMSE (Est)'}, {'Total RMSE (Nom)'}];
    
    % Calculate correlation matrix
    corr_matrix = corr(feature_data_table, results_table, 'Rows', 'complete');
    
    fig4c = figure('Name', 'Fig 4c: Feature Importance', 'Position', [100 100 1000 500]);
    heatmap(results_labels, feature_labels, corr_matrix, ...
            'Title', 'Figure 4c: Correlation (Feature Variation vs. RMSE)', ...
            'Colormap', summer, 'ColorLimits', [-1 1]);
    saveas(fig4c, fullfile(plotOutputPath, 'Fig4c_Correlation_Heatmap.png'));
    
catch ME
    fprintf(2, 'Could not generate Figure 4c: %s\n', ME.message);
end


%% --- A.5. Cross-DOF Behavior (Fig 5) ---
try
    fprintf('  Generating Figure 5 (Error Fingerprint)...\n');
    % Figure 5: "Error Fingerprint" Heatmap
    
    % 1. Normalize RMSE per-DOF (each column 0-1)
    rmse_norm_cols_est = (all_rmse_per_dof_est - min(all_rmse_per_dof_est, [], 1)) ./ ...
                         (max(all_rmse_per_dof_est, [], 1) - min(all_rmse_per_dof_est, [], 1));
    
    % 2. Sort rows by overall magnitude (using L2 norm of the normalized row)
    row_magnitudes_est = vecnorm(rmse_norm_cols_est, 2, 2);
    [~, sort_idx] = sort(row_magnitudes_est, 'ascend');
    
    % 3. Plot (Side-by-side)
    fig5 = figure('Name', 'Fig 5: Error Fingerprint', 'Position', [100 100 1100 500]);
    
    % Estimated
    subplot(1, 2, 1);
    imagesc(rmse_norm_cols_est(sort_idx, :));
    title('Fig 5: Estimated (UKF) Error Fingerprint');
    xlabel('DOF');
    ylabel('Simulation Run (Sorted by error)');
    set(gca, 'XTick', 1:6, 'XTickLabel', dof_labels_short, 'XTickLabelRotation', 30);
    colorbar;
    clim([0 1]);
    
    % Nominal (use the same row sorting for direct comparison)
    % Normalize nominal columns
    rmse_norm_cols_nom = (all_rmse_per_dof_nom - min(all_rmse_per_dof_nom, [], 1)) ./ ...
                         (max(all_rmse_per_dof_nom, [], 1) - min(all_rmse_per_dof_nom, [], 1));
    % Handle potential div by zero if a nom column is constant
    rmse_norm_cols_nom(isnan(rmse_norm_cols_nom)) = 0; 

    subplot(1, 2, 2);
    imagesc(rmse_norm_cols_nom(sort_idx, :));
    title('Fig 5: Nominal (Guess) Error Fingerprint');
    xlabel('DOF');
    ylabel('Simulation Run (Sorted by est. error)');
    set(gca, 'XTick', 1:6, 'XTickLabel', dof_labels_short, 'XTickLabelRotation', 30);
    colorbar;
    clim([0 1]);
    
    saveas(fig5, fullfile(plotOutputPath, 'Fig5_Error_Fingerprint.png'));
    
catch ME
    fprintf(2, 'Could not generate Figure 5: %s\n', ME.message);
end

fprintf('\n-------------------------------------------------\n');
fprintf('All plotting complete. Figures saved to: %s\n', plotOutputPath);
fprintf('-------------------------------------------------\n');