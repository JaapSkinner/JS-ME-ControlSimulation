%% ANALYZE PREPROCESSED SIMULATION DATA (UKF vs RLS vs Nominal)
%
% This script loads all files from a '..._Preprocessed' folder,
% aggregates data, calculates wrench prediction error metrics, and plots
% them against the simulation's ground-truth feature variations.
%
% COMPARISON: 
%   1. Nominal (Baseline Guess)
%   2. UKF (Unscented Kalman Filter - Mean of last 5s)
%   3. RLS (Recursive Least Squares - Snapshot at 17s)
%
% DATA PROCESSING: Trims the first and last 5 seconds of data.
%
clear; clc; close all;

%% --- CONFIGURATION ---
TRIM_SECONDS = 5.0; % Number of seconds to cut from start and end

% Plot Colors
c.nom = [0.6350 0.0780 0.1840]; % Red (Nominal)
c.ukf = [0.0000 0.4470 0.7410]; % Blue (UKF)
c.rls = [0.4660 0.6740 0.1880]; % Green (RLS)
c.true = [0 0 0];               % Black (True)
c.outlier = [1 0 0];            % Red (Markers)

%% --- 1. Select Preprocessed Folder & Create Plot Output Folder ---
fprintf('Step 1: Select the folder containing preprocessed result files...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end

preprocessedPath = uigetdir(startPath, 'Select Folder Containing Preprocessed Results');
if isequal(preprocessedPath, 0), disp('Cancelled.'); return; end

[parentFolder, ~] = fileparts(preprocessedPath);
plotOutputPath = fullfile(parentFolder, 'UKF_RLS_Analysis_Plots'); 

if ~isfolder(plotOutputPath)
    mkdir(plotOutputPath);
    fprintf('Created output directory: %s\n', plotOutputPath);
else
    fprintf('Output directory exists: %s\n', plotOutputPath);
end

resultFiles = dir(fullfile(preprocessedPath, 'preprocessed_estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No preprocessed files found in: %s', preprocessedPath); end

fprintf('Found %d files. Aggregating results (Trim: %.1fs)...\n', numFiles, TRIM_SECONDS);

%% --- 2. Initialize Results Aggregator ---
results = struct('filename', {}, ...
                 'time', {}, ...
                 'features', {}, ...
                 ... % RLS Metrics
                 'rmse_per_dof_rls', {}, 'wrench_error_rls', {}, ...
                 ... % UKF Metrics
                 'rmse_per_dof_ukf', {}, 'wrench_error_ukf', {}, ...
                 ... % Nominal Metrics
                 'rmse_per_dof_nom', {}, 'wrench_error_nom', {});
               
dof_labels = {'F_x (N)', 'F_y (N)', 'F_z (N)', '\tau_x (Nm)', '\tau_y (Nm)', '\tau_z (Nm)'};
dof_labels_short = {'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'};

%% --- 3. Process Each File ---
processedCount = 0;
for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(preprocessedPath, currentFileName);
    
    try
        S = load(fullFilePath);
        
        % Validate Keys
        if ~isfield(S, 'Real_Wrench') || ~isfield(S, 'Predicted_Wrench_UKF') || ...
           ~isfield(S, 'Predicted_Wrench_RLS') || ~isfield(S, 'Nom_Predicted_Wrench')
            fprintf('  [SKIP] Missing wrench data in %s\n', currentFileName);
            continue;
        end
        
        % --- Align Time & Trim ---
        % Find intersection of all valid times
        t_start_raw = max([S.Real_Wrench.Time(1), S.Predicted_Wrench_UKF.Time(1), ...
                           S.Predicted_Wrench_RLS.Time(1)]);
        t_end_raw = min([S.Real_Wrench.Time(end), S.Predicted_Wrench_UKF.Time(end), ...
                         S.Predicted_Wrench_RLS.Time(end)]);
        
        t_trim_start = t_start_raw + TRIM_SECONDS;
        t_trim_end = t_end_raw - TRIM_SECONDS;
        
        if t_trim_end <= t_trim_start
            continue; % Too short
        end
        
        % Create Common Time Vector
        raw_time_vec = S.Real_Wrench.Time;
        time_common = raw_time_vec(raw_time_vec >= t_trim_start & raw_time_vec <= t_trim_end);
        
        if length(time_common) < 10, continue; end
        
        % --- Resample All Data ---
        real_data = resample(S.Real_Wrench, time_common).Data;
        nom_data  = resample(S.Nom_Predicted_Wrench, time_common).Data;
        ukf_data  = resample(S.Predicted_Wrench_UKF, time_common).Data;
        rls_data  = resample(S.Predicted_Wrench_RLS, time_common).Data;
        
        % --- Calculate Errors ---
        err_nom = real_data - nom_data;
        err_ukf = real_data - ukf_data;
        err_rls = real_data - rls_data;
        
        % --- Store Results ---
        processedCount = processedCount + 1;
        results(processedCount).filename = currentFileName;
        results(processedCount).time = time_common;
        results(processedCount).features = S.features_i;
        
        results(processedCount).wrench_error_nom = err_nom;
        results(processedCount).rmse_per_dof_nom = rms(err_nom, 1);
        
        results(processedCount).wrench_error_ukf = err_ukf;
        results(processedCount).rmse_per_dof_ukf = rms(err_ukf, 1);
        
        results(processedCount).wrench_error_rls = err_rls;
        results(processedCount).rmse_per_dof_rls = rms(err_rls, 1);
        
    catch ME
        fprintf('  [ERROR] %s: %s\n', currentFileName, ME.message);
    end
end
results = results(1:processedCount);
numFiles = processedCount;
fprintf('Aggregation Complete. Processed %d files.\n', processedCount);

%% --- 4. Plotting Data Preparation ---
fprintf('Preparing aggregated matrices...\n');

% Flatten RMSE arrays for boxplots [NumFiles x 6]
all_rmse_nom = vertcat(results.rmse_per_dof_nom);
all_rmse_ukf = vertcat(results.rmse_per_dof_ukf);
all_rmse_rls = vertcat(results.rmse_per_dof_rls);

% Prepare Temporal Arrays (Interpolated to common relative time)
all_time_cell = {results.time};
t_durations = cellfun(@(t) t(end) - t(1), all_time_cell);
min_duration = min(t_durations);
t_step = 0.05; % Fixed step for plotting resolution
master_time = (0:t_step:min_duration)'; 
nTimeSteps = length(master_time);

all_err_interp_nom = zeros(nTimeSteps, 6, numFiles);
all_err_interp_ukf = zeros(nTimeSteps, 6, numFiles);
all_err_interp_rls = zeros(nTimeSteps, 6, numFiles);

for i = 1:numFiles
    t_shifted = results(i).time - results(i).time(1);
    all_err_interp_nom(:,:,i) = interp1(t_shifted, results(i).wrench_error_nom, master_time, 'linear', NaN);
    all_err_interp_ukf(:,:,i) = interp1(t_shifted, results(i).wrench_error_ukf, master_time, 'linear', NaN);
    all_err_interp_rls(:,:,i) = interp1(t_shifted, results(i).wrench_error_rls, master_time, 'linear', NaN);
end

% --- Outlier Detection (Based on RLS Performance) ---
total_rmse_rls = vecnorm(all_rmse_rls, 2, 2);
q25 = prctile(total_rmse_rls, 25);
q75 = prctile(total_rmse_rls, 75);
upper_fence = q75 + 1.5 * (q75 - q25);
outlier_indices = find(total_rmse_rls > upper_fence);

fprintf('\nOutlier Analysis (Based on RLS RMSE):\n');
if isempty(outlier_indices)
    fprintf('  No significant outliers detected.\n');
else
    fprintf('  %d Outliers detected (RMSE > %.4f)\n', length(outlier_indices), upper_fence);
end

%% --- 5. PLOTTING ---

% === Figure 1: Single Run Example ===
try
    % Pick the median performing run (based on RLS)
    [~, sortIdx] = sort(total_rmse_rls);
    medianIdx = sortIdx(floor(length(sortIdx)/2));
    
    fileToLoad = fullfile(preprocessedPath, results(medianIdx).filename);
    S1 = load(fileToLoad);
    
    % Re-trim strictly for plotting
    raw_t = S1.Real_Wrench.Time;
    t_bounds = [raw_t(1)+TRIM_SECONDS, raw_t(end)-TRIM_SECONDS];
    mask = (raw_t >= t_bounds(1)) & (raw_t <= t_bounds(2));
    t_p = raw_t(mask);
    
    d_real = resample(S1.Real_Wrench, t_p).Data;
    d_nom  = resample(S1.Nom_Predicted_Wrench, t_p).Data;
    d_ukf  = resample(S1.Predicted_Wrench_UKF, t_p).Data;
    d_rls  = resample(S1.Predicted_Wrench_RLS, t_p).Data;
    
    fig1 = figure('Name', 'Fig 1: Wrench Tracking (Median Run)', 'Position', [100 100 1200 800]);
    tl = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    for k = 1:6
        nexttile;
        plot(t_p, d_real(:,k), 'k-', 'LineWidth', 2, 'DisplayName', 'True'); hold on;
        plot(t_p, d_nom(:,k), '--', 'Color', c.nom, 'LineWidth', 1.5, 'DisplayName', 'Nominal');
        plot(t_p, d_ukf(:,k), '-', 'Color', c.ukf, 'LineWidth', 1.5, 'DisplayName', 'UKF');
        plot(t_p, d_rls(:,k), '-', 'Color', c.rls, 'LineWidth', 1.5, 'DisplayName', 'RLS');
        
        title(dof_labels{k}); grid on; xlim([t_p(1) t_p(end)]);
        if k == 1, legend('Location', 'best'); end
    end
    title(tl, sprintf('Single Run Tracking (File: %s)', results(medianIdx).filename), 'Interpreter', 'none');
    saveas(fig1, fullfile(plotOutputPath, 'Fig1_Tracking_Example.png'));
catch ME
    fprintf('Error plotting Fig 1: %s\n', ME.message);
end

% === Figure 2: Error Evolution (Medians) ===
try
    med_err_nom = median(abs(all_err_interp_nom), 3, 'omitnan');
    med_err_ukf = median(abs(all_err_interp_ukf), 3, 'omitnan');
    med_err_rls = median(abs(all_err_interp_rls), 3, 'omitnan');
    
    fig2 = figure('Name', 'Fig 2: Median Absolute Error', 'Position', [150 150 1200 800]);
    tl2 = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    for k = 1:6
        nexttile;
        plot(master_time, med_err_nom(:,k), 'Color', c.nom, 'LineWidth', 1.5); hold on;
        plot(master_time, med_err_ukf(:,k), 'Color', c.ukf, 'LineWidth', 2);
        plot(master_time, med_err_rls(:,k), 'Color', c.rls, 'LineWidth', 2);
        
        title(dof_labels{k}); grid on; ylabel('Abs Error');
        if k > 3, xlabel('Relative Time (s)'); end
        if k == 1, legend('Nominal', 'UKF', 'RLS'); end
    end
    title(tl2, 'Median Absolute Error across all Runs');
    saveas(fig2, fullfile(plotOutputPath, 'Fig2_Error_Evolution.png'));
catch ME
    fprintf('Error plotting Fig 2: %s\n', ME.message);
end

% === Figure 3: RMSE Distribution (Side-by-Side, 99th Percentile Crop) ===
try
    fig3 = figure('Name', 'Fig 3: RMSE Comparison Side-by-Side', 'Position', [100 100 1400 600]);
    t3 = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    offsets = [-0.2, 0, 0.2]; 
    
    % --- Helper: 99th Percentile Cap ---
    % This ignores the top 1% of extreme errors to ensure the boxes are visible.
    get_smart_ylim = @(d) prctile(d, 99.5) * 1.1; 

    % =====================================================================
    % LEFT TILE: FORCES (Indices 1, 2, 3)
    % =====================================================================
    nexttile; hold on;
    
    % Collect all Force data
    flat_F = [all_rmse_nom(:,1:3); all_rmse_ukf(:,1:3); all_rmse_rls(:,1:3)];
    y_lim_F = get_smart_ylim(flat_F(:));
    
    for i = 1:3 
        boxplot(all_rmse_nom(:,i), 'Positions', i + offsets(1), 'Colors', c.nom, 'Widths', 0.15, 'Symbol', '.');
        boxplot(all_rmse_ukf(:,i), 'Positions', i + offsets(2), 'Colors', c.ukf, 'Widths', 0.15, 'Symbol', '.');
        boxplot(all_rmse_rls(:,i), 'Positions', i + offsets(3), 'Colors', c.rls, 'Widths', 0.15, 'Symbol', '.');
    end
    
    title('Force Estimation Error (RMSE)'); 
    ylabel('RMSE (N)'); 
    set(gca, 'XTick', 1:3, 'XTickLabel', {'F_x', 'F_y', 'F_z'});
    grid on; xlim([0.5 3.5]);
    
    % Force the limit
    ylim([0, y_lim_F]); 
    fprintf('Force Plot Y-Limit set to: %.4f N (99.5th percentile)\n', y_lim_F);

    % Dummy Legend
    h1 = plot(NaN,NaN,'Color',c.nom,'LineWidth',2);
    h2 = plot(NaN,NaN,'Color',c.ukf,'LineWidth',2);
    h3 = plot(NaN,NaN,'Color',c.rls,'LineWidth',2);
    legend([h1 h2 h3], 'Nominal', 'UKF', 'RLS', 'Location', 'best');
    
    % =====================================================================
    % RIGHT TILE: TORQUES (Indices 4, 5, 6)
    % =====================================================================
    nexttile; hold on;
    
    % Collect all Torque data
    flat_T = [all_rmse_nom(:,4:6); all_rmse_ukf(:,4:6); all_rmse_rls(:,4:6)];
    y_lim_T = get_smart_ylim(flat_T(:));

    for i = 1:3 
        idx = i + 3; % 4,5,6
        boxplot(all_rmse_nom(:,idx), 'Positions', i + offsets(1), 'Colors', c.nom, 'Widths', 0.15, 'Symbol', '.');
        boxplot(all_rmse_ukf(:,idx), 'Positions', i + offsets(2), 'Colors', c.ukf, 'Widths', 0.15, 'Symbol', '.');
        boxplot(all_rmse_rls(:,idx), 'Positions', i + offsets(3), 'Colors', c.rls, 'Widths', 0.15, 'Symbol', '.');
    end
    
    title('Torque Estimation Error (RMSE)'); 
    ylabel('RMSE (Nm)'); 
    set(gca, 'XTick', 1:3, 'XTickLabel', {'\tau_x', '\tau_y', '\tau_z'});
    grid on; xlim([0.5 3.5]);
    
    % Force the limit
    ylim([0, y_lim_T]); 
    fprintf('Torque Plot Y-Limit set to: %.4f Nm (99.5th percentile)\n', y_lim_T);
    
    title(t3, 'Estimation Error Distribution (Zoomed to 99.5%)');
    saveas(fig3, fullfile(plotOutputPath, 'Fig3_RMSE_Split_Boxplots.png'));
    
catch ME
    fprintf('Error plotting Fig 3: %s\n', ME.message);
end
% === Figure 4: Variation vs Performance (Scatter with Trendlines) ===
% RESTORED: Trendlines added
try
    % Calculate normalized feature variation score
    all_features = [results.features];
    feat_fields = fieldnames(all_features);
    
    % Extract scalar deviations
    raw_feats = [];
    for k = 1:length(feat_fields)
        fn = feat_fields{k};
        % Handle nested struct or simple value
        if isstruct(all_features(1).(fn)) && isfield(all_features(1).(fn), 'mean_deviation')
             vals = [all_features.(fn)];
             raw_feats = [raw_feats, [vals.mean_deviation]']; %#ok<AGROW>
        end
    end
    
    if ~isempty(raw_feats)
        % Z-Score normalize and sum
        z_feats = normalize(raw_feats);
        total_disturbance = sum(abs(z_feats), 2);
        
        tot_rmse_nom = vecnorm(all_rmse_nom, 2, 2);
        tot_rmse_ukf = vecnorm(all_rmse_ukf, 2, 2);
        tot_rmse_rls = vecnorm(all_rmse_rls, 2, 2);
        
        fig4 = figure('Name', 'Fig 4: Robustness', 'Position', [250 250 900 600]);
        hold on;
        
        % 1. Plot Points
        s1 = scatter(total_disturbance, tot_rmse_nom, 25, 's', 'MarkerEdgeColor', c.nom, 'DisplayName', 'Nominal');
        s2 = scatter(total_disturbance, tot_rmse_ukf, 30, '^', 'MarkerFaceColor', c.ukf, 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'UKF');
        s3 = scatter(total_disturbance, tot_rmse_rls, 30, 'o', 'MarkerFaceColor', c.rls, 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'RLS');
        
        % 2. Calculate & Plot Trendlines
        p_nom = polyfit(total_disturbance, tot_rmse_nom, 1);
        p_ukf = polyfit(total_disturbance, tot_rmse_ukf, 1);
        p_rls = polyfit(total_disturbance, tot_rmse_rls, 1);
        
        x_fit = linspace(min(total_disturbance), max(total_disturbance), 100);
        
        plot(x_fit, polyval(p_nom, x_fit), '--', 'Color', c.nom, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        plot(x_fit, polyval(p_ukf, x_fit), '-', 'Color', c.ukf, 'LineWidth', 2, 'HandleVisibility', 'off');
        plot(x_fit, polyval(p_rls, x_fit), '-', 'Color', c.rls, 'LineWidth', 2, 'HandleVisibility', 'off');

        % 3. Highlight Outliers
        if ~isempty(outlier_indices)
            plot(total_disturbance(outlier_indices), tot_rmse_rls(outlier_indices), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Outliers (RLS)');
        end
        
        xlabel('Total Parameter Disturbance (Norm. Z-Score)');
        ylabel('Total RMSE');
        title('Estimator Robustness: Error vs. Parameter Variation');
        grid on; 
        legend('Location', 'best');
        
        saveas(fig4, fullfile(plotOutputPath, 'Fig4_Robustness.png'));
    end
catch ME
    fprintf('Error plotting Fig 4: %s\n', ME.message);
end