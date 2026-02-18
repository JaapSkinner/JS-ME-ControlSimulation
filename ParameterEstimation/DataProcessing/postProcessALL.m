%% ANALYZE PREPROCESSED SIMULATION DATA (UKF vs RLS vs Nominal)
% Updated: Includes Metric Generation and Scaling Correction (Linear / 0.9)
clear; clc; close all;

%% --- 0. PUBLICATION QUALITY SETTINGS ---
set(groot, 'DefaultAxesTickLabelInterpreter', 'tex');
set(groot, 'DefaultLegendInterpreter', 'tex');
set(groot, 'DefaultTextInterpreter', 'tex');
set(groot, 'DefaultAxesFontSize', 14);          
set(groot, 'DefaultAxesLabelFontSizeMultiplier', 1.1); 
set(groot, 'DefaultAxesLineWidth', 1.5);        
set(groot, 'DefaultLineLineWidth', 2.0);        
set(groot, 'DefaultScatterLineWidth', 1.5);     
set(groot, 'DefaultFigureColor', 'w');          
set(groot, 'DefaultAxesFontName', 'Arial');     

%% --- CONFIGURATION ---
TRIM_SECONDS = 5.0; 
% Plot Colors
c.nom = [0.6350 0.0780 0.1840]; 
c.ukf = [0.0000 0.4470 0.7410]; 
c.rls = [0.4660 0.6740 0.1880]; 
c.true = [0 0 0];               
c.outlier = [1 0 0];            

%% --- 1. Select Preprocessed Folder ---
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
end

resultFiles = dir(fullfile(preprocessedPath, 'preprocessed_estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No preprocessed files found in: %s', preprocessedPath); end
fprintf('Found %d files. Aggregating results (Trim: %.1fs)...\n', numFiles, TRIM_SECONDS);

%% --- 2. Initialize Results Aggregator ---
results = struct('filename', {}, 'time', {}, 'features', {}, ...
                 'rmse_per_dof_rls', {}, 'wrench_error_rls', {}, ...
                 'rmse_per_dof_ukf', {}, 'wrench_error_ukf', {}, ...
                 'rmse_per_dof_nom', {}, 'wrench_error_nom', {});
               
dof_labels = {'a_x', 'a_y', 'a_z', '\alpha_x', '\alpha_y', '\alpha_z'};

%% --- 3. Process Each File ---
processedCount = 0;
for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(preprocessedPath, currentFileName);
    try
        S = load(fullFilePath);
        % Validate Keys
        if ~isfield(S, 'Real_Wrench') || ~isfield(S, 'Predicted_Wrench_UKF')
            continue;
        end
        
        % --- Align Time & Trim ---
        t_start_raw = max([S.Real_Wrench.Time(1), S.Predicted_Wrench_UKF.Time(1), S.Predicted_Wrench_RLS.Time(1)]);
        t_end_raw = min([S.Real_Wrench.Time(end), S.Predicted_Wrench_UKF.Time(end), S.Predicted_Wrench_RLS.Time(end)]);
        
        t_trim_start = t_start_raw + TRIM_SECONDS;
        t_trim_end = t_end_raw - TRIM_SECONDS;
        
        if t_trim_end <= t_trim_start, continue; end
        
        raw_time_vec = S.Real_Wrench.Time;
        time_common = raw_time_vec(raw_time_vec >= t_trim_start & raw_time_vec <= t_trim_end);
        
        if length(time_common) < 10, continue; end
        
        % --- Resample ---
        real_data = resample(S.Real_Wrench, time_common).Data;
        nom_data  = resample(S.Nom_Predicted_Wrench, time_common).Data;
        ukf_data  = resample(S.Predicted_Wrench_UKF, time_common).Data;
        rls_data  = resample(S.Predicted_Wrench_RLS, time_common).Data;
        
        % --- SCALING CORRECTION ---
        % Scale Linear Thrust (Cols 1-3) by dividing by 0.9.
        % Angular (Cols 4-6) remain multiplied by 1.
        scale_vec = [1, 1, 1, 1, 1, 1];
        
        real_data = real_data .* scale_vec;
        nom_data  = nom_data  .* scale_vec;
        ukf_data  = ukf_data  .* scale_vec;
        rls_data  = rls_data  .* scale_vec;

        % --- Calculate Errors ---
        processedCount = processedCount + 1;
        results(processedCount).filename = currentFileName;
        results(processedCount).time = time_common;
        results(processedCount).features = S.features_i;
        
        results(processedCount).wrench_error_nom = real_data - nom_data;
        results(processedCount).rmse_per_dof_nom = rms(real_data - nom_data, 1);
        
        results(processedCount).wrench_error_ukf = real_data - ukf_data;
        results(processedCount).rmse_per_dof_ukf = rms(real_data - ukf_data, 1);
        
        results(processedCount).wrench_error_rls = real_data - rls_data;
        results(processedCount).rmse_per_dof_rls = rms(real_data - rls_data, 1);
        
    catch ME
        fprintf('  [ERROR] %s: %s\n', currentFileName, ME.message);
    end
end
results = results(1:processedCount);
numFiles = processedCount;
fprintf('Aggregation Complete. Processed %d files.\n', processedCount);

%% --- 4. Data Preparation & Disturbance Extraction ---
all_rmse_nom = vertcat(results.rmse_per_dof_nom);
all_rmse_ukf = vertcat(results.rmse_per_dof_ukf);
all_rmse_rls = vertcat(results.rmse_per_dof_rls);

% Calculate Totals (Euclidean norm across all 6 DOFs)
tot_rmse_nom = vecnorm(all_rmse_nom, 2, 2);
tot_rmse_ukf = vecnorm(all_rmse_ukf, 2, 2);
tot_rmse_rls = vecnorm(all_rmse_rls, 2, 2);

% Extract Parameter Disturbance (Z-Score Sum)
all_features = [results.features];
feat_fields = fieldnames(all_features);
raw_feats = [];
for k = 1:length(feat_fields)
    fn = feat_fields{k};
    if isstruct(all_features(1).(fn)) && isfield(all_features(1).(fn), 'mean_deviation')
         vals = [all_features.(fn)];
         raw_feats = [raw_feats, [vals.mean_deviation]']; 
    end
end

if ~isempty(raw_feats)
    z_feats = normalize(raw_feats);
    total_disturbance = sum(abs(z_feats), 2);
else
    total_disturbance = zeros(numFiles, 1);
    warning('No disturbance features found.');
end

% Temporal Interpolation
t_step = 0.05; 
min_duration = min(cellfun(@(t) t(end)-t(1), {results.time}));
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

% Outlier Detection (for plotting exclusion only)
q75 = prctile(tot_rmse_rls, 75);
upper_fence = q75 + 1.5 * (q75 - prctile(tot_rmse_rls, 25));
outlier_indices = find(tot_rmse_rls > upper_fence);

%% --- 5. PLOTTING ---

% === Figure 1: Single Run Example ===
try
    [~, sortIdx] = sort(tot_rmse_rls);
    medianIdx = sortIdx(floor(length(sortIdx)/2));
    
    % Access scaled data from results structure directly to ensure consistency
    fileToLoad = fullfile(preprocessedPath, results(medianIdx).filename);
    S1 = load(fileToLoad);
    
    raw_t = S1.Real_Wrench.Time;
    mask = (raw_t >= raw_t(1)+TRIM_SECONDS) & (raw_t <= raw_t(end)-TRIM_SECONDS);
    t_p = raw_t(mask);
    
    d_real = resample(S1.Real_Wrench, t_p).Data;
    d_nom  = resample(S1.Nom_Predicted_Wrench, t_p).Data;
    d_ukf  = resample(S1.Predicted_Wrench_UKF, t_p).Data;
    d_rls  = resample(S1.Predicted_Wrench_RLS, t_p).Data;
    
    % RE-APPLY SCALING FOR PLOT
    scale_vec = [1/0.98, 1/0.98, 1/0.98, 1, 1, 1];
    d_real = d_real .* scale_vec;
    d_nom  = d_nom  .* scale_vec;
    d_ukf  = d_ukf  .* scale_vec;
    d_rls  = d_rls  .* scale_vec;
    
    fig1 = figure('Name', 'Fig 1', 'Position', [100 100 1200 800]); 
    tl = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    % Define labels locally to ensure they are clean for titles
    local_labels = {'a_x', 'a_y', 'a_z', '\alpha_x', '\alpha_y', '\alpha_z'};
    
    for k = 1:6
        nexttile;
        plot(t_p, d_real(:,k), 'k-', 'DisplayName', 'True'); hold on;
        plot(t_p, d_nom(:,k), '--', 'Color', c.nom, 'DisplayName', 'Nominal');
        plot(t_p, d_ukf(:,k), '-', 'Color', c.ukf, 'DisplayName', 'UKF');
        plot(t_p, d_rls(:,k), '-', 'Color', c.rls, 'DisplayName', 'RLS');
        
        % Title with bold injection
        title(['\bf' local_labels{k}], 'Interpreter', 'tex', 'FontSize', 12); 
        grid on; xlim([t_p(1) t_p(end)]);
        
        % --- AXIS LABELS ---
        % Add Time label to bottom row only (indices 4, 5, 6)
        if k > 3
            xlabel('Time (s)', 'FontWeight', 'bold');
        end
        
        % Add Y-axis labels to left column only (indices 1, 4)
        if k == 1
            ylabel('Linear (m/s^2)', 'FontWeight', 'bold');
        elseif k == 4
            ylabel('Angular (rad/s^2)', 'FontWeight', 'bold');
        end
        
        % Legend only on the first plot
        if k == 1
            legend('Location', 'best', 'FontSize', 10); 
        end
    end
    
    title(tl, 'Single Run Tracking (Median Performance)', 'FontSize', 16, 'FontWeight', 'bold');
    exportgraphics(fig1, fullfile(plotOutputPath, 'Fig1_Tracking_Example.png'), 'Resolution', 300);
catch ME
    fprintf('Error Fig 1: %s\n', ME.message);
end

% === Figure 2: Error Evolution ===
try
    med_err_nom = median(abs(all_err_interp_nom), 3, 'omitnan');
    med_err_ukf = median(abs(all_err_interp_ukf), 3, 'omitnan');
    med_err_rls = median(abs(all_err_interp_rls), 3, 'omitnan');
    
    fig2 = figure('Name', 'Fig 2', 'Position', [150 150 1200 800]);
    tl2 = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    for k = 1:6
        nexttile;
        plot(master_time, med_err_nom(:,k), 'Color', c.nom); hold on;
        plot(master_time, med_err_ukf(:,k), 'Color', c.ukf);
        plot(master_time, med_err_rls(:,k), 'Color', c.rls);
        title(dof_labels{k}, 'FontWeight', 'bold'); grid on; ylabel('Abs Error');
        if k > 3, xlabel('Relative Time (s)'); end
        if k == 1, legend('Nominal', 'UKF', 'RLS', 'FontSize', 10); end
    end
    title(tl2, 'Median Absolute Error Evolution', 'FontSize', 16, 'FontWeight', 'bold');
    exportgraphics(fig2, fullfile(plotOutputPath, 'Fig2_Error_Evolution.png'), 'Resolution', 300);
catch ME
    fprintf('Error Fig 2: %s\n', ME.message);
end
% === Figure 3: RMSE Distribution (Side-by-Side) ===
try
    fig3 = figure('Name', 'Fig 3', 'Position', [100 100 1400 700]);
    t3 = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    offsets = [-0.2, 0, 0.2]; 
    
    % UPDATED: Use max() instead of prctile() to ensure the top is never cut off
    get_smart_ylim = @(d) max(d(:)) * 1.05; 
    
    boxLineWidth = 2.0; % Line thickness
    
    % --- Left Plot: Linear ---
    nexttile; hold on;
    
    % 1. Invisible Dummy Plots for Legend
    hNom = plot(NaN, NaN, '-', 'Color', c.nom, 'LineWidth', boxLineWidth, 'DisplayName', 'Nominal');
    hUKF = plot(NaN, NaN, '-', 'Color', c.ukf, 'LineWidth', boxLineWidth, 'DisplayName', 'UKF');
    hRLS = plot(NaN, NaN, '-', 'Color', c.rls, 'LineWidth', boxLineWidth, 'DisplayName', 'RLS');
    
    % 2. Plot Boxes
    flat_F = [all_rmse_nom(:,1:3); all_rmse_ukf(:,1:3); all_rmse_rls(:,1:3)];
    y_lim_F = get_smart_ylim(flat_F); % Uses max value now
    
    for i = 1:3 
        h = boxplot(all_rmse_nom(:,i), 'Positions', i + offsets(1), 'Colors', c.nom, 'Widths', 0.15, 'Symbol', '.'); set(h, 'LineWidth', boxLineWidth);
        h = boxplot(all_rmse_ukf(:,i), 'Positions', i + offsets(2), 'Colors', c.ukf, 'Widths', 0.15, 'Symbol', '.'); set(h, 'LineWidth', boxLineWidth);
        h = boxplot(all_rmse_rls(:,i), 'Positions', i + offsets(3), 'Colors', c.rls, 'Widths', 0.15, 'Symbol', '.'); set(h, 'LineWidth', boxLineWidth);
    end
    
    % 3. Labels & formatting
    title('Linear (a)', 'FontWeight', 'bold'); 
    ylabel('RMSE (m/s^2)', 'FontWeight', 'bold'); 
    set(gca, 'XTick', 1:3, 'XTickLabel', {'\bf x', '\bf y', '\bf z'}, ...
        'TickLabelInterpreter', 'tex', 'FontSize', 12, 'FontWeight', 'bold');
    grid on; xlim([0.5 3.5]); ylim([0, y_lim_F]); 
    
    % 4. Legend (Top Left)
    legend([hNom, hUKF, hRLS], 'Location', 'northwest', 'FontSize', 11);

    % --- Right Plot: Angular ---
    nexttile; hold on;
    flat_T = [all_rmse_nom(:,4:6); all_rmse_ukf(:,4:6); all_rmse_rls(:,4:6)];
    y_lim_T = get_smart_ylim(flat_T); % Uses max value now
    
    for i = 1:3 
        idx = i + 3;
        h = boxplot(all_rmse_nom(:,idx), 'Positions', i + offsets(1), 'Colors', c.nom, 'Widths', 0.15, 'Symbol', '.'); set(h, 'LineWidth', boxLineWidth);
        h = boxplot(all_rmse_ukf(:,idx), 'Positions', i + offsets(2), 'Colors', c.ukf, 'Widths', 0.15, 'Symbol', '.'); set(h, 'LineWidth', boxLineWidth);
        h = boxplot(all_rmse_rls(:,idx), 'Positions', i + offsets(3), 'Colors', c.rls, 'Widths', 0.15, 'Symbol', '.'); set(h, 'LineWidth', boxLineWidth);
    end
    
    title('Angular (\alpha)', 'FontWeight', 'bold'); 
    ylabel('RMSE (rad/s^2)', 'FontWeight', 'bold'); 
    
    set(gca, 'XTick', 1:3, 'XTickLabel', {'\bf\alpha_x', '\bf\alpha_y', '\bf\alpha_z'}, ...
        'TickLabelInterpreter', 'tex', 'FontSize', 12, 'FontWeight', 'bold');
        
    grid on; xlim([0.5 3.5]); ylim([0, y_lim_T]);
    
    title(t3, 'Estimation Error Distribution', 'FontSize', 16, 'FontWeight', 'bold');
    exportgraphics(fig3, fullfile(plotOutputPath, 'Fig3_RMSE_Split_Boxplots.png'), 'Resolution', 300);
catch ME
    fprintf('Error plotting Fig 3: %s\n', ME.message);
end
% === Figure 4: Robustness ===
try
    if ~isempty(raw_feats)
        fig4 = figure('Name', 'Fig 4', 'Position', [250 250 900 600]);
        hold on;
        scatter(total_disturbance, tot_rmse_nom, 50, 's', 'MarkerEdgeColor', c.nom, 'LineWidth', 1.5);
        scatter(total_disturbance, tot_rmse_ukf, 50, '^', 'MarkerFaceColor', c.ukf, 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.6);
        scatter(total_disturbance, tot_rmse_rls, 50, 'o', 'MarkerFaceColor', c.rls, 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.6);
        
        p_nom = polyfit(total_disturbance, tot_rmse_nom, 1);
        p_ukf = polyfit(total_disturbance, tot_rmse_ukf, 1);
        p_rls = polyfit(total_disturbance, tot_rmse_rls, 1);
        x_fit = linspace(min(total_disturbance), max(total_disturbance), 100);
        
        plot(x_fit, polyval(p_nom, x_fit), '--', 'Color', c.nom);
        plot(x_fit, polyval(p_ukf, x_fit), '-', 'Color', c.ukf);
        plot(x_fit, polyval(p_rls, x_fit), '-', 'Color', c.rls);
        
        xlabel('Total Parameter Disturbance (Z-Score)', 'FontSize', 14, 'FontWeight', 'bold');
        ylabel('Total RMSE', 'FontSize', 14, 'FontWeight', 'bold');
        title('Estimator Robustness', 'FontSize', 16, 'FontWeight', 'bold');
        grid on; 
        legend('Nominal','UKF','RLS','Location','best','FontSize',12);
        exportgraphics(fig4, fullfile(plotOutputPath, 'Fig4_Robustness.png'), 'Resolution', 300);
    end
catch ME
    fprintf('Error Fig 4: %s\n', ME.message);
end

%% --- 6. THESIS METRICS GENERATION ---
fprintf('\n--- GENERATING THESIS METRICS ---\n');

% Open file for writing
metricsFile = fullfile(plotOutputPath, 'Thesis_Metrics_Report.txt');
fid = fopen(metricsFile, 'w');

% Helper to print to both command window and file
printMe = @(fmt, varargin) multiPrint(fid, fmt, varargin{:});

printMe('========================================================\n');
printMe('QUANTITATIVE ANALYSIS REPORT (SCALED)\n');
printMe('Note: Linear Thrust components were scaled by 1/0.9\n');
printMe('Date: %s\n', datestr(now));
printMe('Number of Trials: %d\n', numFiles);
printMe('========================================================\n\n');

% --- 6.1 TOTAL RMSE STATISTICS ---
printMe('1. GLOBAL PERFORMANCE (Total RMSE across all DOFs)\n');
printMe('--------------------------------------------------------\n');
printMe('%-10s | %-12s | %-12s | %-12s | %-12s\n', 'Estimator', 'Mean', 'Std Dev', 'Median', 'IQR');
printMe('--------------------------------------------------------\n');

stats = @(x) [mean(x), std(x), median(x), iqr(x)];
s_nom = stats(tot_rmse_nom);
s_ukf = stats(tot_rmse_ukf);
s_rls = stats(tot_rmse_rls);

printMe('%-10s | %6.4f       | %6.4f       | %6.4f       | %6.4f\n', 'Nominal', s_nom);
printMe('%-10s | %6.4f       | %6.4f       | %6.4f       | %6.4f\n', 'UKF', s_ukf);
printMe('%-10s | %6.4f       | %6.4f       | %6.4f       | %6.4f\n', 'RLS', s_rls);
printMe('--------------------------------------------------------\n');

% Calculate Improvements (Mean Reduction)
imp_ukf = (s_nom(1) - s_ukf(1)) / s_nom(1) * 100;
imp_rls = (s_nom(1) - s_rls(1)) / s_nom(1) * 100;

printMe('Average Improvement over Nominal:\n');
printMe('  UKF: %.2f%%\n', imp_ukf);
printMe('  RLS: %.2f%%\n', imp_rls);
printMe('\n');

% --- 6.2 ROBUSTNESS SENSITIVITY (Slopes) ---
if ~isempty(total_disturbance)
    printMe('2. ROBUSTNESS ANALYSIS (Linear Fit: RMSE = m*Disturbance + c)\n');
    printMe('   "Slope (m)" indicates sensitivity to parameter error.\n');
    printMe('   "R-Squared" indicates predictability.\n');
    printMe('--------------------------------------------------------\n');
    printMe('%-10s | %-12s | %-12s | %-12s\n', 'Estimator', 'Slope (m)', 'Intercept (c)', 'R-Squared');
    printMe('--------------------------------------------------------\n');
    
    calc_robust = @(x,y) [polyfit(x,y,1), corrcoef(x,y).^2];
    
    % Polyfit returns [slope, intercept]. corrcoef returns matrix, take (1,2) squared.
    r_nom_vals = polyfit(total_disturbance, tot_rmse_nom, 1);
    r2_nom = corrcoef(total_disturbance, tot_rmse_nom); r2_nom = r2_nom(1,2)^2;
    
    r_ukf_vals = polyfit(total_disturbance, tot_rmse_ukf, 1);
    r2_ukf = corrcoef(total_disturbance, tot_rmse_ukf); r2_ukf = r2_ukf(1,2)^2;
    
    r_rls_vals = polyfit(total_disturbance, tot_rmse_rls, 1);
    r2_rls = corrcoef(total_disturbance, tot_rmse_rls); r2_rls = r2_rls(1,2)^2;
    
    printMe('%-10s | %6.4f       | %6.4f       | %6.4f\n', 'Nominal', r_nom_vals(1), r_nom_vals(2), r2_nom);
    printMe('%-10s | %6.4f       | %6.4f       | %6.4f\n', 'UKF', r_ukf_vals(1), r_ukf_vals(2), r2_ukf);
    printMe('%-10s | %6.4f       | %6.4f       | %6.4f\n', 'RLS', r_rls_vals(1), r_rls_vals(2), r2_rls);
    printMe('--------------------------------------------------------\n');
    
    % Interpretation
    [~, best_slope_idx] = min(abs([r_nom_vals(1), r_ukf_vals(1), r_rls_vals(1)]));
    names = {'Nominal', 'UKF', 'RLS'};
    printMe('Most Robust Estimator (Lowest Slope): %s\n', names{best_slope_idx});
    printMe('\n');
end

% --- 6.3 DETAILED PER-DOF METRICS ---
printMe('3. DETAILED METRICS PER DEGREE OF FREEDOM (Median RMSE)\n');
printMe('----------------------------------------------------------------------\n');
printMe('%-10s | %-8s | %-8s | %-8s | %-8s | %-8s | %-8s\n', 'Estimator', 'x', 'y', 'z', 'alph_x', 'alph_y', 'alph_z');
printMe('----------------------------------------------------------------------\n');

% Calculate Medians per column
m_nom_dof = median(all_rmse_nom, 1);
m_ukf_dof = median(all_rmse_ukf, 1);
m_rls_dof = median(all_rmse_rls, 1);

printMe('%-10s | %6.3f   | %6.3f   | %6.3f   | %6.3f   | %6.3f   | %6.3f\n', 'Nominal', m_nom_dof);
printMe('%-10s | %6.3f   | %6.3f   | %6.3f   | %6.3f   | %6.3f   | %6.3f\n', 'UKF', m_ukf_dof);
printMe('%-10s | %6.3f   | %6.3f   | %6.3f   | %6.3f   | %6.3f   | %6.3f\n', 'RLS', m_rls_dof);
printMe('----------------------------------------------------------------------\n');

fclose(fid);
fprintf('Metrics report saved to: %s\n', metricsFile);

% --- Local Function for Printing ---
function multiPrint(fid, fmt, varargin)
    % Prints to Command Window
    fprintf(fmt, varargin{:});
    % Prints to File
    if fid > 0
        fprintf(fid, fmt, varargin{:});
    end
end