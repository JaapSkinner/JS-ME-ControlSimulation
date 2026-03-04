%% UAV Mixer Generation & Validation Script
%  1. Loads RLS data via UI.
%  2. Calculates Effectiveness (B) and Normalized Mixer (M).
%  3. PRINTS matrices to the Console.
%  4. VALIDATION: Plots predicted (RLS) vs Nominal vs Real flight data.
%  5. CONVERGENCE: Visualizes parameter evolution.
%  6. RMSE: Calculates error metrics for predictions.
clear; clc; close all;

%% 1. Configuration
sampleTime = 53; % Time to capture the effectiveness matrix
startPath  = fullfile(pwd, 'Results');

% --- VISUALIZATION TOGGLES ---
show_real    = true;   % Set to false to hide 'Real Accel' (Black line)
show_rls     = true;   % Set to false to hide 'RLS Prediction' (Red dashed)
show_nominal = false;  % Set to false to hide 'Nominal/Theoretical' (Purple)

% --- USER DEFINED NOMINAL MATRIX (Example: Standard Quad-X) ---
% Define your theoretical B matrix here.
kt = 1.0;   % Theoretical Thrust Coefficient (N/omega^2)
km = 0.05;  % Theoretical Moment Coefficient (Nm/omega^2)
L  = 0.25;  % Arm Length (m)

B_raw_data = 0.0000001.*[
    -1.64754,  2.03024,  1.60214, -3.82697,  0.00000, -5.25398;  % Motor 0
     1.64754,  2.03024, -1.60214, -3.82697,  0.00000, -5.25398;  % Motor 1
     1.64754, -2.03024,  1.60214,  3.82697,  0.00000, -5.25398;  % Motor 2
    -1.64754, -2.03024, -1.60214,  3.82697,  0.00000, -5.25398;  % Motor 3
     1.64754,  1.64754,  1.60214,  0.00000,  3.82697, -5.25398;  % Motor 4
    -1.64754,  1.64754, -1.60214,  0.00000, -3.82697, -5.25398;  % Motor 5
    -1.64754, -1.64754,  1.60214,  0.00000, -3.82697, -5.25398;  % Motor 6
     1.64754, -1.64754, -1.60214,  0.00000,  3.82697, -5.25398   % Motor 7
];

% 2. Map Columns to Standard [Fx, Fy, Fz, Mx, My, Mz]
col_Fx = B_raw_data(:, 4);
col_Fy = B_raw_data(:, 5);
col_Fz = B_raw_data(:, 6);
col_Mx = 10*B_raw_data(:, 1); 
col_My = 10*B_raw_data(:, 2); 
col_Mz = 10*B_raw_data(:, 3);

% 3. Construct Final Nominal Matrix [6 x N]
B_nominal_def = [
    col_Fx';
    col_Fy';
    col_Fz'; 
    col_Mx';
    col_My';
    col_Mz'
];

%% 2. Load File (UI)
if ~isfolder(startPath), startPath = pwd; end
fprintf('Select simulation file...\n');
[file, path] = uigetfile(fullfile(startPath, '*.mat'), 'Select Data');
if isequal(file, 0), return; end
fullFilePath = fullfile(path, file);
fprintf('Loading: %s ...\n', file);

dataStruct = load(fullFilePath);

% Validate simOut object
if isfield(dataStruct, 'simOut')
    simOutObj = dataStruct.simOut;
else
    error('Variable "simOut" not found.');
end

% Check for RLS Data
if isprop(simOutObj, 'RLSData'), rlsData = simOutObj.RLSData;
else, error('Property "RLSData" not found.'); end

% Check for Validation Data
if isprop(simOutObj, 'omega2'), ts_omega2 = simOutObj.omega2;
else, error('Property "omega2" not found for validation.'); end
if isprop(simOutObj, 'RealAccellerations'), ts_accel = simOutObj.RealAccellerations;
else, error('Property "RealAccellerations" not found for validation.'); end

ts_torque = rlsData.TorqueEffectiveness;
ts_force  = rlsData.ForceEffectiveness;

%% 3. Extract & Form Matrix at Sample Time
% Resample RLS to sample time
t_force  = resample(ts_force, sampleTime);
t_torque = resample(ts_torque, sampleTime);

% Form B Matrix
raw_f = squeeze(t_force.Data);
raw_t = squeeze(t_torque.Data);

% Handle dimensions to ensure [3 x N]
if size(raw_f, 1) ~= 3, raw_f = raw_f'; end
if size(raw_t, 1) ~= 3, raw_t = raw_t'; end

B_Matrix = [raw_f; raw_t]; % 6 x N
[~, n_motors] = size(B_Matrix);

%% 4. Calculate & Normalize Mixer (PX4 Logic)
M_raw = pinv(B_Matrix);
Mixer_Normalized = zeros(size(M_raw));
axis_names = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};

% --- CONSOLE OUTPUTS START ---
fprintf('\n=================================================\n');
fprintf('       UAV MIXER GENERATION (t = %.2fs)\n', sampleTime);
fprintf('=================================================\n');
fprintf('\n--- 1. Effectiveness Matrix (B_RLS) [6 x %d] ---\n', n_motors);
disp(B_Matrix);

fprintf('\n--- 2. Raw Pseudo-Inverse Mixer (M_raw) ---\n');
disp(M_raw);

fprintf('\n--- 3. Normalization Factors ---\n');
for i = 1:6
    col = M_raw(:, i);
    max_val = max(abs(col));
    
    if max_val > 1e-9 
        Mixer_Normalized(:, i) = col / max_val;
        fprintf('Axis %s: Scaled by 1/%.4f (Max input %.4f)\n', axis_names{i}, max_val, max_val);
    else
        Mixer_Normalized(:, i) = col;
        fprintf('Axis %s: Uncontrollable (Max ~ 0)\n', axis_names{i});
    end
end

fprintf('\n--- 4. Final PX4 Normalized Mixer (M) ---\n');
disp(Mixer_Normalized);
fprintf('=================================================\n');
% --- CONSOLE OUTPUTS END ---

%% 5. Validation: Predict Wrenches vs Real Accelerations
fprintf('Running Model Validation across full flight...\n');

% --- A. Prepare Input (Omega^2) ---
target_time = ts_accel.Time;
ts_omega2_res = resample(ts_omega2, target_time);
U_data = ts_omega2_res.Data; 
dims_u = size(U_data);

% Ensure U is [N x Time]
if dims_u(1) ~= n_motors && dims_u(end) == n_motors
    U_data = U_data'; 
elseif dims_u(1) ~= n_motors
    if size(U_data, 1) == length(target_time)
        U_data = U_data';
    end
end

% --- B. Prepare Ground Truth (Real Accelerations) ---
Y_real = ts_accel.Data;
dims_y = size(Y_real);

% Ensure Y is [6 x Time]
if dims_y(1) == length(target_time) && dims_y(2) == 6
    Y_real = Y_real';
end
if length(dims_y) > 2 % Handle [Time x 1 x 6]
    Y_real = squeeze(Y_real)';
end

% --- C. RLS Prediction ---
Y_pred = B_Matrix * U_data;

% --- D. Nominal Prediction ---
[nom_rows, nom_cols] = size(B_nominal_def);
has_nominal = false;
Y_nom = [];

if nom_cols == n_motors && nom_rows == 6
    Y_nom = B_nominal_def * U_data;
    has_nominal = true;
    fprintf('Nominal matrix valid. Calculating nominal trajectory...\n');
else
    warning('Nominal Matrix dimension mismatch! Defined [%dx%d], Expected [6x%d]. Skipping Nominal plot.', nom_rows, nom_cols, n_motors);
end

%% 5b. Error Analysis (RMSE)
fprintf('\n=================================================\n');
fprintf('       PREDICTION ERROR ANALYSIS (RMSE)\n');
fprintf('=================================================\n');

% Initialize arrays
rmse_rls = zeros(1, 6);
rmse_nom = zeros(1, 6);

% Calculate RMSE per axis
for i = 1:6
    % RLS RMSE: sqrt(mean((predicted - real)^2))
    residuals_rls = Y_real(i, :) - Y_pred(i, :);
    rmse_rls(i) = sqrt(mean(residuals_rls.^2));

    % Nominal RMSE (if exists)
    if has_nominal
        residuals_nom = Y_real(i, :) - Y_nom(i, :);
        rmse_nom(i) = sqrt(mean(residuals_nom.^2));
    end
end

% Print Table Header
fprintf('%-6s | %-16s | %-16s\n', 'Axis', 'RLS RMSE', 'Nominal RMSE');
fprintf('-----------------------------------------------\n');

% Print Rows
for i = 1:6
    if has_nominal
        nom_str = sprintf('%.4f', rmse_nom(i));
    else
        nom_str = 'N/A';
    end
    fprintf('%-6s | %-16.4f | %-16s\n', axis_names{i}, rmse_rls(i), nom_str);
end
fprintf('=================================================\n');

%% 6. Visualization (General)
% A. Validation Plot (3x2 Grid)
figure('Name', 'Model Validation: Prediction vs Reality', 'Color', 'w');
y_labels = {'m/s^2', 'm/s^2', 'm/s^2', 'rad/s^2', 'rad/s^2', 'rad/s^2'};
c_purple = [0.6, 0, 0.8]; 

for i = 1:6
    subplot(2, 3, i);
    hold on;
    
    % 1. Plot Real (Flag Checked)
    if show_real
        plot(target_time, Y_real(i, :), 'k', 'LineWidth', 1.0, 'DisplayName', 'Real Accel');
    end
    
    % 2. Plot RLS Predicted (Flag Checked)
    if show_rls
        plot(target_time, Y_pred(i, :), 'r--', 'LineWidth', 1.2, 'DisplayName', 'RLS Pred (B*u)');
    end
    
    % 3. Plot Nominal (Flag Checked)
    if has_nominal && show_nominal
        plot(target_time, Y_nom(i, :), '-.', 'Color', c_purple, 'LineWidth', 1.2, 'DisplayName', 'Nominal (Def)');
    end
    
    % Update Title to include RLS RMSE for quick reference
    title(sprintf('%s (RMSE: %.2f)', axis_names{i}, rmse_rls(i))); 
    ylabel(y_labels{i}); xlabel('Time (s)');
    grid on;
    
    % Only show legend if something was actually plotted
    if i == 1 && (show_real || show_rls || (has_nominal && show_nominal))
        legend('Location', 'best'); 
    end
    
    xline(sampleTime, 'b:', 'LineWidth', 1, 'HandleVisibility', 'off'); % Hide vertical line from legend
end
sgtitle(['Validation: Static B (t=' num2str(sampleTime) 's) vs Full Flight']);

% B. Mixer Heatmap
figure('Name', 'Generated Mixer', 'Color', 'w');
imagesc(Mixer_Normalized);
colormap(jet); clim([-1 1]); c = colorbar;
c.Label.String = 'Normalized Weight';
title('PX4 Normalized Mixer');
xlabel('Control Inputs'); ylabel('Motor Outputs');
xticks(1:6); xticklabels(axis_names);
yticks(1:n_motors);

% Overlay Values
textStrings = num2str(Mixer_Normalized(:), '%0.2f');
textStrings = strtrim(cellstr(textStrings)); 
[x, y] = meshgrid(1:6, 1:n_motors);
text(x(:), y(:), textStrings(:), 'HorizontalAlignment', 'center', ...
    'Color', 'w', 'FontWeight', 'bold');

%% 7. Visualization (Convergence)
fprintf('Generating Convergence Plots...\n');

% Define Time Window (20s before sampleTime)
startTime = max(0, sampleTime - 20);
endTime   = sampleTime;

% --- Helper Function to Extract Windowed Data Safely ---
get_windowed_ts = @(ts) getsamples(ts, find(ts.Time >= startTime & ts.Time <= endTime));

ts_f_sub = get_windowed_ts(ts_force);
ts_t_sub = get_windowed_ts(ts_torque);

t_conv_f = ts_f_sub.Time;
data_conv_f = ts_f_sub.Data;
t_conv_t = ts_t_sub.Time;
data_conv_t = ts_t_sub.Data;

% --- Dimension Check & Correction ---
if size(data_conv_f, 1) ~= length(t_conv_f)
    data_conv_f = permute(data_conv_f, [3 1 2]);
    data_conv_t = permute(data_conv_t, [3 1 2]);
end

extract_axis_data = @(data, axIdx) squeeze(data(:, axIdx, :));

% --- FIGURE 1: LINEAR CONVERGENCE (Fx, Fy, Fz) ---
figure('Name', 'Linear Convergence', 'Color', 'w');
lin_axes = {'Fx Effectiveness', 'Fy Effectiveness', 'Fz Effectiveness'};

for i = 1:3
    subplot(3, 1, i);
    y_plot = extract_axis_data(data_conv_f, i);
    
    plot(t_conv_f, y_plot, 'LineWidth', 1.5);
    title(lin_axes{i});
    ylabel('Coeff Value');
    grid on;
    xlim([startTime, endTime]);
    
    if i == 1
        legend(arrayfun(@(x) sprintf('Motor %d', x), 1:n_motors, 'UniformOutput', false), ...
               'Location', 'eastoutside');
    end
end
xlabel('Time (s)');
sgtitle(['Linear Parameter Convergence (t = ' num2str(startTime) 's to ' num2str(endTime) 's)']);

% --- FIGURE 2: ANGULAR CONVERGENCE (Mx, My, Mz) ---
figure('Name', 'Angular Convergence', 'Color', 'w');
ang_axes = {'Mx Effectiveness', 'My Effectiveness', 'Mz Effectiveness'};

for i = 1:3
    subplot(3, 1, i);
    y_plot = extract_axis_data(data_conv_t, i);
    
    plot(t_conv_t, y_plot, 'LineWidth', 1.5);
    title(ang_axes{i});
    ylabel('Coeff Value');
    grid on;
    xlim([startTime, endTime]);
    
    if i == 1
        legend(arrayfun(@(x) sprintf('Motor %d', x), 1:n_motors, 'UniformOutput', false), ...
               'Location', 'eastoutside');
    end
end
xlabel('Time (s)');
sgtitle(['Angular Parameter Convergence (t = ' num2str(startTime) 's to ' num2str(endTime) 's)']);

%% 8. Data Orthogonality & Excitation Check (NEW)
fprintf('\n=================================================\n');
fprintf('       DATA ORTHOGONALITY CHECK (Input U)\n');
fprintf('=================================================\n');

% Calculate Correlation Matrix of Input Data
Corr_Matrix = corrcoef(U_data');

% Remove diagonal
off_diag_corr = Corr_Matrix - eye(size(Corr_Matrix));
avg_cross_corr = mean(abs(off_diag_corr(:)));

% Calculate Condition Number
cond_num = cond(U_data * U_data');

fprintf('1. Input Condition Number (Lower is better): %.2e\n', cond_num);
if cond_num > 1e6
    fprintf('   -> WARNING: Condition number is high. Inputs may be collinear (poor excitation).\n');
else
    fprintf('   -> OK: Condition number acceptable.\n');
end

fprintf('2. Avg Cross-Motor Correlation (0 to 1): %.4f\n', avg_cross_corr);
if avg_cross_corr > 0.95
    fprintf('   -> WARNING: High correlation. Motors are moving almost identically.\n');
else
    fprintf('   -> OK: Motors show independent movement.\n');
end

figure('Name', 'Input Data Orthogonality', 'Color', 'w');
imagesc(Corr_Matrix);
colormap(parula); colorbar; clim([-1 1]);
title(['Motor Speed Correlation (Avg Abs: ' num2str(avg_cross_corr, '%.2f') ')']);
xlabel('Motor Index'); ylabel('Motor Index');
xticks(1:n_motors); yticks(1:n_motors);

% Annotate
[x, y] = meshgrid(1:n_motors, 1:n_motors);
text(x(:), y(:), num2str(Corr_Matrix(:), '%.2f'), ...
    'HorizontalAlignment', 'center', 'Color', 'w', 'FontSize', 8);

%% 9. Residual Analysis & Spectral Content (FFT)
fprintf('\n=================================================\n');
fprintf('       RESIDUAL & SPECTRAL ANALYSIS\n');
fprintf('=================================================\n');

% --- A. Calculate Time-Domain Residuals ---
% Residual = Real - Predicted
Residuals = Y_real - Y_pred; 

% --- B. Frequency Domain Analysis (FFT) ---
% We assume constant sampling rate for FFT. We'll calculate it from target_time.
fs = 1 / mean(diff(target_time)); % Sampling Frequency (Hz)
L_samples = length(target_time);  % Length of signal
f_axis = fs*(0:(L_samples/2))/L_samples; % Frequency vector

figure('Name', 'Residual Analysis: Time & Frequency', 'Color', 'w');

for i = 1:6
    % --- Time Domain Subplot ---
    subplot(6, 2, 2*i-1);
    plot(target_time, Residuals(i, :), 'Color', [0.4, 0.4, 0.4]);
    grid on;
    ylabel(y_labels{i});
    title(['Residual: ', axis_names{i}]);
    if i == 6, xlabel('Time (s)'); end
    
    % --- Frequency Domain Subplot ---
    subplot(6, 2, 2*i);
    
    % Compute FFT
    Y_fft = fft(Residuals(i, :));
    P2 = abs(Y_fft/L_samples);          % Two-sided spectrum
    P1 = P2(1:floor(L_samples/2)+1);    % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1);
    
    % Plot Magnitude Spectrum
    semilogy(f_axis, P1, 'Color', [0.8, 0.2, 0.2], 'LineWidth', 1);
    grid on;
    title(['FFT of ', axis_names{i}, ' Residual']);
    if i == 6, xlabel('Frequency (Hz)'); end
    if i == 1, ylabel('Magnitude'); end
end

sgtitle('Residual Analysis: Time Domain (Left) vs. Frequency Domain (Right)');

% --- C. Noise Floor Estimation ---
avg_noise_power = mean(var(Residuals, 0, 2));
fprintf('Average Residual Variance (Noise Floor): %.6f\n', avg_noise_power);
fprintf('Spectral analysis complete. Check for peaks indicating unmodeled vibrations.\n');
fprintf('=================================================\n');

%% 10. Corrected Spectral Analysis & Filtered Residuals
fprintf('\n=================================================\n');
fprintf('       FILTERED RESIDUALS & PSD ANALYSIS\n');
fprintf('=================================================\n');

% --- A. Setup Parameters ---
fs = 1 / mean(diff(target_time));   % Sampling Frequency
L_samples = length(target_time);    % Signal Length
window_size = floor(L_samples/8);   % 8 segments for Welch averaging
nfft = 2^nextpow2(window_size);     % Next power of 2 for FFT efficiency

% --- B. Low-Pass Filter for "Clean" Modeling Error ---
% We'll filter out everything above 20Hz (typical drone control bandwidth)
fc = 20; 
[b_filt, a_filt] = butter(4, fc/(fs/2), 'low'); 

figure('Name', 'Residual Analysis: Filtered vs Raw', 'Color', 'w');

for i = 1:6
    % --- 1. Time Domain (Filtered vs Raw) ---
    subplot(6, 2, 2*i-1);
    raw_res = Residuals(i, :);
    filt_res = filtfilt(b_filt, a_filt, raw_res); % Zero-phase filtering
    
    hold on;
    plot(target_time, raw_res, 'Color', [0.8, 0.8, 0.8], 'HandleVisibility', 'off'); % Light gray raw
    plot(target_time, filt_res, 'r', 'LineWidth', 1.2, 'DisplayName', 'Modeling Bias');
    grid on;
    ylabel(y_labels{i});
    title(['Residual: ', axis_names{i}]);
    if i == 1, legend('Location', 'best'); end
    
    % --- 2. Power Spectral Density (Welch) ---
    subplot(6, 2, 2*i);
    % Check for Signal Processing Toolbox, else use basic periodogram
    try
        [pxx, f_psd] = pwelch(raw_res, window_size, [], nfft, fs);
        semilogy(f_psd, pxx, 'LineWidth', 1.2, 'Color', [0, 0.4, 0.7]);
    catch
        % Fallback if pwelch isn't available
        Y_fft = fft(raw_res, nfft);
        pxx = (1/(fs*nfft)) * abs(Y_fft(1:nfft/2+1)).^2;
        pxx(2:end-1) = 2*pxx(2:end-1);
        f_psd = 0:fs/nfft:fs/2;
        semilogy(f_psd, pxx, 'LineWidth', 1.2, 'Color', [0, 0.4, 0.7]);
    end
    
    grid on;
    xlim([0, fs/2]);
    title(['PSD of ', axis_names{i}]);
    if i == 6, xlabel('Frequency (Hz)'); end
end

sgtitle('Residuals: Filtered Time-Domain (Left) & PSD (Right)');