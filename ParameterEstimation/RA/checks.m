%% Overactuated Closed-Loop System ID & Coherence (V7 - Complete)
% Restores all original plots, Welch PSDs, and console outputs while
% maintaining the closed-loop signal decomposition and pure-excitation referencing.
clear; clc; close all;

%% 1. CONFIGURATION
% ---------------------------------------------------------
t_start = 130;          % Start analysis window [s]
t_end   = 144;          % End analysis window [s]
target_motor = 1;       % Which motor to isolate and analyze (e.g., 1-8)
num_peaks    = 2;       % Number of dominant excitation frequencies to track

f_min = 0.05;           % Lower limit for plots [Hz]
f_max = 5.0;            % Upper limit for plots [Hz]

window_length_sec = 1.0; % Desired Welch window length (auto-adjusts if needed)
overlap_pct = 50;        
% ---------------------------------------------------------

%% 2. LOAD DATA
try; proj = currentProject; startPath = proj.RootFolder; catch; startPath = pwd; end
fprintf('Select a processed .mat file...\n');
[fileName, pathName] = uigetfile({'*.mat', 'Processed Data'}, 'Select Data', startPath);
if isequal(fileName, 0); disp('Cancelled.'); return; end
load(fullfile(pathName, fileName));

if ~exist('flightData', 'var')
    error('Invalid file: flightData structure missing.'); 
end

%% 3. PREPARE DATA
time = flightData.time;
mask = time >= t_start & time <= t_end;
t_seg = time(mask);

Fs = 1 / mean(diff(t_seg));
L = length(t_seg);
fprintf('Processing Window: %.1fs to %.1fs (%.1fs total) | Fs: %.1f Hz\n', t_start, t_end, (t_end-t_start), Fs);

% --- 3A. Extract Motor Signals & Decompose ---
if isfield(flightData.actuators, 'rpm')
    u_total = flightData.actuators.rpm(mask, target_motor);
    mot_label = 'RPM';
else
    u_total = flightData.actuators.cmd(mask, target_motor);
    mot_label = 'Cmd';
end

u_exc = zeros(size(u_total));
has_exc = false;
if isfield(flightData, 'multisine_excitation_status')
    if isfield(flightData.multisine_excitation_status, 'excitation')
        u_exc = flightData.multisine_excitation_status.excitation(mask, target_motor);
        has_exc = true;
    else
        field_name = sprintf('excitation_%02d', target_motor-1); 
        if isfield(flightData.multisine_excitation_status, field_name)
            u_exc = flightData.multisine_excitation_status.(field_name)(mask);
            has_exc = true;
        end
    end
end

if has_exc
    fprintf('Multisine Excitation found. Decomposing signals...\n');
    scale_factor = max(abs(detrend(u_total, 'constant'))) / max(abs(u_exc));
    u_exc_scaled = u_exc * scale_factor; 
    u_ctrl = u_total - u_exc_scaled;
else
    error('Multisine Excitation not found. This analysis requires the pure excitation signal.');
end

% --- 3B. Extract Accelerations & Detrend ---
accels = [flightData.px4_est.ang_accel(mask, 1:3), flightData.px4_est.acc_body_meas(mask, 1:3)];
accels_dyn = detrend(accels, 'constant');

u_total_dyn = detrend(u_total, 'constant');
u_exc_dyn   = detrend(u_exc_scaled, 'constant');
u_ctrl_dyn  = detrend(u_ctrl, 'constant');

%% 4. MATHEMATICAL SETUP & CALCULATIONS
% --- Coherence & PSD Guards ---
max_nperseg = floor(L / 4); 
nperseg = min(round(window_length_sec * Fs), max_nperseg);
noverlap = round(nperseg * (overlap_pct / 100));

% --- WELCH PSD ---
[Pxx_u, f_psd] = pwelch(u_total_dyn, hann(nperseg), noverlap, nperseg, Fs);

% --- ZERO-PADDED FFTs ---
NFFT = 2^nextpow2(L * 8); 
f_fft = Fs*(0:(NFFT/2))/NFFT;
valid_idx_fft = f_fft >= f_min & f_fft <= f_max;

% Helper functions to properly calculate, truncate, and scale the 1-sided FFT
get_P1 = @(P2) [P2(1); 2*P2(2:NFFT/2); P2(NFFT/2+1)];
calc_fft = @(sig) get_P1(abs(fft(sig(:) .* hann(L), NFFT)/L));

% Apply the fixed FFT function
P1_total = calc_fft(u_total_dyn); 
P1_exc   = calc_fft(u_exc_dyn);   
P1_ctrl  = calc_fft(u_ctrl_dyn);  

P1_accels = zeros(NFFT/2+1, 6);
for i = 1:6
    P1_accels(:,i) = calc_fft(accels_dyn(:,i));
end

% --- AUTOMATED PEAK DETECTION (On Pure Excitation) ---
% Find the highest peak to set a dynamic threshold (5% of max)
max_exc_mag = max(P1_exc(valid_idx_fft));
threshold = 0.05 * max_exc_mag; 

% Find all frequencies that make up the multisine signal
[pks, locs] = findpeaks(P1_exc(valid_idx_fft), f_fft(valid_idx_fft), ...
    'MinPeakHeight', threshold, 'MinPeakDistance', 0.1);
target_freqs = sort(locs);

% --- MULTISINE FREQUENCY PRINTOUT ---
fprintf('\n=== Detected Multisine Frequencies (Motor %d) ===\n', target_motor);
fprintf('Found %d distinct frequencies in the excitation signal:\n', length(target_freqs));
% Print them out in a clean, comma-separated list
freq_strings = arrayfun(@(f) sprintf('%.3f Hz', f), target_freqs, 'UniformOutput', false);
fprintf('  > %s\n', strjoin(freq_strings, ', '));

% --- SNR CALCULATION & PRINTOUT ---
fprintf('\n=== SNR Analysis at Target Frequencies ===\n');
fprintf('Freq (Hz) | Excitation Mag | Controller Mag | SNR (dB)\n');
fprintf('------------------------------------------------------\n');

for i = 1:length(target_freqs)
    f_val = target_freqs(i);
    [~, idx] = min(abs(f_fft - f_val));
    
    mag_exc  = P1_exc(idx);
    mag_ctrl = max(P1_ctrl(idx), eps); 
    
    snr_db = 20 * log10(mag_exc / mag_ctrl);
    
    fprintf('%8.3f  | %14.4f | %14.4f | %7.2f \n', ...
        f_val, mag_exc, mag_ctrl, snr_db);
end



%% 5. FIGURE 1: TIME DOMAIN OVERVIEW
figure('Name', 'Fig 1: Time Domain Overview', 'Color', 'w', 'Position', [50 50 1000 900]);

subplot(3,1,1);
plot(t_seg, u_total_dyn, 'k', 'LineWidth', 2); hold on; grid on;
plot(t_seg, u_exc_dyn, 'm--', 'LineWidth', 1.5);
plot(t_seg, u_ctrl_dyn, 'b:', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel(sprintf('Magnitude (%s)', mot_label));
legend('Total Output', 'Pure Excitation', 'Controller Effort', 'Location', 'eastoutside');
title(sprintf('Motor %d Signal Decomposition', target_motor));

subplot(3,1,2);
plot(t_seg, accels_dyn(:,1), 'r', 'LineWidth', 1.2); hold on; grid on;
plot(t_seg, accels_dyn(:,2), 'b', 'LineWidth', 1.2);
plot(t_seg, accels_dyn(:,3), 'g', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Ang Accel (rad/s^2)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'eastoutside');
title('Angular Accelerations');

subplot(3,1,3);
plot(t_seg, accels_dyn(:,4), 'c', 'LineWidth', 1.2); hold on; grid on;
plot(t_seg, accels_dyn(:,5), 'm', 'LineWidth', 1.2);
plot(t_seg, accels_dyn(:,6), 'k', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Lin Accel (m/s^2)');
legend('X', 'Y', 'Z', 'Location', 'eastoutside');
title('Linear Accelerations');

%% 6. FIGURE 2: INPUT SIGNAL RETENTION & PSD
figure('Name', 'Fig 2: Input Spectral Analysis', 'Color', 'w', 'Position', [100 100 1000 800]);

subplot(2,1,1);
plot(f_fft(valid_idx_fft), P1_total(valid_idx_fft), 'k', 'LineWidth', 2); hold on; grid on;
plot(f_fft(valid_idx_fft), P1_exc(valid_idx_fft), 'm--', 'LineWidth', 1.5);
plot(f_fft(valid_idx_fft), P1_ctrl(valid_idx_fft), 'b:', 'LineWidth', 1.5);
for i = 1:length(target_freqs)
    xline(target_freqs(i), 'k--', 'LineWidth', 1.5);
    plot(target_freqs(i), pks(find(locs == target_freqs(i))), 'ko', 'MarkerFaceColor', 'k');
    text(target_freqs(i)+0.05, pks(find(locs == target_freqs(i))), sprintf('%.2f Hz', target_freqs(i)), 'FontWeight', 'bold');
end
xlim([f_min f_max]); xlabel('Frequency (Hz)'); ylabel('Magnitude');
legend('Total Command', 'Pure Excitation', 'Controller Effort', 'Location', 'eastoutside');
title('Signal Retention (Is Excitation stronger than Controller?)');

subplot(2,1,2);
plot(f_psd, 10*log10(Pxx_u), 'k', 'LineWidth', 1.5); hold on; grid on;
for i = 1:length(target_freqs); xline(target_freqs(i), 'k--', 'LineWidth', 1.5); end
xlim([f_min f_max]); xlabel('Frequency (Hz)'); ylabel('Power (dB/Hz)');
title('Welch PSD of Total Motor Input (Smoothed baseline)');

%% 7. FIGURE 3: AIRFRAME ACCELERATION FFTS
figure('Name', 'Fig 3: 6-DoF Acceleration FFTs', 'Color', 'w', 'Position', [150 150 1000 800]);

subplot(2,1,1);
plot(f_fft(valid_idx_fft), P1_accels(valid_idx_fft, 1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(f_fft(valid_idx_fft), P1_accels(valid_idx_fft, 2), 'b', 'LineWidth', 1.5);
plot(f_fft(valid_idx_fft), P1_accels(valid_idx_fft, 3), 'g', 'LineWidth', 1.5);
for i = 1:length(target_freqs); xline(target_freqs(i), 'k--', 'LineWidth', 1.5); end
xlim([f_min f_max]); ylabel('Magnitude (rad/s^2)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'eastoutside'); 
title('Angular Acceleration Spectrum');

subplot(2,1,2);
plot(f_fft(valid_idx_fft), P1_accels(valid_idx_fft, 4), 'c', 'LineWidth', 1.5); hold on; grid on;
plot(f_fft(valid_idx_fft), P1_accels(valid_idx_fft, 5), 'm', 'LineWidth', 1.5);
plot(f_fft(valid_idx_fft), P1_accels(valid_idx_fft, 6), 'k', 'LineWidth', 1.5);
for i = 1:length(target_freqs); xline(target_freqs(i), 'k--', 'LineWidth', 1.5); end
xlim([f_min f_max]); xlabel('Frequency (Hz)'); ylabel('Magnitude (m/s^2)');
legend('X', 'Y', 'Z', 'Location', 'eastoutside'); 
title('Linear Acceleration Spectrum');

%% 8. FIGURE 4: TRUE OPEN-LOOP COHERENCE
figure('Name', 'Fig 4: True Open-Loop Coherence', 'Color', 'w', 'Position', [200 200 1200 600]);

[Cxy_roll, f_coh]  = mscohere(u_exc_dyn, accels_dyn(:,1), hann(nperseg), noverlap, nperseg, Fs);
[Cxy_pitch, ~]     = mscohere(u_exc_dyn, accels_dyn(:,2), hann(nperseg), noverlap, nperseg, Fs);
[Cxy_yaw, ~]       = mscohere(u_exc_dyn, accels_dyn(:,3), hann(nperseg), noverlap, nperseg, Fs);
[Cxy_x, ~]         = mscohere(u_exc_dyn, accels_dyn(:,4), hann(nperseg), noverlap, nperseg, Fs);
[Cxy_y, ~]         = mscohere(u_exc_dyn, accels_dyn(:,5), hann(nperseg), noverlap, nperseg, Fs);
[Cxy_z, ~]         = mscohere(u_exc_dyn, accels_dyn(:,6), hann(nperseg), noverlap, nperseg, Fs);

plot(f_coh, Cxy_roll, 'r', 'LineWidth', 1.5); hold on; grid on;
plot(f_coh, Cxy_pitch, 'b', 'LineWidth', 1.5);
plot(f_coh, Cxy_yaw, 'g', 'LineWidth', 1.5);
plot(f_coh, Cxy_x, 'c', 'LineWidth', 1.5);
plot(f_coh, Cxy_y, 'm', 'LineWidth', 1.5);
plot(f_coh, Cxy_z, 'k', 'LineWidth', 1.5);
yline(0.6, 'k:', 'LineWidth', 1.5, 'Label', 'Strong Linearity Threshold');

fprintf('\n=== 6-DoF Coherence at Target Frequencies (Excitation -> Response) ===\n');
fprintf('Freq (Hz) | Roll  | Pitch | Yaw   | X     | Y     | Z     \n');
fprintf('----------------------------------------------------------\n');

for i = 1:length(target_freqs)
    f_val = target_freqs(i); xline(f_val, 'k--', 'LineWidth', 1.5);
    [~, idx] = min(abs(f_coh - f_val));
    
    plot(f_val, Cxy_roll(idx), 'ro', 'MarkerFaceColor', 'r');
    plot(f_val, Cxy_pitch(idx), 'bo', 'MarkerFaceColor', 'b');
    plot(f_val, Cxy_yaw(idx), 'go', 'MarkerFaceColor', 'g');
    plot(f_val, Cxy_x(idx), 'co', 'MarkerFaceColor', 'c');
    plot(f_val, Cxy_y(idx), 'mo', 'MarkerFaceColor', 'm');
    plot(f_val, Cxy_z(idx), 'ko', 'MarkerFaceColor', 'k');
    
    fprintf('%8.3f  | %5.2f | %5.2f | %5.2f | %5.2f | %5.2f | %5.2f \n', ...
        f_val, Cxy_roll(idx), Cxy_pitch(idx), Cxy_yaw(idx), Cxy_x(idx), Cxy_y(idx), Cxy_z(idx));
end

xlim([f_min f_max]); ylim([0 1.1]);
xlabel('Frequency (Hz)'); ylabel('Coherence (\gamma^2)');
legend('Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z', 'Location', 'eastoutside');
title('Targeted Coherence (Based Purely on Injected Excitation)');



%% 10. EXPERIMENTAL EXCITATION QUALITY METRICS
fprintf('\n=== Experimental Excitation Quality (Real-World) ===\n');

% Extract the full matrix of all motor outputs for the experimental data
if isfield(flightData.actuators, 'rpm')
    % Assuming an octocopter (1:8). Change to 1:4 for a quad.
    u_total_all = flightData.actuators.rpm(:, 1:8); 
else
    u_total_all = flightData.actuators.cmd(:, 1:8);
end

% We use the detrended actual output for the specific time window
U_exp_matrix = detrend(u_total_all(mask, :), 'constant');

% --- Metric 1: Max Cross-Correlation ---
R_exp = corrcoef(U_exp_matrix); 
R_exp_off_diag = R_exp - eye(size(R_exp));
max_cross_corr_exp = max(abs(R_exp_off_diag(:)));

% --- Metric 2: Condition Number ---
cond_num_exp = cond(U_exp_matrix);

% --- Metric 3: Root Mean Square (RMS) Power Balance ---
rms_vals_exp = rms(U_exp_matrix);
max_rms_exp = max(rms_vals_exp);
min_rms_exp = min(rms_vals_exp);
power_balance_exp = min_rms_exp / max_rms_exp; 

% --- Print Report ---
fprintf('1. Max Cross-Correlation: %8.4f  (Real-World Target: < 0.20)\n', max_cross_corr_exp);
fprintf('2. Matrix Condition Num:  %8.2f  (Real-World Target: < 50.0)\n', cond_num_exp);
fprintf('3. Excitation Balance:    %8.2f%% (Real-World Target: > 70%%)\n', power_balance_exp * 100);

if cond_num_exp > 100
    fprintf('\nWARNING: Poor condition number! The flight controller might be fighting your excitation.\n');
elseif max_cross_corr_exp > 0.3
    fprintf('\nWARNING: High cross-correlation. Motors are moving too dependently.\n');
else
    fprintf('\nSUCCESS: Experimental data is rich enough for System ID!\n');
end
fprintf('====================================================\n');
disp('=== Analysis Complete ===');




