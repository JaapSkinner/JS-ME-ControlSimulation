%% Simulated Overactuated Closed-Loop System ID & Coherence
% Processes Simulink 'simOut' timeseries data from a saved .mat file.
clear; clc; close all;

%% 1. CONFIGURATION
% ---------------------------------------------------------
t_start = 2;           % Start analysis window [s] (Adjust for your sim)
t_end   = 15;           % End analysis window [s] (Adjust for your sim)
target_motor = 1;       % Which motor to isolate and analyze (e.g., 1-8)
f_min = 0.05;           % Lower limit for plots [Hz]
f_max = 5.0;            % Upper limit for plots [Hz]
window_length_sec = 2.0; % Desired Welch window length 
overlap_pct = 50;        
% ---------------------------------------------------------

%% 2. LOAD DATA
try; proj = currentProject; startPath = proj.RootFolder; catch; startPath = pwd; end
fprintf('Select a saved simulation .mat file...\n');
[fileName, pathName] = uigetfile({'*.mat', 'Simulated Data'}, 'Select Sim Data', startPath);
if isequal(fileName, 0); disp('Cancelled.'); return; end

% Load the selected file
load(fullfile(pathName, fileName));

% Verify the file actually contains the simOut variable
if ~exist('simOut', 'var')
    error('Invalid file: ''simOut'' timeseries data missing from the selected .mat file.'); 
end

%% 3. PREPARE SIMULATION DATA
% Extract Time and Mask
time = squeeze(simOut.motorCmds.Time);
L_total = length(time);
mask = time >= t_start & time <= t_end;
t_seg = time(mask);
Fs = 1 / mean(diff(t_seg));
L = length(t_seg);

fprintf('\n=== SIMULATION DATA LOADED ===\n');
fprintf('File: %s\n', fileName);
fprintf('Processing Window: %.1fs to %.1fs (%.1fs total) | Fs: %.1f Hz\n', t_start, t_end, (t_end-t_start), Fs);

% --- 3A. Extract Motor Signals & Decompose ---
u_total_all = squeeze(simOut.motorCmds.Data);
if size(u_total_all, 1) ~= L_total
    u_total_all = u_total_all'; 
end

u_exc_all = squeeze(simOut.multisineExcitation.Data);
if size(u_exc_all, 1) ~= L_total
    u_exc_all = u_exc_all'; 
end

% Isolate target motor using the mask
u_total = u_total_all(mask, target_motor);
u_exc   = u_exc_all(mask, target_motor);
mot_label = 'Sim Cmd';

% Scale excitation if it was generated as normalized (-1 to 1) 
scale_factor = max(abs(detrend(u_total, 'constant'))) / max(abs(u_exc));
if scale_factor > 0 && ~isinf(scale_factor)
    u_exc_scaled = u_exc * scale_factor;
else
    u_exc_scaled = u_exc; 
end

u_ctrl = u_total - u_exc_scaled;

% --- 3B. Extract Accelerations & Detrend ---
% Match the real-data order: [Angular 1:3, Linear 4:6]
ang_accel_all = squeeze(simOut.angularAccel.Data);
if size(ang_accel_all, 1) ~= L_total
    ang_accel_all = ang_accel_all'; 
end

lin_accel_all = squeeze(simOut.linearAccel.Data);
if size(lin_accel_all, 1) ~= L_total
    lin_accel_all = lin_accel_all'; 
end

accels = [ang_accel_all(mask, :), lin_accel_all(mask, :)];

% Detrend everything for Fourier analysis
accels_dyn  = detrend(accels, 'constant');
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
max_exc_mag = max(P1_exc(valid_idx_fft));
threshold = 0.05 * max_exc_mag; 
[pks, locs] = findpeaks(P1_exc(valid_idx_fft), f_fft(valid_idx_fft), ...
    'MinPeakHeight', threshold, 'MinPeakDistance', 0.1);
target_freqs = sort(locs);

% --- MULTISINE FREQUENCY PRINTOUT (Target Motor) ---
fprintf('\n=== Detected Multisine Frequencies (Motor %d) ===\n', target_motor);
freq_strings = arrayfun(@(f) sprintf('%.3f Hz', f), target_freqs, 'UniformOutput', false);
fprintf('Found %d distinct frequencies:\n  > %s\n', length(target_freqs), strjoin(freq_strings, ', '));

% --- ALL 8 MOTORS FREQUENCY MAP CHECK ---
fprintf('\n=== 8-MOTOR FREQUENCY MAP CHECK ===\n');
for m = 1:8
    sig = u_exc_all(mask, m);
    P1_m = calc_fft(detrend(sig, 'constant'));
    max_mag = max(P1_m(valid_idx_fft));
    if max_mag < 1e-6
        fprintf('Motor %d: No excitation\n', m);
        continue;
    end
    thresh_m = 0.05 * max_mag;
    [~, locs_m] = findpeaks(P1_m(valid_idx_fft), f_fft(valid_idx_fft), 'MinPeakHeight', thresh_m, 'MinPeakDistance', 0.1);
    f_str = arrayfun(@(f) sprintf('%.3f', f), sort(locs_m), 'UniformOutput', false);
    fprintf('Motor %d (%d freqs): [%s] Hz\n', m, length(locs_m), strjoin(f_str, ', '));
end

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
    fprintf('%8.3f  | %14.4f | %14.4f | %7.2f \n', f_val, mag_exc, mag_ctrl, snr_db);
end

%% 5. FIGURE 1: TIME DOMAIN OVERVIEW
figure('Name', 'Fig 1: Time Domain Overview', 'Color', 'w', 'Position', [50 50 1000 900]);
subplot(3,1,1);
plot(t_seg, u_total_dyn, 'k', 'LineWidth', 2); hold on; grid on;
plot(t_seg, u_exc_dyn, 'm--', 'LineWidth', 1.5);
plot(t_seg, u_ctrl_dyn, 'b:', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel(sprintf('Magnitude (%s)', mot_label));
legend('Total Output', 'Pure Excitation', 'Controller Effort', 'Location', 'eastoutside');
title(sprintf('Motor %d Signal Decomposition (Simulated)', target_motor));

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
disp('=== Analysis Complete ===');

%% 9. EXCITATION QUALITY METRICS
fprintf('\n=== Excitation Quality Metrics (All 8 Motors) ===\n');

% Isolate the pure excitation signals for the time window
U_matrix = u_exc_all(mask, :);

% --- Metric 1: Max Cross-Correlation ---
% Compute the correlation matrix
R = corrcoef(U_matrix); 
% Strip out the diagonal (which is always exactly 1.0, comparing a motor to itself)
R_off_diag = R - eye(size(R));
max_cross_corr = max(abs(R_off_diag(:)));

% --- Metric 2: Condition Number ---
% Evaluates how well the signals span the 8-dimensional input space
cond_num = cond(U_matrix);

% --- Metric 3: Root Mean Square (RMS) Power Balance ---
% Checks if all motors are being excited with roughly the same energy
rms_vals = rms(U_matrix);
max_rms = max(rms_vals);
min_rms = min(rms_vals);
power_balance = min_rms / max_rms; % 1.0 means perfectly balanced

% --- Print Report ---
fprintf('1. Max Cross-Correlation: %8.4f  (Ideal: 0.0000, Good: < 0.05)\n', max_cross_corr);
fprintf('2. Matrix Condition Num:  %8.2f  (Ideal: 1.00,   Good: < 10.0)\n', cond_num);
fprintf('3. Excitation Balance:    %8.2f%% (Ideal: 100%%,   Good: > 80%%)\n', power_balance * 100);

if cond_num > 50
    fprintf('\nWARNING: High condition number! Your estimator may struggle to converge.\n');
    fprintf('Check if any motors are unexcited, or if frequencies are overlapping.\n');
elseif max_cross_corr > 0.1
    fprintf('\nWARNING: High cross-correlation detected. Signals are not strictly orthogonal.\n');
else
    fprintf('\nSUCCESS: Excitation signals are highly orthogonal and well-conditioned for System ID!\n');
end
fprintf('=================================================\n');