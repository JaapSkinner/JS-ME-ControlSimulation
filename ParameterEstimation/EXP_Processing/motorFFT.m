%% Motor RPM FFT Analysis Script
% Analyzes the frequency content of motor speeds over a specific time window.
% UPDATES: Added Motor Selection to reduce clutter and High-Contrast colors.
clear; clc; close all;

%% 1. CONFIGURATION
% ---------------------------------------------------------
% Time Window [seconds]
t_start = 23;   % Start analysis 
t_end   = 53;   % End analysis

% Frequency Display Range [Hz]
f_min = 0.1;    % Minimum frequency to plot 
f_max = 1;      % Maximum frequency to plot 

% Motor Selection
motors_to_plot = [1, 2, 3, 4]; % Specific motors to display (e.g., [1 2 3 4])

% Visualization Options
use_log_scale_y = true;  % true = Logarithmic Y-axis (Cleanest), false = Linear
smoothing_factor = 0;    % 0 = Sharpest Peaks (Raw), >0 = Smoother lines
use_windowing = true;    % true = Apply Hann window (Reduces 'leakage' and spread)
% ---------------------------------------------------------

%% 2. FILE SELECTION
try; proj = currentProject; startPath = proj.RootFolder; catch; startPath = pwd; end
fprintf('Select a processed .mat file for FFT analysis...\n');
[fileName, pathName] = uigetfile({'*.mat', 'Processed Data'}, 'Select Data', startPath);
if isequal(fileName, 0); disp('Cancelled.'); return; end
load(fullfile(pathName, fileName));

if ~exist('flightData', 'var') || ~isfield(flightData, 'actuators')
    error('Invalid file: flightData or actuators structure missing.'); 
end

%% 3. DATA PREPARATION
% Check if RPM exists, otherwise warn/fallback
if isfield(flightData.actuators, 'rpm')
    raw_signal = flightData.actuators.rpm;
    sig_label = 'RPM';
elseif isfield(flightData.actuators, 'cmd')
    warning('RPM data not found. Using Motor Outputs (PWM/Norm) instead.');
    raw_signal = flightData.actuators.cmd;
    sig_label = 'Command';
else
    error('No Motor data found.');
end

time = flightData.time;

% Validate Time Window
if t_start < min(time) || t_end > max(time)
    warning('Configured time window is outside data range. Using entire flight.');
    t_start = min(time);
    t_end = max(time);
end

% Slice Data using Logical Mask
mask = time >= t_start & time <= t_end;
t_seg = time(mask);
sig_seg = raw_signal(mask, :);

% Check if data is sufficient
if length(t_seg) < 100
    error('Selected time window is too short for FFT analysis.');
end

% Check if requested motors exist
max_motor_idx = size(sig_seg, 2);
if any(motors_to_plot > max_motor_idx)
    warning('Requested motor index exceeds available motors. Plotting all.');
    motors_to_plot = 1:max_motor_idx;
end

%% 4. FFT CALCULATION
% Calculate Sampling Frequency (Fs) for this segment
dt_avg = mean(diff(t_seg));
Fs = 1 / dt_avg; 
fprintf('Processing Window: %.1fs to %.1fs | Avg Fs: %.1f Hz\n', t_start, t_end, Fs);

% Parameters
L = length(t_seg);             % Length of signal

% Initialize arrays
figure('Name', 'Motor FFT Analysis', 'Color', 'w', 'Position', [100 100 1000 600]);
hold on; grid on;

% Use high-contrast colors (lines) instead of gradient (parula)
colors = lines(length(motors_to_plot)); 

% Pre-calculate Window Vector if enabled
if use_windowing
    w = hann(L);
    % Correction for Amplitude loss due to windowing (Coherent Gain)
    % For Hann window, this factor is 2.
    win_correction = 2; 
else
    w = ones(L,1);
    win_correction = 1;
end

% Loop through only the selected motors
for i = 1:length(motors_to_plot)
    m = motors_to_plot(i);
    
    % 1. Detrend the data 
    % Removes linear drift (slope) which causes massive spread at low Hz.
    y_detrended = detrend(sig_seg(:, m));
    
    % 2. Apply Windowing
    y_processed = y_detrended .* w;
    
    % 3. Compute FFT
    Y = fft(y_processed);
    
    % 4. Compute Single-Sided Spectrum (P1)
    P2 = abs(Y/L);              % Two-sided spectrum
    P1 = P2(1:floor(L/2)+1);    % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1);
    
    % Apply Window Amplitude Correction
    P1 = P1 * win_correction;
    
    % 5. Apply Smoothing (if enabled - default is now 0)
    if smoothing_factor > 0
        P1 = smoothdata(P1, 'gaussian', smoothing_factor);
    end

    % 6. Frequency Vector
    f = Fs*(0:(L/2))/L;
    
    % 7. Plot
    if use_log_scale_y
        semilogy(f, P1, 'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Motor %d', m));
    else
        plot(f, P1, 'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Motor %d', m));
    end
end

%% 5. VISUALIZATION STYLING
title(sprintf('Motor %s Frequency Spectrum (Time: %.1f-%.1fs)', sig_label, t_start, t_end));
xlabel('Frequency (Hz)');
ylabel(['Amplitude (Fluctuation in ' sig_label ')']);
legend('show', 'Location', 'northeast', 'NumColumns', 1);

% --- APPLY FREQUENCY LIMITS ---
% Ensure we don't try to plot beyond the Nyquist frequency (Fs/2)
Nyquist = Fs/2;
if f_max > Nyquist
    warning('Requested max frequency (%.1f Hz) exceeds Nyquist limit (%.1f Hz). Clamping to Nyquist.', f_max, Nyquist);
    f_max = Nyquist;
end
xlim([f_min f_max]);
% ------------------------------

% Annotation box removed per user request

disp('FFT Analysis Complete.');