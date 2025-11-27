function [u_out, t, params] = generate_orthogonal_multisine(N_inputs, T, f_min, f_max, fs)
% GENERATE_ORTHOGONAL_MULTISINE Generates orthogonal multisine signals.
%
%   [u_out, t, params] = generate_orthogonal_multisine(N_inputs, T, f_min, f_max, fs)
%   generates a set of N_inputs mutually orthogonal multisine signals based
%   on the method described by Morelli in "Practical Aspects of
%   Multiple-Input Design for Aircraft System Identification Flight Tests."
%
%   This is the exact method you need to generate test signals for each of
%   your N motors to identify the 'B' (control effectiveness) matrix.
%
%   Inputs:
%     N_inputs - Number of independent signals to generate (e.g., 8 for an 8-motor drone).
%     T        - Total period/length of the signal in seconds.
%     f_min    - Minimum frequency of interest in Hz (e.g., 0.1).
%     f_max    - Maximum frequency of interest in Hz (e.g., 10).
%     fs       - Sampling frequency in Hz (e.g., 100).
%
%   Outputs:
%     u_out    - [N_samples x N_inputs] matrix of generated signals.
%                Each column is a signal for one motor, normalized to [-1, 1].
%     t        - [N_samples x 1] time vector.
%     params   - A struct containing the design parameters (harmonics,
%                phases, etc.) for verification.
%
%   How it works (based on the paper):
%   1.  "Multisine" [1506]: Each signal is a sum of sinusoids.
%   2.  "Orthogonal" [1562, 1570]: Orthogonality is achieved by assigning a
%       different, non-overlapping set of frequencies (harmonics) to each
%       input signal. This script uses the "combing" method [1575].
%   3.  "Optimized" [1641, 1645]: The phases of the sinusoids are chosen to
%       minimize the peak factor of the signal. This script uses
%       Schroeder's formula [1652], a standard and highly effective method.
%
%   Eugene A. Morelli, "Practical Aspects of Multiple-Input Design for
%   Aircraft System Identification Flight Tests," NASA, 2012.

% --- 1. Setup Basic Parameters ---
fprintf('Generating %d orthogonal multisine signals...\n', N_inputs);
f0 = 1 / T;             % Fundamental frequency resolution (Hz)
N_samples = round(T * fs); % Total number of samples
t = (0:N_samples-1)' / fs; % Time vector

% --- 2. Harmonic Selection (The "Orthogonal" Part) ---
% We find all available harmonic numbers (integers) in our frequency range.
h_min = ceil(f_min / f0);
h_max = floor(f_max / f0);
h_pool = (h_min:h_max)'; % Total pool of available harmonics

fprintf('  Frequency resolution (f0): %.3f Hz\n', f0);
fprintf('  Harmonic range: %d (%.2f Hz) to %d (%.2f Hz)\n', ...
        h_min, h_min*f0, h_max, h_max*f0);
fprintf('  Total available harmonics: %d\n', length(h_pool));

% Check if we have enough harmonics for the number of inputs
if length(h_pool) < N_inputs
    error(['Not enough frequency bins for the number of inputs. ' ...
           'Try increasing T or f_max, or decreasing f_min.']);
end

% "Comb" the harmonics to create N_inputs orthogonal sets [1570, 1575]
% This assigns every N-th harmonic to the same input.
harmonics = cell(N_inputs, 1);
for j = 1:N_inputs
    harmonics{j} = h_pool(j:N_inputs:end);
end

% --- 3. Phase Generation (The "Optimized" Part) ---
% We use Schroeder's formula to set the phases for a low peak factor [1652].
% phi_k = pi * k^2 / M
phases = cell(N_inputs, 1);
for j = 1:N_inputs
    M_j = length(harmonics{j}); % Number of harmonics for this input
    if M_j > 0
        k = (1:M_j)';
        phases{j} = pi * k.^2 / M_j;
    else
        phases{j} = []; % Handle case where an input gets no harmonics
    end
end

% --- 4. Signal Generation (The "Multisine" Part) ---
% We sum the sinusoids for each input.
u_raw = zeros(N_samples, N_inputs);

fprintf('  Generating signals...');
for j = 1:N_inputs
    if isempty(harmonics{j})
        continue; % Skip if this input has no harmonics
    end
    
    % Get the specific frequencies and phases for this input
    f_j = harmonics{j} * f0;  % Vector of frequencies (Hz)
    phi_j = phases{j};        % Vector of phases (rad)
    
    % Create matrices for efficient calculation
    % [N_samples x 1] * [1 x M_j] -> [N_samples x M_j]
    omega_t = 2 * pi * t * f_j';
    
    % [1 x M_j] -> [N_samples x M_j]
    phi_matrix = repmat(phi_j', N_samples, 1);
    
    % Sum all sinusoids for this input
    % Amplitudes are set to 1 for a flat power spectrum [1530]
    arguments = omega_t + phi_matrix;
    u_raw(:, j) = sum(sin(arguments), 2);
end
fprintf(' Done.\n');

% --- 5. Normalization ---
% Normalize each signal to be in the range [-1, 1] for practical use.
u_out = zeros(size(u_raw));
for j = 1:N_inputs
    max_val = max(abs(u_raw(:, j)));
    if max_val > 1e-6
        u_out(:, j) = u_raw(:, j) / max_val;
    end
end

% --- 6. Package Output Parameters ---
params.harmonics = harmonics;
params.phases = phases;
params.frequencies = cellfun(@(h) h * f0, harmonics, 'UniformOutput', false);
params.f0 = f0;
params.fs = fs;
params.T = T;
params.t = t;

fprintf('Signal generation complete.\n');

% --- Embedded Demonstration (runs if function is called with no outputs) ---
if nargout == 0
    fprintf('\n--- Running Demonstration (Generating Plots) ---\n');
    
    % --- Configure Plot Settings for Publication ---
    plot_signals_to_show = min(N_inputs, 3); % Show time series for first 3 inputs
    plot_time_duration = min(T, 5); % Plot first 5 seconds
    plot_samples_count = round(plot_time_duration * fs);
    line_width = 1.5;
    font_size = 12;

    % --- Create Main Figure for Paper ---
    figure('Name', 'Orthogonal Multisine Signal Demonstration', ...
           'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 100, 800, 1000]);
    % Use tiledlayout for better spacing
    t_plot = tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    % --- Plot 1: Time-Domain Signals ---
    ax1 = nexttile(t_plot);
    hold(ax1, 'on');
    colors = lines(plot_signals_to_show);
    lgd_entries = cell(plot_signals_to_show, 1);
    for j = 1:plot_signals_to_show
        plot(ax1, t(1:plot_samples_count), u_out(1:plot_samples_count, j), ...
             'LineWidth', line_width, 'Color', colors(j,:));
        lgd_entries{j} = sprintf('Input %d', j);
    end
    hold(ax1, 'off');
    title(ax1, '(A) Time Series (First 5 Seconds)', 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlabel(ax1, 'Time (s)', 'FontSize', font_size);
    ylabel(ax1, 'Amplitude', 'FontSize', font_size);
    grid(ax1, 'on');
    box(ax1, 'on');
    legend(ax1, lgd_entries, 'Location', 'northeast', 'FontSize', font_size - 2);
    xlim(ax1, [0, plot_time_duration]);
    ylim(ax1, [-1.1, 1.1]);
    set(ax1, 'FontSize', font_size);

    % --- Plot 2: Frequency-Domain (Proof of Orthogonality) ---
    ax2 = nexttile(t_plot);
    hold(ax2, 'on');
    
    % Calculate FFT
    U_fft = abs(fft(u_out));
    U_fft = U_fft(1:floor(N_samples/2)+1, :); % Take first half
    U_fft(2:end-1, :) = 2 * U_fft(2:end-1, :); % Scale
    f_vec = (0:floor(N_samples/2))' * fs / N_samples;
    
    colors_fft = lines(N_inputs);
    lgd_entries_fft = cell(N_inputs, 1);
    
    for j = 1:N_inputs
        plot(ax2, f_vec, U_fft(:, j), 'Color', colors_fft(j,:), ...
             'LineWidth', line_width, 'DisplayName', sprintf('Input %d', j));
        lgd_entries_fft{j} = sprintf('Input %d', j);
    end
    hold(ax2, 'off');
    
    title(ax2, '(B) Frequency Content (Power Spectral Density)', 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlabel(ax2, 'Frequency (Hz)', 'FontSize', font_size);
    ylabel(ax2, 'Amplitude', 'FontSize', font_size);
    set(ax2, 'XScale', 'log', 'YScale', 'log');
    xlim(ax2, [f_min * 0.8, f_max * 1.2]);
    grid(ax2, 'on');
    box(ax2, 'on');
    % Only show legend if N_inputs is small
    if N_inputs <= 10
        legend(ax2, lgd_entries_fft, 'Location', 'southwest', 'FontSize', font_size - 2, 'NumColumns', min(N_inputs, 4));
    end
    set(ax2, 'FontSize', font_size);
    
    % --- Plot 3: Cross-Correlation (Proof of Orthogonality) ---
    ax3 = nexttile(t_plot);
    
    [c, lags] = xcorr(u_out, 'normalized');
    lags_ms = lags * (1000/fs);
    
    % Find indices for auto-correlation (j == k) and cross-correlation (j != k)
    auto_corr_indices = 1:N_inputs+1:N_inputs^2;
    cross_corr_indices = setdiff(1:N_inputs^2, auto_corr_indices);

    hold(ax3, 'on');
    % Plot all cross-correlations in light gray
    if ~isempty(cross_corr_indices)
        plot(ax3, lags_ms, c(:, cross_corr_indices), 'Color', [0.7 0.7 0.7]);
    end
    % Plot all auto-correlations in blue
    plot(ax3, lags_ms, c(:, auto_corr_indices), 'b', 'LineWidth', line_width);
    hold(ax3, 'off');
    
    title(ax3, '(C) Time-Domain Correlation', 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlabel(ax3, 'Lag (ms)', 'FontSize', font_size);
    ylabel(ax3, 'Normalized Corr.', 'FontSize', font_size);
    grid(ax3, 'on');
    box(ax3, 'on');
    xlim(ax3, [-500, 500]); % Show +/- 500ms
    ylim(ax3, [-1.1, 1.1]);
    legend(ax3, {'Cross-Correlation (j \neq k)', 'Auto-Correlation (j = k)'}, ...
           'Location', 'northeast', 'FontSize', font_size - 2);
    set(ax3, 'FontSize', font_size);
    
    fprintf('Demonstration plots generated.\n');
    
    % Clear outputs so they don't flood the command window
    clear u_out t params;
end

end