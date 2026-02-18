function [u_out, t, params] = generate_orthogonal_multisine(N_inputs, T, f_min, f_max, fs)
% GENERATE_ORTHOGONAL_MULTISINE Generates orthogonal multisine signals.
%
%   [u_out, t, params] = generate_orthogonal_multisine(N_inputs, T, f_min, f_max, fs)
%
%   ... (Documentation headers remain the same) ...

% --- 1. Setup Basic Parameters ---
fprintf('Generating %d orthogonal multisine signals...\n', N_inputs);
f0 = 1 / T;             % Fundamental frequency resolution (Hz)
N_samples = round(T * fs); % Total number of samples
t = (0:N_samples-1)' / fs; % Time vector

% --- 2. Harmonic Selection (The "Orthogonal" Part) ---
h_min = ceil(f_min / f0);
h_max = floor(f_max / f0);
h_pool = (h_min:h_max)'; 
fprintf('  Frequency resolution (f0): %.3f Hz\n', f0);
fprintf('  Harmonic range: %d (%.2f Hz) to %d (%.2f Hz)\n', ...
        h_min, h_min*f0, h_max, h_max*f0);
fprintf('  Total available harmonics: %d\n', length(h_pool));

if length(h_pool) < N_inputs
    error(['Not enough frequency bins for the number of inputs. ' ...
           'Try increasing T or f_max, or decreasing f_min.']);
end

harmonics = cell(N_inputs, 1);
for j = 1:N_inputs
    harmonics{j} = h_pool(j:N_inputs:end);
end

% --- 3. Phase Generation (The "Optimized" Part) ---
phases = cell(N_inputs, 1);
for j = 1:N_inputs
    M_j = length(harmonics{j}); 
    if M_j > 0
        k = (1:M_j)';
        phases{j} = pi * k.^2 / M_j;
    else
        phases{j} = []; 
    end
end

% --- 4. Signal Generation (The "Multisine" Part) ---
u_raw = zeros(N_samples, N_inputs);
fprintf('  Generating signals...');
for j = 1:N_inputs
    if isempty(harmonics{j})
        continue; 
    end
    
    f_j = harmonics{j} * f0;  
    phi_j = phases{j};        
    
    omega_t = 2 * pi * t * f_j';
    phi_matrix = repmat(phi_j', N_samples, 1);
    arguments = omega_t + phi_matrix;
    u_raw(:, j) = sum(sin(arguments), 2);
end
fprintf(' Done.\n');

% --- 5. Normalization ---
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
    
    % --- Configure Plot Settings ---
    plot_signals_to_show = min(N_inputs, 3); 
    plot_time_duration = min(T, 5); 
    plot_samples_count = round(plot_time_duration * fs);
    line_width = 1.5;
    font_size = 12;

    % =====================================================================
    % FIGURE 1: TIME DOMAIN
    % =====================================================================
    figure('Name', 'Multisine: Time Domain', 'NumberTitle', 'off', 'Color', 'w');
    ax1 = gca;
    hold(ax1, 'on');
    colors = lines(plot_signals_to_show);
    lgd_entries = cell(plot_signals_to_show, 1);
    
    for j = 1:plot_signals_to_show
        plot(ax1, t(1:plot_samples_count), u_out(1:plot_samples_count, j), ...
             'LineWidth', line_width, 'Color', colors(j,:));
        % UPDATED LEGEND LABEL HERE
        lgd_entries{j} = sprintf('Motor %d', j);
    end
    
    hold(ax1, 'off');
    title(ax1, 'Time Series (First 5 Seconds)', 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlabel(ax1, 'Time (s)', 'FontSize', font_size);
    ylabel(ax1, 'Amplitude (Normalized)', 'FontSize', font_size);
    grid(ax1, 'on');
    box(ax1, 'on');
    legend(ax1, lgd_entries, 'Location', 'northeast', 'FontSize', font_size);
    xlim(ax1, [0, plot_time_duration]);
    ylim(ax1, [-1.1, 1.1]);
    set(ax1, 'FontSize', font_size);

    % =====================================================================
    % FIGURE 2: FREQUENCY DOMAIN
    % =====================================================================
    figure('Name', 'Multisine: Frequency Domain', 'NumberTitle', 'off', 'Color', 'w');
    ax2 = gca;
    hold(ax2, 'on');
    
    % Calculate FFT
    U_fft = abs(fft(u_out));
    U_fft = U_fft(1:floor(N_samples/2)+1, :); 
    U_fft(2:end-1, :) = 2 * U_fft(2:end-1, :); 
    f_vec = (0:floor(N_samples/2))' * fs / N_samples;
    
    colors_fft = lines(N_inputs);
    lgd_entries_fft = cell(N_inputs, 1);
    
    for j = 1:N_inputs
        % UPDATED LEGEND LABEL HERE
        plot(ax2, f_vec, U_fft(:, j), 'Color', colors_fft(j,:), ...
             'LineWidth', line_width, 'DisplayName', sprintf('Motor %d', j));
        lgd_entries_fft{j} = sprintf('Motor %d', j);
    end
    hold(ax2, 'off');
    
    title(ax2, 'Frequency Content (Power Spectral Density)', 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlabel(ax2, 'Frequency (Hz)', 'FontSize', font_size);
    ylabel(ax2, 'Magnitude', 'FontSize', font_size);
    set(ax2, 'XScale', 'log', 'YScale', 'log');
    xlim(ax2, [f_min * 0.8, f_max * 1.2]);
    grid(ax2, 'on');
    box(ax2, 'on');
    
    if N_inputs <= 10
        legend(ax2, lgd_entries_fft, 'Location', 'southwest', 'FontSize', font_size, 'NumColumns', min(N_inputs, 4));
    end
    set(ax2, 'FontSize', font_size);

    % =====================================================================
    % FIGURE 3: CORRELATION
    % =====================================================================
    figure('Name', 'Multisine: Correlation', 'NumberTitle', 'off', 'Color', 'w');
    ax3 = gca;
    
    [c, lags] = xcorr(u_out, 'normalized');
    lags_ms = lags * (1000/fs);
    
    auto_corr_indices = 1:N_inputs+1:N_inputs^2;
    cross_corr_indices = setdiff(1:N_inputs^2, auto_corr_indices);
    hold(ax3, 'on');
    
    if ~isempty(cross_corr_indices)
        plot(ax3, lags_ms, c(:, cross_corr_indices), 'Color', [0.7 0.7 0.7]);
    end
    plot(ax3, lags_ms, c(:, auto_corr_indices), 'b', 'LineWidth', line_width);
    hold(ax3, 'off');
    
    title(ax3, 'Time-Domain Correlation', 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlabel(ax3, 'Lag (ms)', 'FontSize', font_size);
    ylabel(ax3, 'Normalized Corr.', 'FontSize', font_size);
    grid(ax3, 'on');
    box(ax3, 'on');
    xlim(ax3, [-500, 500]); 
    ylim(ax3, [-1.1, 1.1]);
    legend(ax3, {'Cross-Correlation (Motor j \neq k)', 'Auto-Correlation (Motor j = k)'}, ...
           'Location', 'northeast', 'FontSize', font_size);
    set(ax3, 'FontSize', font_size);
    
    fprintf('Demonstration plots generated in separate windows.\n');
    clear u_out t params;
end
end