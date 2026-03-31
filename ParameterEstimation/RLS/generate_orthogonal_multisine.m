function [u_out, t, params] = generate_orthogonal_multisine(N_inputs, T, f_min, f_max, fs)
% GENERATE_ORTHOGONAL_MULTISINE Generates orthogonal multisine signals.
%
%   [u_out, t, params] = generate_orthogonal_multisine(N_inputs, T, f_min, f_max, fs)
%
%   See header for full details. 
%   This version includes an adjustable spacing multiplier to spread frequencies out.

% --- 1. Setup Basic Parameters ---
fprintf('Generating %d orthogonal multisine signals...\n', N_inputs);
f0 = 1 / T;             % Fundamental frequency resolution (Hz)
N_samples = round(T * fs); % Total number of samples
t = (0:N_samples-1)' / fs; % Time vector

% --- 2. Harmonic Selection (The "Orthogonal" Part) ---
h_min = ceil(f_min / f0);
h_max = floor(f_max / f0);
h_pool = (h_min:h_max)'; 

% Check bounds
if length(h_pool) < N_inputs
    error(['Not enough frequency bins for the number of inputs. ' ...
           'Try increasing T or f_max, or decreasing f_min.']);
end

% >>> NEW: Spacing Multiplier <<<
% Increase this >1 to spread the frequencies further apart for each motor.
% E.g., a value of 3 will triple the Hz gap between frequencies on Motor 1.
spacing_multiplier = 4; 

% "Comb" the harmonics
harmonics = cell(N_inputs, 1);
stride = N_inputs * spacing_multiplier;

for j = 1:N_inputs
    % Deal out the harmonics, skipping by the new expanded stride
    harmonics{j} = h_pool(j : stride : end);
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
    
    % --- CONFIGURATION ---
    plot_scale_factor = 0.25; % SCALING FOR PLOT ONLY
    plot_signals_to_show = min(N_inputs, 3); 
    plot_time_duration = min(T, 5); 
    plot_samples_count = round(plot_time_duration * fs);
    line_width = 1.2;
    font_size = 12;
    
    % Create Scaled Data for Plotting
    u_plot = u_out * plot_scale_factor;
    
    % --- FIGURE 1: Time-Domain Signals ---
    figure('Name', 'Time Domain', 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    colors = lines(plot_signals_to_show);
    
    for j = 1:plot_signals_to_show
        plot(t(1:plot_samples_count), u_plot(1:plot_samples_count, j), ...
             'LineWidth', line_width, 'Color', colors(j,:), ...
             'DisplayName', sprintf('Motor Addition %d', j));
    end
    hold off;
    
    title('Time Series (Scaled Output)', 'FontSize', font_size + 2);
    xlabel('Time (s)', 'FontSize', font_size);
    ylabel('Motor Setpoint Addition', 'FontSize', font_size); 
    grid on; box on;
    legend('Location', 'northeast', 'FontSize', font_size - 1);
    xlim([0, plot_time_duration]);
    ylim([-1.1*plot_scale_factor, 1.1*plot_scale_factor]); 
    set(gca, 'FontSize', font_size);
    
    % --- FIGURE 2: Frequency-Domain (Proof of Orthogonality) ---
    figure('Name', 'Frequency Domain', 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    
    % Calculate FFT on the SCALED data
    U_fft = abs(fft(u_plot));
    U_fft = U_fft(1:floor(N_samples/2)+1, :); 
    U_fft(2:end-1, :) = 2 * U_fft(2:end-1, :); 
    f_vec = (0:floor(N_samples/2))' * fs / N_samples;
    
    colors_fft = lines(N_inputs);
    
    % Use STEM plots to make distinct frequencies easier to see
    for j = 1:N_inputs
        stem(f_vec, U_fft(:, j), 'Color', colors_fft(j,:), ...
             'LineWidth', 1.5, 'Marker', 'none', ...
             'DisplayName', sprintf('Motor Addition %d', j));
    end
    hold off;
    
    title(sprintf('Frequency Content (Spacing Multiplier: %d)', spacing_multiplier), 'FontSize', font_size + 2);
    xlabel('Frequency (Hz)', 'FontSize', font_size);
    ylabel('Magnitude (Scaled)', 'FontSize', font_size);
    
    set(gca, 'YScale', 'linear'); 
    xlim([f_min * 0.9, f_max * 1.1]);
    
    grid on; box on;
    if N_inputs <= 12
        legend('Location', 'northeast', 'FontSize', 10, 'NumColumns', ceil(N_inputs/4));
    end
    set(gca, 'FontSize', font_size);
    
    % --- FIGURE 3: Cross-Correlation ---
    figure('Name', 'Correlation', 'Color', 'w', 'NumberTitle', 'off');
    
    [c, lags] = xcorr(u_out, 'normalized');
    lags_ms = lags * (1000/fs);
    
    auto_corr_indices = 1:N_inputs+1:N_inputs^2;
    cross_corr_indices = setdiff(1:N_inputs^2, auto_corr_indices);
    
    hold on;
    if ~isempty(cross_corr_indices)
        h1 = plot(lags_ms, c(:, cross_corr_indices), 'Color', [0.8 0.8 0.8]);
    end
    h2 = plot(lags_ms, c(:, auto_corr_indices), 'b', 'LineWidth', 1.5);
    hold off;
    
    title('Orthogonality Check (Correlation)', 'FontSize', font_size + 2);
    xlabel('Lag (ms)', 'FontSize', font_size);
    ylabel('Normalized Correlation', 'FontSize', font_size);
    grid on; box on;
    xlim([-500, 500]); 
    ylim([-1.1, 1.1]);
    
    if ~isempty(cross_corr_indices)
        legend([h2(1), h1(1)], {'Auto-Correlation', 'Cross-Correlation'}, ...
            'Location', 'northeast', 'FontSize', font_size);
    else
        legend(h2(1), 'Auto-Correlation', 'Location', 'northeast');
    end
    set(gca, 'FontSize', font_size);
    
    fprintf('Demonstration plots generated in separate windows.\n');
    clear u_out t params;
end
end