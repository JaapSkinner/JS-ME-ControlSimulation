%% Interpolation Method Demonstration for Thesis
clear; clc; close all;

% 1. Create Synthetic Data (Sparse Sampling)
% Imagine this is a digital signal sampled at 10Hz
t_low = 0:0.1:1; 
% Signal: A step change followed by a ramp
data_low = [0, 0, 0, 1, 1, 1, 1.5, 2.0, 2.5, 3.0, 3.0]; 

% 2. Define High-Frequency Query Grid (e.g., 100Hz Simulation Step)
t_high = 0:0.01:1;

% 3. Apply Methods
% Method A: Linear (First-Order Hold) - For Physics (Pos, Vel, RPM)
data_linear = interp1(t_low, data_low, t_high, 'linear');

% Method B: Previous (Zero-Order Hold) - For Digital Commands
data_zoh = interp1(t_low, data_low, t_high, 'previous');

%% 4. Generate Thesis-Ready Plot
figure('Color', 'w', 'Position', [200, 200, 700, 400]);

% Plot the "True" sparse samples
p1 = plot(t_low, data_low, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Raw Samples (Low Rate)');
hold on; grid on;

% Plot Linear Interpolation
p2 = plot(t_high, data_linear, 'b-', 'LineWidth', 2, 'DisplayName', 'Linear (Physics/RPM)');

% Plot ZOH Interpolation
p3 = plot(t_high, data_zoh, 'r--', 'LineWidth', 2, 'DisplayName', 'Zero-Order Hold (Commands)');

% Formatting
xlabel('Time [s]');
ylabel('Signal Amplitude');
title('Comparison of Resampling Strategies');
legend([p1, p2, p3], 'Location', 'northwest');
ylim([-0.5 3.5]);

% Add Annotations for Thesis Context
text(0.35, 0.5, 'Digital "Step" preserved', 'Color', 'r', 'FontSize', 10);
text(0.65, 1.8, 'Smooth physical transition', 'Color', 'b', 'FontSize', 10);