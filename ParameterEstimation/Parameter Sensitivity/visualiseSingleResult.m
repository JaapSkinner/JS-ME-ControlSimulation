% =========================================================================
% VISUALIZE SINGLE MONTE CARLO RESULT (v3 - Z-Axis Inverted Plot)
% =========================================================================
% This script loads a single data file and creates a summary plot.
% It now includes an option to normalize the output wrench.
% CRUCIALLY, it inverts the Z-axis force for plotting to match the intuitive
% 'Z-up' convention, while the underlying data remains in the 'Z-down' frame.
%
clear; clc; close all;

% --- User Settings ---
normalize_output = true; 
nominal_max_force = 41.0;
nominal_max_torque = 1.2;

% --- 1. Load a Result File ---
[fileName, pathName] = uigetfile('*.mat', 'Select a Monte Carlo Result File');
if isequal(fileName, 0)
    disp('User selected Cancel. Script terminated.');
    return;
end
fullFilePath = fullfile(pathName, fileName);
fprintf('Loading data from: %s\n', fullFilePath);
data = load(fullFilePath);

% --- 2. Display Parameter Variations (Unchanged) ---
% ... (this section is the same as before) ...
fprintf('\n--- Parameter Variation Summary for this Run ---\n');
fprintf('Motor Thrust Constant (Kt) Stats:\n');
fprintf('  - Mean: %.4e\n', mean(data.Motor_params.K_T));
fprintf('  - Std Dev: %.4e (%.2f%% of mean)\n', std(data.Motor_params.K_T), 100*std(data.Motor_params.K_T)/mean(data.Motor_params.K_T));
fprintf('Motor Resistance (R) Stats:\n');
fprintf('  - Mean: %.4f Ohms\n', mean(data.Motor_params.R));
fprintf('  - Std Dev: %.4f Ohms (%.2f%% of mean)\n', std(data.Motor_params.R), 100*std(data.Motor_params.R)/mean(data.Motor_params.R));
fprintf('\n--- End of Summary ---\n\n');


% --- 3. Prepare Data for Plotting ---
nSetpoints = size(data.setpoints, 1);
setpoint_indices = 1:nSetpoints;

wrench_to_plot = data.wrenches; 
y_label_force = 'Force (N)';
y_label_torque = 'Torque (Nm)';
plot_title_suffix = '(Physical Units)';

% --- NEW CHANGE IS HERE ---
% Invert the Z-axis force for intuitive plotting (Z-up is positive on the graph)
wrench_to_plot(3, :) = -wrench_to_plot(3, :);
% -------------------------

if normalize_output
    fprintf('Normalizing outputs with Max Force = %.1f N, Max Torque = %.1f Nm\n', ...
        nominal_max_force, nominal_max_torque);
    
    wrench_to_plot(1:3, :) = wrench_to_plot(1:3, :) / nominal_max_force;
    wrench_to_plot(4:6, :) = wrench_to_plot(4:6, :) / nominal_max_torque;
    wrench_to_plot = max(-1, min(1, wrench_to_plot));
    
    y_label_force = 'Normalized Force Command / Output';
    y_label_torque = 'Normalized Torque Command / Output';
    plot_title_suffix = '(Normalized)';
end


% --- 4. Visualize Inputs vs. Outputs ---
figure('Name', ['Visualization for ', fileName], 'NumberTitle', 'off', 'Position', [100, 100, 900, 700]);

% Subplot 1: Forces
subplot(2, 1, 1);
hold on;
plot(setpoint_indices, wrench_to_plot(1,:), 'r-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fx');
plot(setpoint_indices, wrench_to_plot(2,:), 'g-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fy');
plot(setpoint_indices, wrench_to_plot(3,:), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fz (Plot Inverted)');
plot(setpoint_indices, data.setpoints(:,1), 'r--d', 'DisplayName', 'Desired Fx');
plot(setpoint_indices, data.setpoints(:,2), 'g--d', 'DisplayName', 'Desired Fy');
plot(setpoint_indices, data.setpoints(:,3), 'b--d', 'DisplayName', 'Desired Fz');
hold off;
grid on;
title(['Force Comparison ', plot_title_suffix]);
xlabel('Setpoint Index');
ylabel([y_label_force, ' (Z-Axis Flipped for Plot)']);
legend('show', 'Location', 'best');
xticks(setpoint_indices);
xlim([0.5, nSetpoints + 0.5]);
if normalize_output, ylim([-1.1, 1.1]); end

% Subplot 2: Torques (Unchanged)
subplot(2, 1, 2);
hold on;
plot(setpoint_indices, wrench_to_plot(4,:), 'r-o', 'LineWidth', 1.5, 'DisplayName', 'Output Tx');
plot(setpoint_indices, wrench_to_plot(5,:), 'g-o', 'LineWidth', 1.5, 'DisplayName', 'Output Ty');
plot(setpoint_indices, wrench_to_plot(6,:), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Output Tz');
plot(setpoint_indices, data.setpoints(:,4), 'r--d', 'DisplayName', 'Desired Tx');
plot(setpoint_indices, data.setpoints(:,5), 'g--d', 'DisplayName', 'Desired Ty');
plot(setpoint_indices, data.setpoints(:,6), 'b--d', 'DisplayName', 'Desired Tz');
hold off;
grid on;
title(['Torque Comparison ', plot_title_suffix]);
xlabel('Setpoint Index');
ylabel(y_label_torque);
legend('show', 'Location', 'best');
xticks(setpoint_indices);
xlim([0.5, nSetpoints + 0.5]);
if normalize_output, ylim([-1.1, 1.1]); end

sgtitle(['Input vs. Output Wrench for ', strrep(fileName, '_', '\_')]);

disp('Visualization complete.');