% =========================================================================
% CALCULATE MAXIMUM WRENCH OUTPUT & VISUALIZE (v9)
% =========================================================================
% This script uses the nominal vehicle parameters to determine the theoretical
% maximum force and torque on each axis.
%
% VERSION 9 REFINEMENTS:
% - Bundles max_authority and min_thrust_offset into a single struct
%   for easy copy-pasting into other scripts.
%
clear; clc; close all;

% --- User Settings for Plotting ---
GENERATE_PLOTS = true;

fprintf('Calculating theoretical maximum wrench output...\n');
% --- 1. Load Nominal Vehicle Parameters ---
run('ParameterEstimationBaseOL.m'); 
Motor_nom = Motor;
Uav_nom = Uav;
N_motors = Uav.N_ROTORS;
Uav_nom.COM = [0,0,0];

% --- 2. Assemble the Nominal 'params' Struct ---
params = struct();
u_min = [-1.0; -1.0; 0.0; -1.0; -1.0; -1.0];
u_max = [ 1.0;  1.0; 1.0;  1.0;  1.0;  1.0];
alpha_Tx = Motor_nom.CommandMixing(2) / 10000;
alpha_Ty = Motor_nom.CommandMixing(3) / 10000;
alpha_Tz = Motor_nom.CommandMixing(4) / 10000;
P = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 1 0 0 0; 1 0 0 0 0 0; 0 1 0 0 0 0];
A_scale = diag([alpha_Tx, alpha_Ty, alpha_Tz, 1, 1, 1]);
params.A_eff = A_scale * P;
params.u_min = u_min;
params.u_max = u_max;
params.M     = Motor.mixingMatrix;
params.N     = N_motors;
params.Kt = Motor_nom.K_T; params.Ke = Motor_nom.K_E; params.R = Motor_nom.R;
params.C_tau = Motor_nom.C_TAU; params.mv = Motor_nom.volt_slope;
params.V_off = Motor_nom.Volt_offset; params.I0 = Motor_nom.I_0;
params.V_off(:) = 0; params.I0(:) = 0; % Force ideal motor behavior
kf_vec = 0.5 * Uav_nom.RHO_AIR * Aero.Cz3P.coefs(1) * (Uav_nom.D_PROP.^2) .* Uav_nom.A_PROP;
km_vec = Motor_nom.C_TAU;
B_matrix = zeros(6, N_motors);
for m = 1:N_motors
    t_m = [0; 0; -kf_vec(m)];
    tau_m = [0; 0; -Uav_nom.ROTOR_DIRECTION(m) * km_vec(m)];
    R_m_b = Uav_nom.R_MOTOR_TO_BODY(:,:,m);
    T_body_i = R_m_b * t_m;
    tau_body_i = R_m_b * tau_m;
    moment_arm = Uav_nom.MotorLoc(m, 1:3)' - Uav_nom.COM';
    tau_from_thrust = cross(moment_arm, T_body_i);
    B_matrix(:, m) = [T_body_i; tau_body_i + tau_from_thrust];
end
params.B_matrix = B_matrix;

% --- 3. Define Maximum Test Setpoints ---
setpoints_max_test = [
     1.0, 0.0, 0.5, 0.0, 0.0, 0.0; -1.0, 0.0, 0.5, 0.0, 0.0, 0.0;
     0.0, 1.0, 0.5, 0.0, 0.0, 0.0;  0.0,-1.0, 0.5, 0.0, 0.0, 0.0;
     0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  0.0, 0.0, 0.1, 0.0, 0.0, 0.0;
     0.0, 0.0, 0.5, 1.0, 0.0, 0.0;  0.0, 0.0, 0.5,-1.0, 0.0, 0.0;
     0.0, 0.0, 0.5, 0.0, 1.0, 0.0;  0.0, 0.0, 0.5, 0.0,-1.0, 0.0;
     0.0, 0.0, 0.5, 0.0, 0.0, 1.0;  0.0, 0.0, 0.5, 0.0, 0.0,-1.0;
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
];
nTests = size(setpoints_max_test, 1);
results = zeros(6, nTests);

% --- 4. Run Calculations ---
for i = 1:nTests
    W_des = setpoints_max_test(i, :)';
    results(:, i) = computeWrench(W_des, params);
end


% --- 5. Analyze and Display Per-Axis Results ---
max_authority = max(abs(results), [], 2);
zero_setpoint_idx = find(all(setpoints_max_test == 0, 2), 1);
min_thrust_offset = 0;
if ~isempty(zero_setpoint_idx)
    min_thrust_offset = abs(results(3, zero_setpoint_idx));
    fprintf('\nDetected a minimum thrust offset of %.3f N at zero command.\n', min_thrust_offset);
end
fprintf('\n--- Maximum Wrench Calculation Results ---\n');
fprintf('Max Fx output: %.2f N\n',   max_authority(1));
fprintf('Max Fy output: %.2f N\n',   max_authority(2));
fprintf('Max Fz output: %.2f N\n',   max_authority(3));
fprintf('------------------------------------------\n');
fprintf('Max Tx output: %.2f Nm\n',  max_authority(4));
fprintf('Max Ty output: %.2f Nm\n',  max_authority(5));
fprintf('Max Tz output: %.2f Nm\n',  max_authority(6));
fprintf('------------------------------------------\n\n');

% --- 6. MODIFIED: Final Output for Analysis Scripts ---
fprintf('>>> COPY THIS STRUCT FOR ANALYSIS SCRIPTS <<<\n');
fprintf('norm_params = struct();\n');
fprintf('norm_params.max_authority = [%.4f; %.4f; %.4f; %.4f; %.4f; %.4f];\n', max_authority);
fprintf('norm_params.min_thrust_offset = %.4f;\n', min_thrust_offset);
fprintf('\n');

% --- 7. Generate Visualization Plots ---
if GENERATE_PLOTS
    % (This section is unchanged and will work as before)
    fprintf('Generating visualization plots...\n');
    setpoint_indices = 1:nTests;
    
    % --- FIGURE 1: UN-NORMALIZED (PHYSICAL UNITS) ---
    wrench_physical = results;
    wrench_physical(3, :) = -wrench_physical(3, :);
    figure('Name', 'Nominal Vehicle Performance (Physical Units)', 'Position', [100, 100, 900, 700]);
    subplot(2, 1, 1);
    hold on;
    plot(setpoint_indices, wrench_physical(1,:), 'r-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fx');
    plot(setpoint_indices, wrench_physical(2,:), 'g-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fy');
    plot(setpoint_indices, wrench_physical(3,:), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fz (Plot Inverted)');
    hold off; grid on; title('Force Comparison (Physical Units)');
    xlabel('Setpoint Index'); ylabel('Force (N) (Z-Axis Flipped for Plot)');
    legend('show', 'Location', 'best'); xticks(setpoint_indices); xlim([0.5, nTests + 0.5]);
    subplot(2, 1, 2);
    hold on;
    plot(setpoint_indices, wrench_physical(4,:), 'r-o', 'LineWidth', 1.5, 'DisplayName', 'Output Tx');
    plot(setpoint_indices, wrench_physical(5,:), 'g-o', 'LineWidth', 1.5, 'DisplayName', 'Output Ty');
    plot(setpoint_indices, wrench_physical(6,:), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Output Tz');
    hold off; grid on; title('Torque Comparison (Physical Units)');
    xlabel('Setpoint Index'); ylabel('Torque (Nm)');
    legend('show', 'Location', 'best'); xticks(setpoint_indices); xlim([0.5, nTests + 0.5]);
    sgtitle('Nominal Vehicle Response to Max Authority Test Setpoints');

    % --- FIGURE 2: NORMALIZED UNITS ---
    wrench_normalized = results;
    wrench_normalized(3, :) = -wrench_normalized(3, :);
    wrench_normalized([1,2,4,5,6], :) = wrench_normalized([1,2,4,5,6], :) ./ max_authority([1,2,4,5,6]);
    max_fz_output = max_authority(3);
    thrust_range = max_fz_output - min_thrust_offset;
    if thrust_range > 0
        wrench_normalized(3, :) = (wrench_normalized(3, :) - min_thrust_offset) / thrust_range;
    end
    wrench_normalized = max(-1, min(1, wrench_normalized));
    
    figure('Name', 'Nominal Vehicle Performance (Normalized)', 'Position', [150, 150, 900, 700]);
    subplot(2, 1, 1);
    hold on;
    plot(setpoint_indices, wrench_normalized(1,:), 'r-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fx');
    plot(setpoint_indices, wrench_normalized(2,:), 'g-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fy');
    plot(setpoint_indices, wrench_normalized(3,:), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Output Fz (Plot Inverted)');
    plot(setpoint_indices, setpoints_max_test(:,1), 'r--d', 'DisplayName', 'Desired Fx');
    plot(setpoint_indices, setpoints_max_test(:,2), 'g--d', 'DisplayName', 'Desired Fy');
    plot(setpoint_indices, setpoints_max_test(:,3), 'b--d', 'DisplayName', 'Desired Fz');
    hold off; grid on; title('Force Comparison (Normalized)');
    xlabel('Setpoint Index'); ylabel('Normalized Force (Z-Axis Flipped)');
    legend('show', 'Location', 'best'); xticks(setpoint_indices); xlim([0.5, nTests + 0.5]); ylim([-1.1, 1.1]);
    subplot(2, 1, 2);
    hold on;
    plot(setpoint_indices, wrench_normalized(4,:), 'r-o', 'LineWidth', 1.5, 'DisplayName', 'Output Tx');
    plot(setpoint_indices, wrench_normalized(5,:), 'g-o', 'LineWidth', 1.5, 'DisplayName', 'Output Ty');
    plot(setpoint_indices, wrench_normalized(6,:), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Output Tz');
    plot(setpoint_indices, setpoints_max_test(:,4), 'r--d', 'DisplayName', 'Desired Tx');
    plot(setpoint_indices, setpoints_max_test(:,5), 'g--d', 'DisplayName', 'Desired Ty');
    plot(setpoint_indices, setpoints_max_test(:,6), 'b--d', 'DisplayName', 'Desired Tz');
    hold off; grid on; title('Torque Comparison (Normalized)');
    xlabel('Setpoint Index'); ylabel('Normalized Torque');
    legend('show', 'Location', 'best'); xticks(setpoint_indices); xlim([0.5, nTests + 0.5]); ylim([-1.1, 1.1]);
    sgtitle('Normalized Response vs. Desired Commands');

    % --- FIGURE 3: FZ THRUST RESPONSE CURVE ---
    figure('Name', 'Nominal Fz Thrust Response Curve', 'Position', [200, 200, 800, 600]);
    hold on;
    fz_commands = setpoints_max_test(:, 3);
    fz_results_physical = -results(3, :);
    [unique_fz_cmds, ~, ic] = unique(fz_commands);
    avg_fz_results = accumarray(ic, fz_results_physical, [], @mean);
    plot(unique_fz_cmds, avg_fz_results, 'b-o', 'LineWidth', 2, 'DisplayName', 'Actual Model Response');
    plot([0, 1], [min_thrust_offset, max_fz_output], 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ideal Linear Response (with offset)');
    hold off; grid on;
    title('Fz Thrust Response Curve of Nominal Model', 'FontSize', 14);
    xlabel('Normalized Fz Command');
    ylabel('Actual Fz Output (N)');
    legend('show', 'Location', 'southeast');
    set(gca, 'FontSize', 12);
    xlim([0, 1]);
end