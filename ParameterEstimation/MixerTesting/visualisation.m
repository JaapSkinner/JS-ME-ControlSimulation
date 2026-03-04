%% Plot Validation Results: Nominal vs. RLS Comparison
% This script loads a "Validation_*.mat" file and generates high-quality 
% plots suitable for publication/thesis.
clearvars; clc; close all;

%% 1. Load Data
fprintf('Select a Validation Result file (Validation_*.mat)...\n');
baseDir = fullfile(pwd, 'Results', 'Mixer_Validation');
if ~isfolder(baseDir), baseDir = pwd; end
[fName, pName] = uigetfile(fullfile(baseDir, '*.mat'), 'Select Validation Data');
if isequal(fName, 0), return; end
load(fullfile(pName, fName));
fprintf('Loaded: %s\n', fName);

if ~exist('simOut_Nominal', 'var') || ~exist('simOut_RLS', 'var')
    error('This file does not contain comparison data (simOut_Nominal & simOut_RLS).');
end

%% 2. Data Extraction
names_ref    = {'trajectory_timeseries', 'Ref_Trajectory', 'Reference'};
names_states = {'UAV_State', 'States', 'x_state', 'Plant_States'};
names_torque = {'Moments_cmd', 'Torque_cmd', 'Moments'};

% --- Extract Reference ---
if exist('trajectory_timeseries', 'var')
    ts_ref = trajectory_timeseries;
else
    ts_ref = find_signal(simOut_Nominal, names_ref);
end
if isempty(ts_ref), error('Reference Trajectory not found.'); end
[t_ref, pos_ref] = sanitize_data(ts_ref, 1:3); 
[~,     att_ref] = sanitize_data(ts_ref, 4:6);

% --- Extract Nominal Run ---
[t_nom, pos_nom, att_nom, torq_nom] = extract_run(simOut_Nominal, names_states, names_torque);

% --- Extract RLS Run ---
[t_rls, pos_rls, att_rls, torq_rls] = extract_run(simOut_RLS, names_states, names_torque);

%% 3. Configuration & Plot Styling
T_pad  = 20; 
T_step = 10; 
T_rest = 10;
block_dur = T_step + T_rest;
axis_names = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};

% --- Plot Styling for Thesis (BOLD Lines) ---
LW_ref  = 2.0; % Reference Line Width (Dashed)
LW_nom  = 2.5; % Nominal Line Width
LW_rls  = 3.0; % RLS Line Width (Thickest)
FS_axis = 12;  % Axis Font Size
FS_title= 14;  % Title Font Size

%% 4. Calculate Leakage Metrics

% --- A. X-Axis Step Leakage (Index 1) ---
t_x_start = T_pad; 
t_x_end   = T_pad + T_step;
[rmse_zx_nom, rmse_zx_rls] = calc_leakage(t_ref, t_nom, t_rls, pos_ref(:,3), pos_nom(:,3), pos_rls(:,3), t_x_start, t_x_end);

% --- B. Pitch Step Leakage (Index 5) ---
t_p_start = T_pad + 4*block_dur;
t_p_end   = t_p_start + T_step;
[rmse_zp_nom, rmse_zp_rls] = calc_leakage(t_ref, t_nom, t_rls, pos_ref(:,3), pos_nom(:,3), pos_rls(:,3), t_p_start, t_p_end);

%% 5. Print Metrics to Command Window
fprintf('\n================ RESULTS SUMMARY ================\n');
% Overall RMSE
fprintf('\n--- OVERALL MISSION PERFORMANCE (RMSE) ---\n');
fprintf('%-6s | %-10s | %-10s | %-12s\n', 'Axis', 'Nom RMSE', 'RLS RMSE', 'Improvement');
fprintf('%s\n', repmat('-', 1, 46));
for i = 1:6
    if i <= 3, col=i; ref_full=pos_ref(:,col); nom_full=pos_nom(:,col); rls_full=pos_rls(:,col);
    else, col=i-3;    ref_full=att_ref(:,col); nom_full=att_nom(:,col); rls_full=att_rls(:,col);
    end
    nom_i = interp1(t_nom, nom_full, t_ref, 'linear', 'extrap');
    rls_i = interp1(t_rls, rls_full, t_ref, 'linear', 'extrap');
    rmse_n = sqrt(mean((ref_full - nom_i).^2));
    rmse_r = sqrt(mean((ref_full - rls_i).^2));
    fprintf('%-6s | %10.4f | %10.4f | %10.1f%%\n', axis_names{i}, rmse_n, rmse_r, (rmse_n-rmse_r)/rmse_n*100);
end

% Step-Specific RMSE
fprintf('\n--- STEP-SPECIFIC PERFORMANCE (RMSE) ---\n');
fprintf('%-6s | %-10s | %-10s | %-12s\n', 'Axis', 'Nom RMSE', 'RLS RMSE', 'Improvement');
fprintf('%s\n', repmat('-', 1, 46));
for i = 1:6
    t_s = T_pad + (i-1)*block_dur; t_e = t_s + T_step; 
    if i <= 3, col=i; ref_f=pos_ref(:,col); nom_f=pos_nom(:,col); rls_f=pos_rls(:,col);
    else, col=i-3;    ref_f=att_ref(:,col); nom_f=att_nom(:,col); rls_f=att_rls(:,col);
    end
    [rmse_n, rmse_r] = calc_leakage(t_ref, t_nom, t_rls, ref_f, nom_f, rls_f, t_s, t_e);
    fprintf('%-6s | %10.4f | %10.4f | %10.1f%%\n', axis_names{i}, rmse_n, rmse_r, (rmse_n-rmse_r)/rmse_n*100);
end

% Leakage Metrics
fprintf('\n--- CROSS-COUPLING LEAKAGE METRICS ---\n');
fprintf('Z-Leakage during X-Step: Nominal: %.4fm | RLS: %.4fm | Imp: %.1f%%\n', ...
    rmse_zx_nom, rmse_zx_rls, (rmse_zx_nom-rmse_zx_rls)/rmse_zx_nom*100);
fprintf('Z-Leakage during Pitch-Step: Nominal: %.4fm | RLS: %.4fm | Imp: %.1f%%\n', ...
    rmse_zp_nom, rmse_zp_rls, (rmse_zp_nom-rmse_zp_rls)/rmse_zp_nom*100);
fprintf('=================================================\n');

%% 6. Figure 1: 6-DOF Comparison (Grid Layout)
fig1 = figure('Name', '6-DOF Comparison', 'Color', 'w', 'Position', [100 100 1400 800]);
labels = {'X [m]', 'Y [m]', 'Z [m]', 'Roll [rad]', 'Pitch [rad]', 'Yaw [rad]'};

for i = 1:6
    subplot(2, 3, i); hold on;
    if i<=3
        d_ref=pos_ref(:,i); d_nom=pos_nom(:,i); d_rls=pos_rls(:,i);
    else
        d_ref=att_ref(:,i-3); d_nom=att_nom(:,i-3); d_rls=att_rls(:,i-3);
    end
    % Apply bold line widths
    plot(t_ref, d_ref, 'k--', 'LineWidth', LW_ref);
    plot(t_nom, d_nom, 'r', 'LineWidth', LW_nom);
    plot(t_rls, d_rls, 'b', 'LineWidth', LW_rls);
    
    ylabel(labels{i}, 'FontSize', FS_axis); grid on; set(gca, 'FontSize', FS_axis);
    if i==2, title('Position Tracking', 'FontSize', FS_title); legend('Ref','Nom','RLS'); end
    if i==5, title('Attitude Tracking', 'FontSize', FS_title); end
    if i>3, xlabel('Time [s]', 'FontSize', FS_axis); end
    
    t_step_start = T_pad + (i-1)*block_dur;
    % HandleVisibility off hides these from the legend
    xline(t_step_start, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

%% 7. Figure 2: X-Axis Detailed Analysis
fig2 = figure('Name', 'Analysis: X-Axis & Pitch', 'Color', 'w', 'Position', [150 150 800 900]);
t_focus_start = 15; t_focus_end = 40; 

ax1 = subplot(3,1,1); hold on;
plot(t_ref, pos_ref(:,1), 'k--', 'LineWidth', LW_ref, 'DisplayName', 'Ref');
plot(t_nom, pos_nom(:,1), 'r', 'LineWidth', LW_nom, 'DisplayName', 'Nominal');
plot(t_rls, pos_rls(:,1), 'b', 'LineWidth', LW_rls, 'DisplayName', 'RLS');
ylabel('X Pos [m]', 'FontSize', FS_axis); title('1. Position Tracking (X-Axis)', 'FontSize', FS_title);
legend('Location', 'best'); grid on; set(gca, 'FontSize', FS_axis);

ax2 = subplot(3,1,2); hold on;
ref_i_nom = interp1(t_ref, att_ref(:,2), t_nom, 'linear', 'extrap');
ref_i_rls = interp1(t_ref, att_ref(:,2), t_rls, 'linear', 'extrap');
plot(t_nom, ref_i_nom - att_nom(:,2), 'r', 'LineWidth', LW_nom);
plot(t_rls, ref_i_rls - att_rls(:,2), 'b', 'LineWidth', LW_rls);
ylabel('Error [rad]', 'FontSize', FS_axis); title('2. Pitch Tracking Error', 'FontSize', FS_title);
grid on; set(gca, 'FontSize', FS_axis);

ax3 = subplot(3,1,3); hold on;
plot(t_nom, torq_nom(:,2), 'r', 'LineWidth', LW_nom);
plot(t_rls, torq_rls(:,2), 'b', 'LineWidth', LW_rls);
ylabel('Torque [Nm]', 'FontSize', FS_axis); xlabel('Time [s]', 'FontSize', FS_axis); 
title('3. Pitch Moment Effort', 'FontSize', FS_title);
grid on; set(gca, 'FontSize', FS_axis);

linkaxes([ax1, ax2, ax3], 'x'); xlim([t_focus_start, t_focus_end]);

%% 8. Figure 3: Leakage - X-Step vs Z-Axis
fig3 = figure('Name', 'Leakage: X-Step vs Z-Axis', 'Color', 'w', 'Position', [200 200 1000 700]);
t_leak_start = t_x_start - 5; t_leak_end = t_x_end + 15; 

axL1 = subplot(2,1,1); hold on;
plot(t_ref, pos_ref(:,1), 'k--', 'LineWidth', LW_ref, 'DisplayName', 'Ref');
plot(t_nom, pos_nom(:,1), 'r', 'LineWidth', LW_nom, 'DisplayName', 'Nominal');
plot(t_rls, pos_rls(:,1), 'b', 'LineWidth', LW_rls, 'DisplayName', 'RLS');
ylabel('X Position [m]', 'FontSize', FS_axis); title('Intended Motion: X-Axis Step', 'FontSize', FS_title);
grid on; legend('Location', 'best'); set(gca, 'FontSize', FS_axis);
% Hide xlines from legend
xline(t_x_start, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off'); 
xline(t_x_end, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');

axL2 = subplot(2,1,2); hold on;
plot(t_ref, pos_ref(:,3), 'k--', 'LineWidth', LW_ref);
plot(t_nom, pos_nom(:,3), 'r', 'LineWidth', LW_nom);
plot(t_rls, pos_rls(:,3), 'b', 'LineWidth', LW_rls);
ylabel('Z Position [m]', 'FontSize', FS_axis); xlabel('Time [s]', 'FontSize', FS_axis);
title(sprintf('Unintended Motion: Z-Leakage (Nom RMSE: %.4f, RLS RMSE: %.4f)', rmse_zx_nom, rmse_zx_rls), 'FontSize', FS_title);
grid on; set(gca, 'FontSize', FS_axis);

linkaxes([axL1, axL2], 'x'); xlim([t_leak_start, t_leak_end]);

%% 9. Figure 4: Leakage - Pitch-Step vs Z-Axis
fig4 = figure('Name', 'Leakage: Pitch-Step vs Z-Axis', 'Color', 'w', 'Position', [250 250 1000 700]);
t_leak_start = t_p_start - 5; t_leak_end = t_p_end + 15; 

axP1 = subplot(2,1,1); hold on;
plot(t_ref, att_ref(:,2), 'k--', 'LineWidth', LW_ref, 'DisplayName', 'Ref');
plot(t_nom, att_nom(:,2), 'r', 'LineWidth', LW_nom, 'DisplayName', 'Nominal');
plot(t_rls, att_rls(:,2), 'b', 'LineWidth', LW_rls, 'DisplayName', 'RLS');
ylabel('Pitch Angle [rad]', 'FontSize', FS_axis); title('Intended Motion: Pitch Step', 'FontSize', FS_title);
grid on; legend('Location', 'best'); set(gca, 'FontSize', FS_axis);
% Hide xlines from legend
xline(t_p_start, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off'); 
xline(t_p_end, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');

axP2 = subplot(2,1,2); hold on;
plot(t_ref, pos_ref(:,3), 'k--', 'LineWidth', LW_ref);
plot(t_nom, pos_nom(:,3), 'r', 'LineWidth', LW_nom);
plot(t_rls, pos_rls(:,3), 'b', 'LineWidth', LW_rls);
ylabel('Z Position [m]', 'FontSize', FS_axis); xlabel('Time [s]', 'FontSize', FS_axis);
title(sprintf('Unintended Motion: Z-Leakage (Nom RMSE: %.4f, RLS RMSE: %.4f)', rmse_zp_nom, rmse_zp_rls), 'FontSize', FS_title);
grid on; set(gca, 'FontSize', FS_axis);

linkaxes([axP1, axP2], 'x'); xlim([t_leak_start, t_leak_end]);

%% --- HELPER FUNCTIONS ---
function [rmse_nom, rmse_rls] = calc_leakage(t_ref, t_nom, t_rls, ref_full, nom_full, rls_full, t_s, t_e)
    idx_ref = t_ref >= t_s & t_ref <= t_e;
    idx_nom = t_nom >= t_s & t_nom <= t_e;
    idx_rls = t_rls >= t_s & t_rls <= t_e;
    ref_seg = ref_full(idx_ref); nom_seg = nom_full(idx_nom); rls_seg = rls_full(idx_rls);
    t_seg   = t_ref(idx_ref);
    if isempty(t_seg), rmse_nom=0; rmse_rls=0; return; end
    nom_i = interp1(t_nom(idx_nom), nom_seg, t_seg, 'linear', 'extrap');
    rls_i = interp1(t_rls(idx_rls), rls_seg, t_seg, 'linear', 'extrap');
    rmse_nom = sqrt(mean((ref_seg - nom_i).^2));
    rmse_rls = sqrt(mean((ref_seg - rls_i).^2));
end

function [t, pos, att, torq] = extract_run(simOut, name_states, name_torques)
    ts_states = find_signal(simOut, name_states);
    if isempty(ts_states), error('States not found in simOut'); end
    [t, data] = sanitize_data(ts_states);
    pos = data(:, 1:3); att = data(:, 7:9);
    ts_torq = find_signal(simOut, name_torques);
    if isempty(ts_torq), torq = zeros(length(t), 3); else
        [t_torq, raw_torq] = sanitize_data(ts_torq);
        if length(t_torq) ~= length(t), torq = interp1(t_torq, raw_torq, t, 'linear', 'extrap'); else, torq = raw_torq; end
    end
end

function [t, d] = sanitize_data(ts, indices)
    t = ts.Time; raw = squeeze(ts.Data);
    [R, C] = size(raw); if R ~= length(t) && C == length(t), raw = raw'; end
    if nargin > 1, max_col = size(raw, 2); valid = indices(indices <= max_col); d = raw(:, valid); else, d = raw; end
end

function ts = find_signal(simOut, signalNames)
    if ~iscell(signalNames), signalNames = {signalNames}; end
    ts = [];
    for k = 1:length(signalNames), name = signalNames{k};
        if isa(simOut, 'Simulink.SimulationOutput')
            if isprop(simOut, name), prop = simOut.get(name); if isa(prop,'timeseries'), ts=prop; return; end; if isa(prop,'struct'), ts=timeseries(prop.signals.values,prop.time); return; end; end
            dSets = {'logsout','yout'}; for d=1:2, if isprop(simOut, dSets{d}), ds = simOut.get(dSets{d}); if isa(ds,'Simulink.SimulationData.Dataset'), el = ds.get(name); if ~isempty(el), ts=el.Values; return; end; end; end; end
        end
    end
end