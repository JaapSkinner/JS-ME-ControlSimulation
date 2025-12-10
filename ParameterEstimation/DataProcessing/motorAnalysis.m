%% MASTER ANALYSIS: 4-PARAM UKF (Accuracy, Variance & Metrics)
clear; clc; close all;

%% --- CONFIGURATION ---
ROTOR_ID = 1;           % Index of rotor to analyze for step/sweep
RLS_SAMPLE_TIME = 15.0; % Time point to extract RLS parameters
STEP_MAGNITUDE = 0.7;   % PWM level for the Transient Step Test

% Timing Config
INIT_DURATION = 0.5;    % Time to sit at 0 before stepping
STEP_DURATION = 0.3;    % Time to record AFTER the step
DT_SIM = 0.0005;        % Higher precision for the 0.1s rise time

% Sweep Config
SWEEP_POINTS = 20;      % Number of points from 0.0 to 1.0
SWEEP_SETTLE = 2.0;     % Time to settle at each point

% Colors
c.true = [0.2 0.2 0.2];          % Dark Grey
c.ukf  = [0.0000 0.4470 0.7410]; % Blue
c.rls  = [0.4660 0.6740 0.1880]; % Green

%% --- 1. Load Data ---
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end

fprintf('Step 1: Select UKF File...\n');
[ukfFile, ukfPath] = uigetfile(fullfile(startPath, 'estimation_*.mat'), 'Select UKF File');
if isequal(ukfFile,0), return; end
S_ukf = load(fullfile(ukfPath, ukfFile));

fprintf('Step 2: Select RLS File...\n');
[rlsFile, rlsPath] = uigetfile(fullfile(ukfPath, '*.mat'), 'Select RLS File');
if isequal(rlsFile,0), return; end
S_rls = load(fullfile(rlsPath, rlsFile));

% 1.1 Extract Ground Truth
Motor_True = S_ukf.Motor;
N_ROTORS = S_ukf.Uav.N_ROTORS;

% 1.2 Extract UKF Parameters
ukfData = S_ukf.simOut.UKFData.UKF_DATA;
t_end = ukfData.Time(end);
subset = getsampleusingtime(ukfData, t_end-5, t_end).Data;
params_avg = mean(subset, 1);

% --- PARAMETER EXTRACTION (4-PARAM MODEL) ---
total_cols = size(params_avg, 2);
idx_start_params = total_cols - (4 * N_ROTORS) + 1;
idx.Gu   = idx_start_params : idx_start_params + N_ROTORS - 1;
idx.Cw   = idx.Gu(end) + 1  : idx.Gu(end) + N_ROTORS;
idx.Cw2  = idx.Cw(end) + 1  : idx.Cw(end) + N_ROTORS;
idx.Bias = idx.Cw2(end) + 1 : idx.Cw2(end) + N_ROTORS;

% Extract specific rotor for simulation
UKF.Gu   = params_avg(idx.Gu(ROTOR_ID));
UKF.Cw   = params_avg(idx.Cw(ROTOR_ID));
UKF.Cw2  = params_avg(idx.Cw2(ROTOR_ID));
UKF.Bias = params_avg(idx.Bias(ROTOR_ID));

% 1.3 Extract RLS Parameters
rlsData = S_rls.simOut.RLSData.MotorParams; 
[~, idx_t] = min(abs(rlsData.Time - RLS_SAMPLE_TIME));
Theta = rlsData.Data(:, ROTOR_ID, idx_t);
RLS.t1 = Theta(1); RLS.t2 = Theta(2); RLS.t3 = Theta(3); RLS.t4 = Theta(4);

%% --- 2. Run Simulations ---

% 2.1 Transient Simulation (Step)
t_total = INIT_DURATION + STEP_DURATION;
t_trans = 0:DT_SIM:t_total; 
u_trans = zeros(size(t_trans));
u_trans(t_trans >= INIT_DURATION) = STEP_MAGNITUDE;

[w_true_t, w_ukf_t, w_rls_t] = run_sim_vector(t_trans, u_trans, Motor_True, UKF, RLS, ROTOR_ID);

% 2.2 Steady State Sweep
pwm_levels = linspace(0, 1.0, SWEEP_POINTS);
ss_true = zeros(size(pwm_levels));
ss_ukf  = zeros(size(pwm_levels));
ss_rls  = zeros(size(pwm_levels));
t_sweep = 0:0.01:SWEEP_SETTLE; 
u_const = ones(size(t_sweep)); 

for i = 1:length(pwm_levels)
    u_val = pwm_levels(i);
    u_vec = u_const * u_val;
    [wt, wu, wr] = run_sim_vector(t_sweep, u_vec, Motor_True, UKF, RLS, ROTOR_ID);
    ss_true(i) = wt(end);
    ss_ukf(i)  = wu(end);
    ss_rls(i)  = wr(end);
end

%% --- 3. CALCULATE METRICS ---

% 3.1 Transient Metrics
idx_step = find(t_trans >= INIT_DURATION, 1);
w_true_cut = w_true_t(idx_step:end);
w_ukf_cut  = w_ukf_t(idx_step:end);
w_rls_cut  = w_rls_t(idx_step:end);
time_cut   = t_trans(idx_step:end) - t_trans(idx_step);

% A. Rise Time
rt_true = get_rise_time(time_cut, w_true_cut);
rt_ukf  = get_rise_time(time_cut, w_ukf_cut);
rt_rls  = get_rise_time(time_cut, w_rls_cut);

% B. Transient RMSE
rmse_trans_ukf = sqrt(mean((w_true_cut - w_ukf_cut).^2));
rmse_trans_rls = sqrt(mean((w_true_cut - w_rls_cut).^2));

% 3.2 Steady State Metrics
% A. Mapping RMSE
rmse_ss_ukf = sqrt(mean((ss_true - ss_ukf).^2));
rmse_ss_rls = sqrt(mean((ss_true - ss_rls).^2));

% B. Idle Error
idle_err_ukf = abs(ss_true(1) - ss_ukf(1));
idle_err_rls = abs(ss_true(1) - ss_rls(1));

%% --- 4. PLOTTING ---

% FIG 1: Transient Response
figure('Name', 'Zoomed Step Response', 'Position', [100 200 600 500]);
hold on;
plot(t_trans, w_true_t, 'k-', 'LineWidth', 2.5, 'DisplayName', 'True');
plot(t_trans, w_ukf_t, '-', 'Color', c.ukf, 'LineWidth', 2, 'DisplayName', 'UKF');
plot(t_trans, w_rls_t, '-', 'Color', c.rls, 'LineWidth', 2, 'DisplayName', 'RLS');
xline(INIT_DURATION, 'k:', 'Step Input', 'HandleVisibility', 'off');
grid on;
xlim([INIT_DURATION - 0.1, INIT_DURATION + 0.2]); 
ylabel('Omega (rad/s)'); xlabel('Time (s)');
title({ 'Transient Zoom (4-Param UKF)', ...
        sprintf('RMSE: UKF=%.1f | Rise Time Err: %.1f ms', rmse_trans_ukf, (rt_ukf-rt_true)*1000) });
legend('Location', 'Southeast');

% FIG 2: Steady State Mapping
figure('Name', 'Steady State Mapping', 'Position', [750 200 600 500]);
plot(pwm_levels, ss_true, 'k-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'True Physics'); hold on;
plot(pwm_levels, ss_ukf,  '-s', 'Color', c.ukf, 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'UKF Model');
plot(pwm_levels, ss_rls,  '-^', 'Color', c.rls, 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'RLS Model');
grid on;
xlabel('Input PWM (0.0 - 1.0)');
ylabel('Steady State Speed (rad/s)');
title({ 'Steady State Mapping', ...
        sprintf('RMSE: UKF=%.1f | Idle Err: %.1f rad/s', rmse_ss_ukf, idle_err_ukf) });
legend('Location', 'Southeast');

% FIG 3: Convergence
figure('Name', 'UKF Parameter Convergence', 'Position', [150 150 1200 400]);
t = tiledlayout(1, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'UKF Motor Parameter Convergence (All Rotors)');
time_vec = ukfData.Time;

nexttile; plot(time_vec, ukfData.Data(:, idx.Gu), 'LineWidth', 1.2);
grid on; ylabel('Gain (p1)'); title('Input Gain (Gu)');
nexttile; plot(time_vec, ukfData.Data(:, idx.Cw), 'LineWidth', 1.2);
grid on; ylabel('Damping (p2)'); title('Linear Damping (Cw)');
nexttile; plot(time_vec, ukfData.Data(:, idx.Cw2), 'LineWidth', 1.2);
grid on; ylabel('Drag (p3)'); title('Quadratic Drag (Cw2)');
nexttile; plot(time_vec, ukfData.Data(:, idx.Bias), 'LineWidth', 1.2);
grid on; ylabel('Bias (p4)'); title('Bias Force');
nexttile(1); 
leg_str = cell(1, N_ROTORS);
for i=1:N_ROTORS, leg_str{i} = sprintf('M%d', i); end
legend(leg_str, 'Location', 'best', 'NumColumns', 2);

% FIG 4: Multi-Motor Variance Analysis
% Calculate TRUE params for all motors
True_Gu = zeros(1, N_ROTORS); True_Bias = zeros(1, N_ROTORS);
True_Cw = zeros(1, N_ROTORS); True_Cw2 = zeros(1, N_ROTORS);

for i = 1:N_ROTORS
    Kt = get_param(Motor_True, 'K_T', i);
    Ke = get_param(Motor_True, 'K_E', i);
    R  = get_param(Motor_True, 'R', i);
    Ctau = get_param(Motor_True, 'C_TAU', i);
    mv = get_param(Motor_True, 'volt_slope', i);
    Voff = get_param(Motor_True, 'Volt_offset', i);
    I0 = get_param(Motor_True, 'I_0', i);
    Irzz = get_param(Motor_True, 'I_R_ZZ', i);
    
    True_Gu(i)  = (Kt * mv) / (Irzz * R);
    V_net       = Voff - (I0 * R);
    True_Bias(i) = (Kt / (Irzz * R)) * V_net;
    True_Cw(i)  = (Kt * Ke) / (Irzz * R);
    True_Cw2(i) = Ctau / Irzz;
end

Est_Gu   = params_avg(idx.Gu); Est_Bias = params_avg(idx.Bias);
Est_Cw   = params_avg(idx.Cw); Est_Cw2  = params_avg(idx.Cw2);

figure('Name', 'Multi-Motor Variance Analysis', 'Position', [100 100 1200 800]);
t = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, sprintf('Manufacturing Variance Tracking (Across %d Motors)', N_ROTORS));

nexttile; bar([True_Gu', Est_Gu'], 'grouped'); ylabel('Gain'); title('Gain Variance');
legend({'True Physics', 'UKF Estimate'}, 'Location', 'best');
nexttile; bar([True_Bias', Est_Bias'], 'grouped'); ylabel('Bias (rad/s^2)'); title('Idle Force Variance');
nexttile; bar([True_Cw', Est_Cw'], 'grouped'); ylabel('Damping'); title('Linear Damping Variance');
nexttile; bar([True_Cw2', Est_Cw2'], 'grouped'); ylabel('Drag'); title('Quad Drag Variance');

%% --- 5. PRINT METRICS TABLE ---
fprintf('\n===============================================\n');
fprintf('     QUANTITATIVE PERFORMANCE COMPARISON       \n');
fprintf('===============================================\n');
fprintf('Metric                     | UKF (Phys) | RLS (Curve)\n');
fprintf('---------------------------|------------|------------\n');
fprintf('Transient RMSE (rad/s)     | %10.2f | %10.2f\n', rmse_trans_ukf, rmse_trans_rls);
fprintf('Rise Time Error (ms)       | %10.1f | %10.1f\n', (rt_ukf-rt_true)*1000, (rt_rls-rt_true)*1000);
fprintf('Steady State RMSE (rad/s)  | %10.2f | %10.2f\n', rmse_ss_ukf, rmse_ss_rls);
fprintf('Idle Speed Error (rad/s)   | %10.2f | %10.2f\n', idle_err_ukf, idle_err_rls);
fprintf('-----------------------------------------------\n');
fprintf('True Rise Time: %.1f ms\n', rt_true*1000);
fprintf('===============================================\n');

%% --- Helper Functions ---
function rt = get_rise_time(t, y)
    final_val = y(end); start_val = y(1);
    range = final_val - start_val;
    val_10 = start_val + 0.10 * range;
    val_90 = start_val + 0.90 * range;
    t_10 = t(find(y >= val_10, 1));
    t_90 = t(find(y >= val_90, 1));
    if isempty(t_10) || isempty(t_90), rt = NaN; else, rt = t_90 - t_10; end
end

function [w_true, w_ukf, w_rls] = run_sim_vector(t, u_vec, M_struct, UKF, RLS, rid)
    dt = t(2)-t(1);
    N = length(t);
    w_true = zeros(1,N); w_ukf = zeros(1,N); w_rls = zeros(1,N);
    M = extractMotorScalar(M_struct, rid);
    for k = 1:N-1
        u = u_vec(k);
        % True
        Vi = M.volt_slope * u + M.Volt_offset;
        term_elec = (M.K_T/M.R)*(Vi - M.I_0*M.R - w_true(k)*M.K_E);
        term_drag = M.C_TAU * w_true(k)^2;
        w_true(k+1) = max(0, w_true(k) + (1/M.I_R_ZZ)*(term_elec - term_drag)*dt);
        % UKF
        dot_u = UKF.Gu*u + UKF.Bias - abs(UKF.Cw)*w_ukf(k) - abs(UKF.Cw2)*w_ukf(k)^2;
        w_ukf(k+1) = max(0, w_ukf(k) + dot_u*dt);
        % RLS
        num = RLS.t1*u + RLS.t2*sqrt(max(0,u)) + RLS.t3 - w_rls(k);
        den = max(abs(RLS.t4), 1e-6) * sign(RLS.t4);
        w_rls(k+1) = max(0, w_rls(k) + (num/den)*dt);
    end
end

function M_out = extractMotorScalar(M_in, idx)
    fields = fieldnames(M_in);
    for i = 1:length(fields)
        f = fields{i}; val = M_in.(f);
        if isnumeric(val) && (size(val,1)==1 || size(val,2)==1) && numel(val)>=idx
            M_out.(f) = val(idx);
        else
            M_out.(f) = val;
        end
    end
end

function val = get_param(M_struct, field, idx)
    raw = M_struct.(field);
    if length(raw) == 1, val = raw; else, val = raw(idx); end
end

function ts_out = getsampleusingtime(ts_in, t_start, t_end)
    if nargin < 3, t_end = t_start; end
    mask = (ts_in.Time >= t_start) & (ts_in.Time <= t_end);
    raw_data = ts_in.Data;
    ts_out = timeseries(raw_data(mask, :), ts_in.Time(mask));
end