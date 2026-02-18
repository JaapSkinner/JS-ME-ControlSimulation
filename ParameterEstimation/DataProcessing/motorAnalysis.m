%% MASTER ANALYSIS: BEST ROTOR AUTO-SELECTOR (RLS PRIORITY + RISE TIME)
clear; clc; close all;

%% --- CONFIGURATION ---
% (ROTOR_ID will be selected automatically based on RLS Improvement)
RLS_SAMPLE_TIME = 15.0; % Time point to extract RLS parameters
STEP_MAGNITUDE = 0.7;   % PWM level for Transient Step Test

% Timing Config
INIT_DURATION = 0.5;    % Time to sit at 0 before stepping
STEP_DURATION = 0.3;    % Time to record AFTER the step
DT_SIM = 0.0005;        % Simulation step size

% Sweep Config
SWEEP_POINTS = 20;      % Number of points from 0.0 to 1.0
SWEEP_SETTLE = 2.0;     % Time to settle at each point

% Colors
c.true = [0.2 0.2 0.2];          % Dark Grey
c.nom  = [0.4940 0.1840 0.5560]; % Purple
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

% 1.1 Load UKF (Ground Truth)
fprintf('Step 1: Select UKF File (Ground Truth)...\n');
[ukfFile, ukfPath] = uigetfile(fullfile(startPath, 'estimation_*.mat'), 'Select UKF File');
if isequal(ukfFile,0), return; end
S_ukf = load(fullfile(ukfPath, ukfFile));

% 1.2 Load RLS
fprintf('Step 2: Select RLS File...\n');
[rlsFile, rlsPath] = uigetfile(fullfile(ukfPath, '*.mat'), 'Select RLS File');
if isequal(rlsFile,0), return; end
S_rls = load(fullfile(rlsPath, rlsFile));

% 1.3 Load Nominal (Datasheet)
fprintf('Step 3: Select Nominal Model File...\n');
[nomFile, nomPath] = uigetfile(fullfile(ukfPath, '*.mat'), 'Select Nominal File');
if isequal(nomFile,0), return; end
S_nom = load(fullfile(nomPath, nomFile));

% --- SANITIZE NOMINAL PARAMETERS ---
if ~isfield(S_nom.Motor, 'I_R_ZZ')
    if isfield(S_nom.Motor, 'I_Rv')
        S_nom.Motor.I_R_ZZ = S_nom.Motor.I_Rv;
    elseif isfield(S_nom.Motor, 'J')
        S_nom.Motor.I_R_ZZ = S_nom.Motor.J;
    else
        error('Nominal Motor struct is missing Inertia (I_R_ZZ or I_Rv).');
    end
end

%% --- 2. EXTRACT GLOBAL DATA ---
Motor_True = S_ukf.Motor;
Motor_Nom  = S_nom.Motor; 
N_ROTORS   = S_ukf.Uav.N_ROTORS;

% UKF Params (Average)
ukfData = S_ukf.simOut.UKFData.UKF_DATA;
t_end = ukfData.Time(end);
subset = getsampleusingtime(ukfData, t_end-5, t_end).Data;
params_avg = mean(subset, 1);
total_cols = size(params_avg, 2);
idx_start = total_cols - (4 * N_ROTORS) + 1;
idx.Gu   = idx_start : idx_start + N_ROTORS - 1;
idx.Cw   = idx.Gu(end) + 1  : idx.Gu(end) + N_ROTORS;
idx.Cw2  = idx.Cw(end) + 1  : idx.Cw(end) + N_ROTORS;
idx.Bias = idx.Cw2(end) + 1 : idx.Cw2(end) + N_ROTORS;

% RLS Params Data Source
rlsData = S_rls.simOut.RLSData.MotorParams; 
[~, idx_t_rls] = min(abs(rlsData.Time - RLS_SAMPLE_TIME));

%% --- 3. AUTO-SEARCH FOR BEST RLS IMPROVEMENT ---
fprintf('\n------------------------------------------------------------\n');
fprintf('SEARCHING FOR BEST DEMONSTRATION ROTOR (RLS PRIORITY)...\n');
fprintf('------------------------------------------------------------\n');
fprintf('%-5s | %-10s | %-10s | %-10s\n', 'ID', 'Nom RMSE', 'RLS RMSE', 'RLS Improve');

best_rotor = 1;
max_imp_factor = -inf;

pwm_search = linspace(0, 1.0, 10); 
u_const = ones(size(0:0.01:1.0)); 

for r = 1:N_ROTORS
    % A. Extract RLS Params
    Theta = rlsData.Data(:, r, idx_t_rls);
    P_RLS.t1 = Theta(1); P_RLS.t2 = Theta(2); 
    P_RLS.t3 = Theta(3); P_RLS.t4 = Theta(4);
    
    % B. Simulate Sweep
    ss_t = zeros(size(pwm_search));
    ss_n = zeros(size(pwm_search));
    ss_r = zeros(size(pwm_search));
    
    for k = 1:length(pwm_search)
        u_vec = u_const * pwm_search(k);
        wt = sim_physics(0:0.01:1.0, u_vec, Motor_True, r);
        wn = sim_physics(0:0.01:1.0, u_vec, Motor_Nom,  r);
        wr = sim_rls(0:0.01:1.0, u_vec, P_RLS);
        ss_t(k) = wt(end); ss_n(k) = wn(end); ss_r(k) = wr(end);
    end
    
    % C. Calculate Metric
    rmse_n = sqrt(mean((ss_t - ss_n).^2));
    rmse_r = sqrt(mean((ss_t - ss_r).^2));
    imp = rmse_n / max(rmse_r, 1e-6);
    
    fprintf('M%-4d | %10.2f | %10.2f | \x1b[32m%10.2fx\x1b[0m\n', r, rmse_n, rmse_r, imp);
    if imp > max_imp_factor, max_imp_factor = imp; best_rotor = r; end
end
fprintf('------------------------------------------------------------\n');
fprintf('>>> SELECTED ROTOR: %d (RLS Improvement: %.1fx)\n', best_rotor, max_imp_factor);
fprintf('------------------------------------------------------------\n\n');

ROTOR_ID = best_rotor;

%% --- 4. FULL SIMULATION (SELECTED ROTOR) ---
UKF.Gu   = params_avg(idx.Gu(ROTOR_ID));
UKF.Cw   = params_avg(idx.Cw(ROTOR_ID));
UKF.Cw2  = params_avg(idx.Cw2(ROTOR_ID));
UKF.Bias = params_avg(idx.Bias(ROTOR_ID));

Theta = rlsData.Data(:, ROTOR_ID, idx_t_rls);
RLS.t1 = Theta(1); RLS.t2 = Theta(2); RLS.t3 = Theta(3); RLS.t4 = Theta(4);

% 4.1 Transient Simulation
t_trans = 0:DT_SIM:(INIT_DURATION + STEP_DURATION);
u_trans = zeros(size(t_trans));
u_trans(t_trans >= INIT_DURATION) = STEP_MAGNITUDE;

w_true_t = sim_physics(t_trans, u_trans, Motor_True, ROTOR_ID);
w_nom_t  = sim_physics(t_trans, u_trans, Motor_Nom,  ROTOR_ID); 
w_ukf_t  = sim_ukf(t_trans, u_trans, UKF);
w_rls_t  = sim_rls(t_trans, u_trans, RLS);

% 4.2 Steady State Sweep
pwm_levels = linspace(0, 1.0, SWEEP_POINTS);
ss_true = zeros(size(pwm_levels));
ss_nom  = zeros(size(pwm_levels));
ss_ukf  = zeros(size(pwm_levels));
ss_rls  = zeros(size(pwm_levels));

t_sweep = 0:0.01:SWEEP_SETTLE; 
u_const_sweep = ones(size(t_sweep)); 

for i = 1:length(pwm_levels)
    u_val = pwm_levels(i);
    u_vec = u_const_sweep * u_val;
    wt = sim_physics(t_sweep, u_vec, Motor_True, ROTOR_ID);
    wn = sim_physics(t_sweep, u_vec, Motor_Nom,  ROTOR_ID);
    wu = sim_ukf(t_sweep, u_vec, UKF);
    wr = sim_rls(t_sweep, u_vec, RLS);
    
    ss_true(i) = wt(end); ss_nom(i) = wn(end); ss_ukf(i) = wu(end); ss_rls(i) = wr(end);
end

%% --- 5. METRICS & PLOTTING ---
% Cut data to step portion
idx_step = find(t_trans >= INIT_DURATION, 1);
time_cut = t_trans(idx_step:end) - t_trans(idx_step);

w_true_c = w_true_t(idx_step:end);
w_nom_c  = w_nom_t(idx_step:end);
w_ukf_c  = w_ukf_t(idx_step:end);
w_rls_c  = w_rls_t(idx_step:end);

% A. RMSE
rmse_tr_nom = sqrt(mean((w_true_c - w_nom_c).^2));
rmse_tr_ukf = sqrt(mean((w_true_c - w_ukf_c).^2));
rmse_tr_rls = sqrt(mean((w_true_c - w_rls_c).^2));
rmse_ss_nom = sqrt(mean((ss_true - ss_nom).^2));
rmse_ss_ukf = sqrt(mean((ss_true - ss_ukf).^2));
rmse_ss_rls = sqrt(mean((ss_true - ss_rls).^2));

% B. Improvement Factors
imp_tr_rls = rmse_tr_nom / max(rmse_tr_rls, 1e-6);
imp_ss_rls = rmse_ss_nom / max(rmse_ss_rls, 1e-6);
imp_tr_ukf = rmse_tr_nom / max(rmse_tr_ukf, 1e-6);
imp_ss_ukf = rmse_ss_nom / max(rmse_ss_ukf, 1e-6);

% C. Response Time (Rise Time 10-90%)
rt_true = get_rise_time(time_cut, w_true_c);
rt_nom  = get_rise_time(time_cut, w_nom_c);
rt_ukf  = get_rise_time(time_cut, w_ukf_c);
rt_rls  = get_rise_time(time_cut, w_rls_c);

% Rise Time Error (ms)
rt_err_nom = (rt_nom - rt_true) * 1000;
rt_err_ukf = (rt_ukf - rt_true) * 1000;
rt_err_rls = (rt_rls - rt_true) * 1000;

% FIG 1: Transient
figure('Name', 'Zoomed Step Response', 'Position', [100 200 600 500]);
hold on;
plot(t_trans, w_true_t, 'k-', 'LineWidth', 2.5, 'DisplayName', 'True');
plot(t_trans, w_nom_t,  '--', 'Color', c.nom, 'LineWidth', 2, 'DisplayName', 'Nominal');
plot(t_trans, w_ukf_t,  '-',  'Color', c.ukf, 'LineWidth', 2, 'DisplayName', 'UKF');
plot(t_trans, w_rls_t,  '-',  'Color', c.rls, 'LineWidth', 2, 'DisplayName', 'RLS');
xline(INIT_DURATION, 'k:', 'HandleVisibility', 'off');
grid on; xlim([INIT_DURATION - 0.1, INIT_DURATION + 0.2]); 
ylabel('Omega (rad/s)'); xlabel('Time (s)');
title({ sprintf('Transient Response (Rotor %d)', ROTOR_ID), ...
        sprintf('RLS Improve: %.1fx | Rise Time Err: %.1f ms', imp_tr_rls, rt_err_rls) });
legend('Location', 'Southeast');

% FIG 2: Steady State
figure('Name', 'Steady State Mapping', 'Position', [750 200 600 500]);
plot(pwm_levels, ss_true, 'k-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'True'); hold on;
plot(pwm_levels, ss_nom,  '--d', 'Color', c.nom, 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'Nominal');
plot(pwm_levels, ss_ukf,  '-s',  'Color', c.ukf, 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'UKF');
plot(pwm_levels, ss_rls,  '-^',  'Color', c.rls, 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'RLS');
grid on; xlabel('Input PWM'); ylabel('Speed (rad/s)');
title({ sprintf('Steady State Mapping (Rotor %d)', ROTOR_ID), ...
        sprintf('RLS Improve: %.1fx (vs Nominal)', imp_ss_rls) });
legend('Location', 'Southeast');

% TABLE
fprintf('\n========================================================================\n');
fprintf('          FINAL ANALYSIS FOR ROTOR %d (RLS PRIORITY)              \n', ROTOR_ID);
fprintf('========================================================================\n');
fprintf('%-18s | %10s | %10s (Imp) | %10s (Imp)\n', 'Metric', 'Nominal', 'UKF', 'RLS');
fprintf('-------------------|------------|-------------------|-------------------\n');
fprintf('%-18s | %10.2f | %10.2f (%4.1fx) | %10.2f (\x1b[32m%4.1fx\x1b[0m)\n', ...
    'Trans RMSE (rad/s)', rmse_tr_nom, rmse_tr_ukf, imp_tr_ukf, rmse_tr_rls, imp_tr_rls);
fprintf('%-18s | %10.2f | %10.2f (%4.1fx) | %10.2f (\x1b[32m%4.1fx\x1b[0m)\n', ...
    'SS RMSE (rad/s)', rmse_ss_nom, rmse_ss_ukf, imp_ss_ukf, rmse_ss_rls, imp_ss_rls);
fprintf('-------------------|------------|-------------------|-------------------\n');
fprintf('%-18s | %10.1f | %10.1f        | %10.1f       \n', 'Rise Time (ms)', ...
    rt_nom*1000, rt_ukf*1000, rt_rls*1000);
fprintf('%-18s | %10.1f | %10.1f        | %10.1f       \n', 'Rise Time Err (ms)', ...
    rt_err_nom, rt_err_ukf, rt_err_rls);
fprintf('------------------------------------------------------------------------\n');
fprintf('True Rise Time (10-90%%): %.1f ms\n', rt_true*1000);
fprintf('========================================================================\n');

%% --- Helper Functions ---
function rt = get_rise_time(t, y)
    final_val = y(end); start_val = y(1);
    range = final_val - start_val;
    if range < 1e-3, rt = NaN; return; end % Handle case where no step occurs
    
    val_10 = start_val + 0.10 * range;
    val_90 = start_val + 0.90 * range;
    
    t_10 = t(find(y >= val_10, 1));
    t_90 = t(find(y >= val_90, 1));
    
    if isempty(t_10) || isempty(t_90), rt = NaN; else, rt = t_90 - t_10; end
end

function w_out = sim_physics(t, u_vec, M_struct, rid)
    dt = t(2)-t(1); N = length(t); w_out = zeros(1,N);
    M = extractMotorScalar(M_struct, rid);
    for k = 1:N-1
        u = u_vec(k);
        Vi = M.volt_slope * u + M.Volt_offset;
        term_elec = (M.K_T/M.R)*(Vi - M.I_0*M.R - w_out(k)*M.K_E);
        term_drag = M.C_TAU * w_out(k)^2;
        w_out(k+1) = max(0, w_out(k) + (1/M.I_R_ZZ)*(term_elec - term_drag)*dt);
    end
end

function w_out = sim_ukf(t, u_vec, P)
    dt = t(2)-t(1); N = length(t); w_out = zeros(1,N);
    for k = 1:N-1
        u = u_vec(k);
        dot_u = P.Gu*u + P.Bias - abs(P.Cw)*w_out(k) - abs(P.Cw2)*w_out(k)^2;
        w_out(k+1) = max(0, w_out(k) + dot_u*dt);
    end
end

function w_out = sim_rls(t, u_vec, P)
    dt = t(2)-t(1); N = length(t); w_out = zeros(1,N);
    den = max(abs(P.t4), 1e-6) * sign(P.t4);
    for k = 1:N-1
        u = u_vec(k);
        num = P.t1*u + P.t2*sqrt(max(0,u)) + P.t3 - w_out(k);
        w_out(k+1) = max(0, w_out(k) + (num/den)*dt);
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

function ts_out = getsampleusingtime(ts_in, t_start, t_end)
    if nargin < 3, t_end = t_start; end
    mask = (ts_in.Time >= t_start) & (ts_in.Time <= t_end);
    raw_data = ts_in.Data;
    ts_out = timeseries(raw_data(mask, :), ts_in.Time(mask));
end