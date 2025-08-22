% =========================================================================
% ADVANCED SENSITIVITY ANALYSIS SINGLE (v1 - w/ Setpoint Guide)
% =========================================================================
% This script analyzes the effect of parameter variations on the performance
% of a SINGLE, user-specified setpoint. This helps to isolate relationships
% that might be hidden when averaging across all maneuvers.
%
clear; clc; close all;

% --- User Settings ---
% =========================================================================
% >>> CHOOSE THE SETPOINT TO ANALYZE HERE <<<
%
% SETPOINT GUIDE: [Fx, Fy, Fz, Tx, Ty, Tz] (Normalized Commands)
% --- Single-Axis Positive Tests ---
%  1: Strong forward thrust (Fx=0.7)
%  2: Strong right thrust (Fy=0.7)
%  3: High vertical thrust (Fz=0.9)
%  4: Strong roll right torque (Tx=0.7)
%  5: Strong pitch forward torque (Ty=0.7)
%  6: Strong yaw right torque (Tz=0.7)
% --- Single-Axis Negative Tests ---
%  7: Strong backward thrust (Fx=-0.7)
%  8: Strong left thrust (Fy=-0.7)
%  9: Low vertical thrust (Fz=0.1)
% 10: Strong roll left torque (Tx=-0.7)
% 11: Strong pitch backward torque (Ty=-0.7)
% 12: Strong yaw left torque (Tz=-0.7)
% --- Combination Tests ---
% 13: Forward flight (Fx=0.5, Ty=0.5)
% 14: Rolling while moving left (Fy=-0.5, Tx=0.5)
% 15: High thrust climb with yaw (Fz=0.8, Tz=0.5)
% 16: Complex maneuver (multiple inputs)
setpoint_to_analyze = 1; % <--- CHANGE THIS VALUE
% =========================================================================


% --- 1. Load and Aggregate Data (This part is the same as before) ---
fprintf('Step 1: Loading all result files to build data table...\n');
run('ParameterEstimationBaseOL.m'); 
Motor_nom = Motor; Uav_nom = Uav; N_motors = Uav.N_ROTORS;
params_nom = struct();
u_min_nom = [-1.0;-1.0;0.0;-1.0;-1.0;-1.0]; u_max_nom = [1.0;1.0;1.0;1.0;1.0;1.0];
alpha_Tx_nom = Motor.CommandMixing(2)/10000; alpha_Ty_nom = Motor.CommandMixing(3)/10000; alpha_Tz_nom = Motor.CommandMixing(4)/10000;
P_nom = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 1 0 0 0; 1 0 0 0 0 0; 0 1 0 0 0 0];
A_scale_nom = diag([alpha_Tx_nom, alpha_Ty_nom, alpha_Tz_nom, 1, 1, 1]);
params_nom.A_eff = A_scale_nom * P_nom;
params_nom.u_min = u_min_nom; params_nom.u_max = u_max_nom; params_nom.M = Motor.mixingMatrix; params_nom.N = N_motors;
params_nom.Kt=Motor_nom.K_T; params_nom.Ke=Motor_nom.K_E; params_nom.R=Motor_nom.R; params_nom.C_tau=Motor_nom.C_TAU;
params_nom.mv=Motor_nom.volt_slope; params_nom.V_off=Motor_nom.Volt_offset; params_nom.I0=Motor_nom.I_0;
kf_vec_nom=0.5*Uav_nom.RHO_AIR*Aero.Cz3P.coefs(1)*(Uav_nom.D_PROP.^2).*Uav_nom.A_PROP; km_vec_nom=Motor_nom.C_TAU;
B_matrix_nom=zeros(6,N_motors);
for m=1:N_motors
    t_m = [0; 0; -kf_vec_nom(m)];
    tau_m = [0; 0; -Uav_nom.ROTOR_DIRECTION(m) * km_vec_nom(m)];
    R_m_b = Uav_nom.R_MOTOR_TO_BODY(:,:,m);
    T_body_i=R_m_b*t_m; tau_body_i=R_m_b*tau_m; r_i=Uav_nom.MotorLoc(m,1:3)';
    tau_from_thrust=cross(r_i,T_body_i); B_matrix_nom(:,m)=[T_body_i;tau_body_i+tau_from_thrust];
end
params_nom.B_matrix = B_matrix_nom;
setpoints_max_test=[1,0,0.5,0,0,0;-1,0,0.5,0,0,0;0,1,0.5,0,0,0;0,-1,0.5,0,0,0;0,0,1,0,0,0;0,0,0,0,0,0;0,0,0.5,1,0,0;0,0,0.5,-1,0,0;0,0,0.5,0,1,0;0,0,0.5,0,-1,0;0,0,0.5,0,0,1;0,0,0.5,0,0,-1];
results_max=zeros(6,size(setpoints_max_test,1));
for i=1:size(setpoints_max_test,1), results_max(:,i)=computeWrench(setpoints_max_test(i,:)',params_nom); end
max_authority = max(abs(results_max), [], 2);
max_authority(max_authority < 1e-6) = 1; 

resultsPath = uigetdir('', 'Select the Folder Containing Monte Carlo Results');
if isequal(resultsPath, 0), disp('User selected Cancel.'); return; end
resultFiles = dir(fullfile(resultsPath, 'simResult_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0, error('No simResult files found.'); end

% --- 2. Aggregate Data for the CHOSEN SETPOINT ---
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints;
nSetpoints = size(setpoints, 1);
if setpoint_to_analyze > nSetpoints || setpoint_to_analyze < 1
    error('Invalid setpoint_to_analyze. Please choose an index between 1 and %d.', nSetpoints);
end
fprintf('Analyzing performance for Setpoint #%d: [%s]\n', ...
    setpoint_to_analyze, num2str(setpoints(setpoint_to_analyze, :)));

is_single_axis = (sum(setpoints(setpoint_to_analyze, :)~=0,2)==1) | (sum(setpoints(setpoint_to_analyze, :)~=0,2)==2 & setpoints(setpoint_to_analyze, 3)~=0);

varNames = {'ErrorScore','LeakageScore','CoG_AbsDev_X','CoG_AbsDev_Y','CoG_AbsDev_Z','StdDev_KV','Range_KV','StdDev_R','Range_R','StdDev_I0','Range_I0','Mass_AbsDev','Inertia_AbsDev_Mag'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [numFiles, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);

for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    
    sp = setpoint_to_analyze;
    w_des = data.setpoints(sp, :)';
    w_actual = data.wrenches(:, sp);
    error_raw = w_actual - w_des;
    results_table.ErrorScore(i) = norm(error_raw ./ max_authority);
    if is_single_axis
        off_axis_indices = find(w_des == 0);
        norm_leakage = norm(w_actual(off_axis_indices));
        norm_des = norm(w_des);
        if norm_des > 1e-6, results_table.LeakageScore(i) = (norm_leakage / norm_des) * 100;
        else, results_table.LeakageScore(i) = NaN; end
    else, results_table.LeakageScore(i) = NaN; end
    
    if isfield(data.Sampled_features,'COM')&&isfield(data.Sampled_features.COM,'per_axis_deviation'), cog_dev=data.Sampled_features.COM.per_axis_deviation; results_table.CoG_AbsDev_X(i)=abs(cog_dev(1)); results_table.CoG_AbsDev_Y(i)=abs(cog_dev(2)); results_table.CoG_AbsDev_Z(i)=abs(cog_dev(3)); end
    if isfield(data.Sampled_features,'K_V'), results_table.StdDev_KV(i)=data.Sampled_features.K_V.std_across_motors; results_table.Range_KV(i)=data.Sampled_features.K_V.range; end
    if isfield(data.Sampled_features,'R'), results_table.StdDev_R(i)=data.Sampled_features.R.std_across_motors; results_table.Range_R(i)=data.Sampled_features.R.range; end
    if isfield(data.Sampled_features,'I_0'), results_table.StdDev_I0(i)=data.Sampled_features.I_0.std_across_motors; results_table.Range_I0(i)=data.Sampled_features.I_0.range; end
    if isfield(data.Sampled_features,'M'), results_table.Mass_AbsDev(i)=abs(data.Sampled_features.M.mean_deviation); end
    if isfield(data.Sampled_features,'I')&&isfield(data.Sampled_features.I,'per_axis_deviation'), results_table.Inertia_AbsDev_Mag(i)=norm(data.Sampled_features.I.per_axis_deviation); else, results_table.Inertia_AbsDev_Mag(i)=NaN; end
    
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files loaded)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Data aggregation complete.\n\n');

% --- 3. Perform and Display Multiple Linear Regression ---
fprintf('Step 2: Building linear models for Setpoint #%d...\n\n', setpoint_to_analyze);
mdl_error = fitlm(results_table, 'ErrorScore ~ CoG_AbsDev_X + CoG_AbsDev_Y + CoG_AbsDev_Z + StdDev_KV + Range_KV + StdDev_R + Range_R + StdDev_I0 + Range_I0 + Mass_AbsDev + Inertia_AbsDev_Mag');
disp('----------------- MODEL 1: ERROR SCORE -----------------');
disp(mdl_error);

if ~all(isnan(results_table.LeakageScore))
    mdl_leakage = fitlm(results_table, 'LeakageScore ~ CoG_AbsDev_X + CoG_AbsDev_Y + CoG_AbsDev_Z + StdDev_KV + Range_KV + StdDev_R + Range_R + StdDev_I0 + Range_I0 + Mass_AbsDev + Inertia_AbsDev_Mag');
    disp('----------------- MODEL 2: LEAKAGE SCORE -----------------');
    disp(mdl_leakage);
else
    disp('Leakage analysis skipped: The selected setpoint is not a single-axis test.');
    mdl_leakage = [];
end

% --- 4. Generate Partial Regression Plots ---
fprintf('Step 3: Generating partial regression plots...\n');
predictor_names = mdl_error.PredictorNames;
num_predictors = length(predictor_names);

figure('Name', sprintf('Error Score Sensitivity | Setpoint %d', setpoint_to_analyze), 'Position', [50, 50, 1400, 800]);
sgtitle(sprintf('Independent Effect of Each Parameter on Error Score for Setpoint #%d', setpoint_to_analyze), 'FontSize', 16, 'FontWeight', 'bold');
for i = 1:num_predictors
    subplot(ceil(num_predictors/4), 4, i);
    plotAdded(mdl_error, predictor_names{i});
    title(strrep(predictor_names{i}, '_', ' '));
    xlabel(['Effect of ', strrep(predictor_names{i}, '_', ' ')]);
    ylabel('Effect on Error Score');
    grid on;
end

if ~isempty(mdl_leakage)
    figure('Name', sprintf('Leakage Score Sensitivity | Setpoint %d', setpoint_to_analyze), 'Position', [100, 100, 1400, 800]);
    sgtitle(sprintf('Independent Effect of Each Parameter on Leakage Score for Setpoint #%d', setpoint_to_analyze), 'FontSize', 16, 'FontWeight', 'bold');
    for i = 1:num_predictors
        subplot(ceil(num_predictors/4), 4, i);
        plotAdded(mdl_leakage, predictor_names{i});
        title(strrep(predictor_names{i}, '_', ' '));
        xlabel(['Effect of ', strrep(predictor_names{i}, '_', ' ')]);
        ylabel('Effect on Leakage (%)');
        grid on;
    end
end
fprintf('Analysis complete.\n');