% =========================================================================
% ADVANCED SENSITIVITY ANALYSIS (v1)
% =========================================================================
% This script uses Multiple Linear Regression to analyze the independent
% effect of each parameter variation on system performance. It overcomes the
% "blob" problem in simple scatter plots by statistically controlling for
% confounding variables.
%
% OUTPUT:
% 1. Console Output: A table of regression coefficients, showing the
%    strength and significance of each parameter's effect.
% 2. Plots: Partial regression plots that visualize the "cleaned up"
%    relationship between each parameter and the performance scores.
%
clear; clc; close all;

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
fprintf('Found %d result files. Aggregating data for regression analysis...\n', numFiles);

% (The data aggregation loop is the same robust version as before)
firstData = load(fullfile(resultFiles(1).folder, resultFiles(1).name));
setpoints = firstData.setpoints; nSetpoints = size(setpoints, 1);
is_single_axis = (sum(setpoints~=0,2)==1) | (sum(setpoints~=0,2)==2 & setpoints(:,3)~=0);
single_axis_indices = find(is_single_axis);
varNames = {'Score_Error','Score_Leakage','CoG_AbsDev_X','CoG_AbsDev_Y','CoG_AbsDev_Z','StdDev_KV','Range_KV','StdDev_R','Range_R','StdDev_I0','Range_I0','Mass_AbsDev','Inertia_AbsDev_Mag'};
varTypes = repmat({'double'}, 1, length(varNames));
results_table = table('Size', [numFiles, length(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);
for i = 1:numFiles
    data = load(fullfile(resultFiles(i).folder, resultFiles(i).name));
    current_run_errors=zeros(nSetpoints,1); current_run_leakages=[];
    for sp = 1:nSetpoints
        w_des=data.setpoints(sp,:)'; w_actual=data.wrenches(:,sp);
        error_raw=w_actual-w_des; current_run_errors(sp)=norm(error_raw./max_authority);
        if ismember(sp,single_axis_indices)
            off_axis_indices=find(w_des==0); norm_leakage=norm(w_actual(off_axis_indices)); norm_des=norm(w_des);
            if norm_des > 1e-6, current_run_leakages(end+1)=(norm_leakage/norm_des)*100; end
        end
    end
    results_table.Score_Error(i)=mean(current_run_errors); results_table.Score_Leakage(i)=mean(current_run_leakages);
    if isfield(data.Sampled_features,'COM')&&isfield(data.Sampled_features.COM,'per_axis_deviation'), cog_dev=data.Sampled_features.COM.per_axis_deviation; results_table.CoG_AbsDev_X(i)=abs(cog_dev(1)); results_table.CoG_AbsDev_Y(i)=abs(cog_dev(2)); results_table.CoG_AbsDev_Z(i)=abs(cog_dev(3)); end
    if isfield(data.Sampled_features,'K_V'), results_table.StdDev_KV(i)=data.Sampled_features.K_V.std_across_motors; results_table.Range_KV(i)=data.Sampled_features.K_V.range; end
    if isfield(data.Sampled_features,'R'), results_table.StdDev_R(i)=data.Sampled_features.R.std_across_motors; results_table.Range_R(i)=data.Sampled_features.R.range; end
    if isfield(data.Sampled_features,'I_0'), results_table.StdDev_I0(i)=data.Sampled_features.I_0.std_across_motors; results_table.Range_I0(i)=data.Sampled_features.I_0.range; end
    if isfield(data.Sampled_features,'M'), results_table.Mass_AbsDev(i)=abs(data.Sampled_features.M.mean_deviation); end
    if isfield(data.Sampled_features,'I')&&isfield(data.Sampled_features.I,'per_axis_deviation'), results_table.Inertia_AbsDev_Mag(i)=norm(data.Sampled_features.I.per_axis_deviation); else, results_table.Inertia_AbsDev_Mag(i)=NaN; end
    if mod(i, 500) == 0 || i == numFiles, fprintf('... Progress: %.0f%% (%d / %d files loaded)\n', (i/numFiles)*100, i, numFiles); end
end
fprintf('Data aggregation complete.\n\n');

% --- 2. Perform Multiple Linear Regression ---
fprintf('Step 2: Building linear models to determine parameter sensitivity...\n\n');

% Fit a model for the Error Score
mdl_error = fitlm(results_table, 'Score_Error ~ CoG_AbsDev_X + CoG_AbsDev_Y + CoG_AbsDev_Z + StdDev_KV + Range_KV + StdDev_R + Range_R + StdDev_I0 + Range_I0 + Mass_AbsDev + Inertia_AbsDev_Mag');

% Fit a model for the Leakage Score
mdl_leakage = fitlm(results_table, 'Score_Leakage ~ CoG_AbsDev_X + CoG_AbsDev_Y + CoG_AbsDev_Z + StdDev_KV + Range_KV + StdDev_R + Range_R + StdDev_I0 + Range_I0 + Mass_AbsDev + Inertia_AbsDev_Mag');

% Display the model summaries in the console
disp('----------------- MODEL 1: AVERAGE ERROR SCORE -----------------');
disp(mdl_error);
disp('----------------- MODEL 2: AVERAGE LEAKAGE SCORE -----------------');
disp(mdl_leakage);


% --- 3. Generate Partial Regression Plots ---
fprintf('Step 3: Generating partial regression plots to visualize independent effects...\n');
predictor_names = mdl_error.PredictorNames; % Get names of all parameters in the model
num_predictors = length(predictor_names);

% Figure 1: Partial Regression Plots for Error Score
figure('Name', 'Partial Regression: Error Score', 'NumberTitle', 'off', 'Position', [50, 50, 1400, 800]);
sgtitle('Independent Effect of Each Parameter on Average Error Score', 'FontSize', 16, 'FontWeight', 'bold');
for i = 1:num_predictors
    subplot(ceil(num_predictors/4), 4, i);
    plotAdded(mdl_error, predictor_names{i});
    % Customize plot appearance
    title(strrep(predictor_names{i}, '_', ' '));
    xlabel(['Effect of ', strrep(predictor_names{i}, '_', ' ')]);
    ylabel('Effect on Avg Error Score');
    grid on;
end

%% Figure 2: Partial Regression Plots for Leakage Score
figure('Name', 'Partial Regression: Leakage Score', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 800]);
sgtitle('Independent Effect of Each Parameter on Average Leakage Score', 'FontSize', 16, 'FontWeight', 'bold');
for i = 1:num_predictors
    subplot(ceil(num_predictors/4), 4, i);
    plotAdded(mdl_leakage, predictor_names{i});
    % Customize plot appearance
    title(strrep(predictor_names{i}, '_', ' '));
    xlabel(['Effect of ', strrep(predictor_names{i}, '_', ' ')]);
    ylabel('Effect on Avg Leakage (%)');
    grid on;
end

fprintf('Analysis complete.\n');