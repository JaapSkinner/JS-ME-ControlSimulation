function x = runSimulationVariedFcn()
%% Parameter Sweep - Script Version
clearvars;
clear simOut;

% % Configure these inputs manually
% paramNames = {
%     'Motor.K_V';
%     'Motor.R';
%     'Motor.volt_slope';
%     'Motor.Volt_offset';
%     'Motor.B';
% };
% varianceList = [35; 35; 35; 35; 35];  % Only vary K_V by 20%

%% Run init
run('ParameterEstimationBase.m');
run('InitUKF.m') % Innit UKF before varying parameters


Motor_nom = Motor;
Uav_nom = Uav;
Uav_nom.COM = [0 0 0];
variationPercent = [
%   Parameter        % Variation | Justification
%   -----------------|-----------|-------------------------------------------
    3.0;             % K_V       | Motor manufacturing tolerance
    3.0;             % K_E       | Linked to K_V
    3.0;             % C_TAU     | Linked to K_V
    1.0;             % B         | Motor damping (minor effect)
    2.0;             % Volt_offset | ESC/electronics tolerance
    2.0;             % volt_slope| ESC/electronics tolerance
    5.0;             % R         | Varies with motor temp & quality
    10.0;            % I_0       | No-load current, sensitive to bearings/friction
    0.5;             % D_UAV     | Rigid airframe dimension
    1.0;             % D_PROP    | Propeller manufacturing tolerance
    2.0;             % M         | Overall mass variation (component weight, battery)
    5.0;             % I         | Inertia, highly sensitive to component placement
    10.0;            % RHO_AIR   | Environmental (temp, altitude, humidity)
    1.0;             % R_PROP    | Propeller dimension
    0.5;             % A_UAV     | Rigid airframe dimension
    1.0;             % A_PROP    | Propeller manufacturing tolerance
    5.0;             % ZETA      | Aerodynamic coefficient (often uncertain)
    1.0;             % COM       | Set to 1 because it's driven by an absolute sigma
];

[Motor_i, Uav_i, features_i] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], Uav.N_ROTORS);


run('mlebusgen.m');
run('generate_trajectory.m')
addpath(submodulePath);





% %% Apply Variance to Parameters
% for i = 1:length(paramNames)
%     pname = paramNames{i};
%     pct = varianceList(i);
% 
%     parts = strsplit(pname, '.');
%     base = parts{1};
%     field = parts{2};
% 
%     val = eval(sprintf('%s.%s', base, field))
%     newVal = val .* (1 + (2*rand(size(val)) - 1) * (pct / 100))  % random variation
%     eval(sprintf('%s.%s = newVal;', base, field))
% end

%% Output tag
testCase = 'estimation';
tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation/UKFData');
outputFile = sprintf('%s_%s_%s', testCase, uavType, tStr);

%% Deactivate initial states
set_param(modelName, 'LoadInitialState', 'off');
set_param(modelName, 'LoadExternalInput', 'off');

%% Set simulation input
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setVariable('windInput', windInput);
simIn = simIn.setVariable('uavType', uavType);
simIn = simIn.setVariable('windFile', windFile);
simIn = simIn.setVariable('Simulation', Simulation);
simIn = simIn.setVariable('Uav', Uav_i);
simIn = simIn.setVariable('Motor', Motor_i);
simIn = simIn.setVariable('Initial', Initial);
simIn = simIn.setVariable('Aero', Aero);
simIn = simIn.setVariable('MLEBUS', MLEBUS);
simIn = simIn.setVariable('UKF', UKF);
simIn = simIn.setVariable('trajectory_timeseries', trajectory_timeseries);

%% Run
simOut = sim(simIn);

%% Save
if ~isfolder(outputFolder)
    mkdir(outputFolder)
end
save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor','features_i','B_matrix_nominal');
x=1;
end