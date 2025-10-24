%% Parameter Sweep - Script Version
clear all; clc;
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
run('InitUKF.m')
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
outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation/Sims');
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
simIn = simIn.setVariable('Uav', Uav);
simIn = simIn.setVariable('Motor', Motor);
simIn = simIn.setVariable('Initial', Initial);
simIn = simIn.setVariable('Aero', Aero);
simIn = simIn.setVariable('MLEBUS', MLEBUS);
simIn = simIn.setVariable('UKF', UKF);

%% Run
simOut = sim(simIn);

%% Save
if ~isfolder(outputFolder)
    mkdir(outputFolder)
end
save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor');


