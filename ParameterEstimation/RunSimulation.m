% RunParameterEstimationSim.m
clear; clc;

% Run the base script to load and initialize everything
run('ParameterEstimationBase.m');

% Create test tag and file name
testCase = 'estimation';
tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
outputFolder = fullfile(projectRoot, 'data_results', 'param_estimation');
outputFile = sprintf('%s_%s_%s', testCase, uavType, tStr);

% Make sure trim options are disabled for PX4 model
set_param(modelName, 'LoadInitialState', 'off');
set_param(modelName, 'LoadExternalInput', 'off');

% Mean wind for log
UMean = mean(windInput.Data(:,1));

% Set up simulation input
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setVariable('windInput', windInput);
simIn = simIn.setVariable('uavType', uavType);
simIn = simIn.setVariable('windFile', windFile);
simIn = simIn.setVariable('Simulation', Simulation);
simIn = simIn.setVariable('Uav', Uav);
simIn = simIn.setVariable('Motor', Motor);
simIn = simIn.setVariable('Initial', Initial);

% Display run info
disp('Simulation runs:')
fprintf('\t[%d/%d] wind file: % 18s, Umean = %5.2f m/s\n', 1, 1, windFile, UMean);

% Run sim
simOut = sim(simIn);

% Save results
if ~isfolder(outputFolder)
    mkdir(outputFolder)
end
save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor', 'windInput', 'windFile');
