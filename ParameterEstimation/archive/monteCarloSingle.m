%% Single test run
N_motors = 8;
resultsFolder = 'Results/ParameterEstimation/MonteCarlo';
if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

% Load nominals
run('ParameterEstimationBaseOL.m');
Motor_nom = Motor;
Uav_nom = Uav;
Uav_nom.COM = [0 0 0];
variationPercent = 0 * ones(17,1);

% Choose one setpoint to test
setpoint = timeseries([0 0 0.5 0 0 0.6], 0);

% Sample parameters once
[~, ~, features_i] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], N_motors);


Motor_i = Motor;
Uav_i = Uav;
% Run simulation once for that setpoint
% Replace with your real simulation call
% output = runYourSim(Motor_i, Uav_i, setpoint);

simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setVariable('Simulation', Simulation);
simIn = simIn.setVariable('Uav', Uav);
simIn = simIn.setVariable('Motor', Motor);
simIn = simIn.setVariable('Initial', Initial);
simIn = simIn.setVariable('Aero', Aero);

%% Run
simOut = sim(simIn);
simResults = struct();

simResults(1).TBody = simOut.TBody;
simResults(1).tauBody = simOut.tauBody;
% Store results
simResults(1).setpoint = setpoint;
% simResults(1).output = output;  % uncomment when integrated

dataStruct = struct('Motor_i', Motor_i, ...
                    'Uav_i', Uav_i, ...
                    'features_i', features_i, ...
                    'simResults', simResults);

filename = fullfile(resultsFolder, 'simResult_single.mat');
save(filename, '-struct', 'dataStruct');
