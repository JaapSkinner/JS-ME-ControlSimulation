N_samples = 1000;
N_motors = 8;
resultsFolder = 'Results/ParameterEstimation/MonteCarlo/temp';
if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

run('ParameterEstimationBaseOL.m');
Motor_nom = Motor;
Uav_nom = Uav;
Uav_nom.COM = [0 0 0];
variationPercent = 15 * ones(17,1);

setpoints = [
    0      0      0    1.2    1.3    1.0;
    0.05   0.05   0    1.4    0.0    1.2;
    0      0      0    0.0    1.5    1.1;
    0.02   0.01   0    1.0    1.0    1.3;
    0      0      0    1.6    0.0    1.3;
    0      0      0    0.0    0.0    1.0;
    0      0      0    0.0    0.0    1.0;
    0      0      0    0.0    1.2    1.2;
    0      0      0    0.0    1.1    1.1;
    0      0      0    0.0    0.0    1.3;
];

for i = 1:N_samples
    [Motor_i, Uav_i, features_i] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], N_motors);

    % Prepare SimulationInput objects for all setpoints
    nSetpoints = size(setpoints,1);
    simInputs(nSetpoints) = Simulink.SimulationInput(modelName); % preallocate array

    for sp = 1:nSetpoints
        spVal = setpoints(sp,:);
        simInputs(sp) = Simulink.SimulationInput(modelName); % <-- set modelName here
        simInputs(sp) = simInputs(sp).setVariable('Simulation', Simulation);
        simInputs(sp) = simInputs(sp).setVariable('Uav', Uav_i);
        simInputs(sp) = simInputs(sp).setVariable('Motor', Motor_i);
        simInputs(sp) = simInputs(sp).setVariable('Initial', Initial);
        simInputs(sp) = simInputs(sp).setVariable('Aero', Aero); % if needed
        simInputs(sp) = simInputs(sp).setVariable('setpoint', spVal);
    end


    % Run all simulations in parallel for the setpoints
    simOuts = parsim(simInputs, 'ShowProgress', 'on');

    % Collect results
    simResults = struct();
    for sp = 1:nSetpoints
        simResults(sp).setpoint = setpoints(sp,:);
        simResults(sp).TBody = simOuts(sp).TBody;
        simResults(sp).tauBody = simOuts(sp).tauBody;
    end

    dataStruct = struct('Motor_i', Motor_i, 'Uav_i', Uav_i, 'features_i', features_i, 'simResults', simResults);
    filename = fullfile(resultsFolder, sprintf('simResult_%04d.mat', i));
    save(filename, '-fromstruct', dataStruct);
end
