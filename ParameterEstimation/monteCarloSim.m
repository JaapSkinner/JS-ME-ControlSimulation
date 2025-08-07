N_samples = 1000;
N_motors = 8;
resultsFolder = 'simulationResults';
if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

% Load nominals, variationPercent, distTypes as needed
run('ParameterEstimationBase.m');
Motor_nom = Motor;
Uav_nom = Uav;
variationPercent = 5 * ones(16,1);

% Define your setpoints (example 10x6 matrix)
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

parfor i = 1:N_samples
    [Motor_i, Uav_i, features_i] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], N_motors);

    % Preallocate results storage
    simResults = struct();
    nSetpoints = size(setpoints,1);

    for sp = 1:nSetpoints
        spVal = setpoints(sp,:);
        % Run simulation for this setpoint with Motor_i, Uav_i, spVal
        % Example placeholder:
        % [output] = runYourSim(Motor_i, Uav_i, spVal);

        % Store output keyed by setpoint index or value
        simResults(sp).setpoint = spVal;
        % simResults(sp).output = output;  % uncomment when ready
        % optionally also store features_i if you want per setpoint
    end

    dataStruct = struct('Motor_i', Motor_i, 'Uav_i', Uav_i, 'features_i', features_i, 'simResults', simResults);
    filename = fullfile(resultsFolder, sprintf('simResult_%04d.mat', i));
    save(filename, '-fromstruct', 'dataStruct');
end
