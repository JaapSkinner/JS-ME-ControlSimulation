function runMonteCarloChunks()
    disp("starting Monte Carlo Chunked Simulation")
    N_samples   = 1000;
    chunk_size  = 20;
    N_motors    = 8;
    resultsFolder = 'simulationResults';
    if ~exist(resultsFolder, 'dir')
        mkdir(resultsFolder);
    end

    % Load base parameters
    run('ParameterEstimationBaseOL.m');
    Motor_nom = Motor;
    Uav_nom   = Uav;
    Uav_nom.COM = [0 0 0];
    variationPercent = 15 * ones(17,1);

    % Define setpoints
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

    % Loop over batches
    for startIdx = 1:chunk_size:N_samples
        endIdx = min(startIdx+chunk_size-1, N_samples);

        parfor i = startIdx:endIdx
            [Motor_i, Uav_i, features_i] = sampleParameters(Motor_nom, Uav_nom, variationPercent, [], N_motors);

            simResults = struct();
            nSetpoints = size(setpoints,1);

            for sp = 1:nSetpoints
                spVal = setpoints(sp,:);

                simIn = Simulink.SimulationInput(modelName);
                simIn = simIn.setVariable('Simulation', Simulation);
                simIn = simIn.setVariable('Uav', Uav_i);
                simIn = simIn.setVariable('Motor', Motor_i);
                simIn = simIn.setVariable('Initial', Initial);
                simIn = simIn.setVariable('Aero', Aero);
                simIn = simIn.setVariable('setpoint', spVal);

                simOut = sim(simIn);

                simResults(sp).setpoint = spVal;
                simResults(sp).TBody    = simOut.TBody;
                simResults(sp).tauBody  = simOut.tauBody;
            end

            dataStruct = struct('Motor_i', Motor_i, 'Uav_i', Uav_i, ...
                                'features_i', features_i, 'simResults', simResults);
            filename = fullfile(resultsFolder, sprintf('simResult_%04d.mat', i));
            save(filename, '-fromstruct', dataStruct);
        end

        clearvars -except N_samples chunk_size N_motors resultsFolder ...
                           Motor_nom Uav_nom variationPercent setpoints ...
                           startIdx endIdx modelName Simulation Initial Aero
        pack
    end
end
