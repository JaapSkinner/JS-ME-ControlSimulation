    % RunParameterEstimationSim.m
    clear; clc;
    
    % Run the base script to load and initialize everything
    run('ParameterEstimationBase.m');
    run('mlebusgen.m');
    addpath(submodulePath);
    
    
    
    
    %% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %% Create test tag and file name
    testCase = 'estimation';
    tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation');
    outputFile = sprintf('%s_%s_%s', testCase, uavType, tStr);
    
    
  
    
    %% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %% Deactivate initial states
    set_param(modelName, 'LoadInitialState', 'off' );
    set_param( modelName, 'LoadExternalInput', 'off' );
    
    % Mean wind for log
    UMean = mean(windInput.Data(:,1));
    
    
    
    
    
    %% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %% 4) CUSTOM CODE
    % pUT ANY SPECIAL CONDITIONS FOR YOUR SIMULATION, SUCH AS POSITION OR
    % ATTITUDE SETPOINTS, HERE...
    
    
    
    
    
    
    
    
    %% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %% RUN SIMULATION
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setVariable('windInput', windInput);
    simIn = simIn.setVariable('uavType', uavType);
    simIn = simIn.setVariable('windFile', windFile);
    simIn = simIn.setVariable('Simulation', Simulation);
    simIn = simIn.setVariable('Uav', Uav);
    simIn = simIn.setVariable('Motor', Motor);
    simIn = simIn.setVariable('Initial', Initial);
    
    disp('Simulation runs:')
    fprintf('\t[%d/%d] wind file: % 18s, Umean = %5.2f m/s\n', 1, 1, windFile, UMean);
    
    simOut = sim(simIn);
    
    if ~isfolder(outputFolder)
        mkdir(outputFolder)
    end
    save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor', 'windInput', 'windFile');
