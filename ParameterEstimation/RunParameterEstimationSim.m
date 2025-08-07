% RunParameterEstimationSim.m
function simOut = RunParameterEstimationSim(~, variationPercent)
    rng(6831539);
    clear simIn simOut logsout
    % Run the base script to load and initialize everything
    run('ParameterEstimationBase.m');
    run('mlebusgen.m');
    addpath(submodulePath);
    
    %% Parameter Variation
    vary = @(x, p) x .* (1 + (2*rand(size(x)) - 1) * (p / 100));
    
    % MOTOR
    Motor.K_V         = vary(Motor.K_V,         variationPercent(1));
    Motor.K_E         = vary(Motor.K_E,         variationPercent(2));
    Motor.B           = vary(Motor.B,           variationPercent(3));
    Motor.Volt_offset = vary(Motor.Volt_offset, variationPercent(4));
    Motor.volt_slope  = vary(Motor.volt_slope,  variationPercent(5));
    Motor.R           = vary(Motor.R,           variationPercent(6));
    Motor.I_0         = vary(Motor.I_0,         variationPercent(7));
    
    % UAV
    Uav.D_UAV         = vary(Uav.D_UAV,         variationPercent(8));
    Uav.D_PROP        = vary(Uav.D_PROP,        variationPercent(9));
    Uav.M             = vary(Uav.M,             variationPercent(10));
    Uav.I             = vary(Uav.I,             variationPercent(11));
    Uav.RHO_AIR       = vary(Uav.RHO_AIR,       variationPercent(12));
    Uav.R_PROP        = vary(Uav.R_PROP,        variationPercent(13));
    Uav.A_UAV         = vary(Uav.A_UAV,         variationPercent(14));
    Uav.A_PROP        = vary(Uav.A_PROP,        variationPercent(15));
    Uav.ZETA          = vary(Uav.ZETA,          variationPercent(16));



        
    
    
    %% Create test tag and file name
    testCase = 'estimation';
    tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation');
    outputFile = sprintf('%s_%s_%s', testCase, uavType, tStr);
    
    
    
    
    %% Deactivate initial states
    set_param(modelName, 'LoadInitialState', 'off' );
    set_param( modelName, 'LoadExternalInput', 'off' );
    
    % Mean wind for log
    UMean = mean(windInput.Data(:,1));
    
    
    
    
    
    %% 4) CUSTOM CODE
    % pUT ANY SPECIAL CONDITIONS FOR YOUR SIMULATION, SUCH AS POSITION OR
    % ATTITUDE SETPOINTS, HERE...
    
    
    
    
    
    
    
    
    %% RUN SIMULATION
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
    simIn = simIn.setModelParameter('SaveOutput', 'off', 'SaveTime', 'off', 'SaveState', 'off', 'SaveFormat', 'StructureWithTime');
    warning('off','SDI:sdi:LowDiskSpaceRecordOff');
    warning('off');
    % 
    % disp('Simulation runs:')
    % fprintf('\t[%d/%d] wind file: % 18s, Umean = %5.2f m/s\n', 1, 1, windFile, UMean);
    
    simOut = sim(simIn);
    warning("on")
    [~, warnId] = lastwarn;
    fprintf('Last warning ID: %s\n', warnId);
    
    if ~isfolder(outputFolder)
        mkdir(outputFolder)
    end
    save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor', 'windInput', 'windFile');
    simOut = sim(simIn);
    
    if ~isfolder(outputFolder)
        mkdir(outputFolder)
    end
    save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor', 'windInput', 'windFile');
    close_system(modelName, 0);
end