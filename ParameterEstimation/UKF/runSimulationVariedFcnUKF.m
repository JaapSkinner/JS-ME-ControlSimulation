function runSimulationVariedFcnUKF(sampleIndex)
%% RUN SIMULATION FROM FILE
% Runs a single simulation using a pre-generated parameter set from the
% 'Data/ParameterSets_Batch1' folder.
%
% Usage:
%   runSimulationFromFile(1);   % Runs ParamSet_001.mat
%   runSimulationFromFile(15);  % Runs ParamSet_015.mat

    %% 1. Environment Setup
    % Note: We cannot use 'clearvars' here because it would wipe 'sampleIndex'
    run('UKFBase.m');

    % Ensure projectRoot is defined (fallback to pwd if missing)
    if ~exist('projectRoot', 'var')
        projectRoot = pwd; 
    end
    
    folderName = 'ParameterSetCOMZ'; % Folder name from GenerateParameterSets.m
    dataSetDir = fullfile(projectRoot, 'ParameterEstimation', folderName);
    
    % Construct filename
    paramFileName = sprintf('ParamSet_%03d.mat', sampleIndex);
    paramFilePath = fullfile(dataSetDir, paramFileName);
    
    if ~isfile(paramFilePath)
        error('Parameter file not found: %s\nRun GenerateParameterSets.m first.', paramFilePath);
    end
    
    % fprintf('--------------------------------------------------\n');
    % fprintf('Running Simulation for Sample Index: %d\n', sampleIndex);
    % fprintf('Loading Parameters: %s\n', paramFileName);
    % 
    %% 2. Initialize Base Simulation Environment
    % Run the standard initialization to get the environment ready.
    % This sets up the default paths, buses, and constants.
    run('InitUKF.m');
    
    % Generate Trajectory
    run('generate_trajectory.m');
    
    %% 3. Load & Apply Pre-Generated Parameters
    loadedData = load(paramFilePath);
    
    % Extract variables
    Uav_perturbed = loadedData.Uav;
    Motor_perturbed = loadedData.Motor;
    features_i = loadedData.features_i;
    B_matrix_nominal_CLEAN = loadedData.B_matrix_nominal; % This is the "Gold Standard"
    
    % --- CRITICAL: FORCE UPDATE BASE WORKSPACE ---
    % Simulink InitFcn often looks at the Base Workspace. We must overwrite
    % the defaults created by Step 2 with our perturbed loaded data.
    assignin('base', 'Uav', Uav_perturbed);
    assignin('base', 'Motor', Motor_perturbed);
    
    % Run Bus Generation (In case it relies on Uav dimensions)
    % Note: This might recalculate B_matrix based on the perturbed Uav,
    % so we must restore the clean one immediately after.
    run('mlebusgen.m'); 
    
    % Restore the Clean Nominal Matrix (from file) to the Workspace
    assignin('base', 'B_matrix_nominal', B_matrix_nominal_CLEAN);
    
    % Local variables for the simulation object
    Uav = Uav_perturbed;
    Motor = Motor_perturbed;
    B_matrix_nominal = B_matrix_nominal_CLEAN;
    
    %% 4. Prepare Simulation Input    
    % Output Settings
    testCase = 'estimation';
    tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    
    % Include SampleIndex in the filename for easy tracking
    outputFile = sprintf('%s_Sample%03d_%s_%s', testCase, sampleIndex, uavType, tStr);
    outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation', 'UKFDataFixedParamsCOMZ3');
    
    % Simulation Input Object
    simIn = Simulink.SimulationInput(modelName);
    
    % Simulation Configuration
    simIn = simIn.setModelParameter('LoadInitialState', 'off');
    simIn = simIn.setModelParameter('LoadExternalInput', 'off');
    
    % Inject Variables (Redundant with assignin, but good practice)
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
    simIn = simIn.setVariable('trajectory_timeseries', trajectory_timeseries);
    simIn = simIn.setVariable('B_matrix_nominal', B_matrix_nominal);

    %% 5. Run Simulation
    % fprintf('Starting Simulink...\n');
    
    % Force Simulink to refresh parameters from Base Workspace
    % This fixes the "Shadowed Variable" bug where it ignores setVariable
    % set_param(modelName, 'SimulationCommand', 'update');
    
    simOut = sim(simIn);
    
    %% 6. Save Results
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    
    savePath = fullfile(outputFolder, outputFile);
    
    save(savePath, 'simIn', 'simOut', 'Uav', 'Motor', 'features_i', 'B_matrix_nominal');
    
    % fprintf('Success! Results saved to:\n%s\n', savePath);

end