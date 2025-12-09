function run_RLS_replay(sampleIndex)
%% RLS Replay Execution Function
% Runs the RLS Estimator against Real Flight Data.
% Usage: run_RLS_replay(1)

    %% 1. Project & Path Setup
    % Ensure we are in the right environment
    try
        proj = currentProject;
        projectRoot = proj.RootFolder;
    catch
        projectRoot = pwd;
    end
    
    % Define paths (Adjust to match your folder structure)
    modelName = 'RLS_Replay'; % Name of your NEW Replay Simulink File
    modelDataFile = fullfile(projectRoot, 'ParameterEstimation', 'Hex.mat');
    
    % Add paths
    addpath(genpath(fullfile(projectRoot, 'Simulink_Flight_Simulation')));
    addpath(genpath(fullfile(projectRoot, 'ParameterEstimation')));

    %% 2. Initialize Vehicle Parameters (Nominal)
    % We need Uav, Motor, Aero structs to calculate RLS_INITIAL_GUESS
    if isfile(modelDataFile)
        load(modelDataFile); % Loads Uav, Motor, Aero, Initial
    else
        error('Hex.mat not found. Cannot initialize vehicle parameters.');
    end
    
    % Run Standard Initialization (fills in derived values like Inertia)
    [Uav, Motor, Aero, Initial] = InitializeParametersUAV(Uav, Motor, Aero);
    
    %% 3. (Optional) Load Perturbed Parameters
    % If you want to test RLS robustness to bad initial guesses, load them here.
    % If sampleIndex > 0, we try to load a parameter set.
    if nargin > 0 && sampleIndex > 0
        paramFile = fullfile(projectRoot, 'ParameterEstimation', 'ParameterSetHEX', ...
            sprintf('ParamSet_%03d.mat', sampleIndex));
        
        if isfile(paramFile)
            fprintf('Loading Perturbed Parameters: %s\n', paramFile);
            ptb = load(paramFile);
            Uav = ptb.Uav;
            Motor = ptb.Motor;
            % Re-Init derived params with perturbed values
            [Uav, Motor, Aero, Initial] = InitializeParametersUAV(Uav, Motor, Aero);
        else
            warning('Parameter set %d not found. Using Nominal.', sampleIndex);
        end
    end

    %% 4. Initialize RLS (Calculate Guesses)
    % This script uses Uav/Motor structs to build RLS_INITIAL_GUESS
    fprintf('Initializing RLS Estimator...\n');
    run('InitRLSEXP.m'); 
    
    % Ensure B_matrix_nominal exists (Simulink often needs it)
    if ~exist('B_matrix_nominal', 'var')
        warning('B_matrix_nominal was not created by InitRLSEXP.m. Checking logic...');
    end

    %% 5. Load Replay Data
    fprintf('Loading Flight Data...\n');
    run('setup_replay_data.m'); % Uses the script above
    
    %% 6. Simulation Configuration
    outputFolder = fullfile(projectRoot, 'Results', 'ReplayRLS');
    if ~isfolder(outputFolder); mkdir(outputFolder); end
    
    tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    outputFile = sprintf('RLS_Replay_Log_%s', tStr);
    
    % Create Simulation Input
    simIn = Simulink.SimulationInput(modelName);
    
    % --- Inject Variables into Simulation ---
    % 1. Vehicle Params (for Mixer/Init)
    simIn = simIn.setVariable('Uav', Uav);
    simIn = simIn.setVariable('Motor', Motor);
    
    % 2. RLS Specifics
    simIn = simIn.setVariable('InitialRLS', RLS_INITIAL_GUESS);
    simIn = simIn.setVariable('B_matrix_nominal', B_matrix_nominal);
    
    % 3. Filter Coefficients (Calculated in setup_replay_data)
    simIn = simIn.setVariable('diff_num', diff_num);
    simIn = simIn.setVariable('diff_den', diff_den);
    
    % 4. Replay Signals (Timeseries)
    simIn = simIn.setVariable('ts_u_cmd', ts_u_cmd);
    simIn = simIn.setVariable('ts_omega', ts_omega);
    simIn = simIn.setVariable('ts_accel', ts_accel);
    simIn = simIn.setVariable('ts_pos_truth', ts_pos_truth);
    
    % 5. Config
    simIn = simIn.setModelParameter('StopTime', string(sim_duration));
    simIn = simIn.setModelParameter('SolverType', 'Fixed-step');
    simIn = simIn.setModelParameter('FixedStep', string(replay_dt));

    %% 7. Run & Save
    fprintf('Running Replay Simulation...\n');
    simOut = sim(simIn);
    
    save(fullfile(outputFolder, outputFile), 'simOut', 'Uav', 'Motor', 'RLS_INITIAL_GUESS');
    fprintf('Replay Complete. Results saved to: %s\n', outputFile);

end