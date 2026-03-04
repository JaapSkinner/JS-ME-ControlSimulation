function runSimulation_Batch()
%% VALIDATION BATCH: Comparative Test (Nominal vs. RLS Mixer)
% This script asks for a FOLDER, then loops through ALL Mixer .mat files inside.
% For each file, it runs:
% 1. NOMINAL Simulation (Perturbed Drone + Baseline Mixer)
% 2. RLS Simulation     (Perturbed Drone + Optimized Mixer)
%
% It automatically saves a corresponding "Validation_X.mat" for each input.

clearvars; clc;

%% 1. Initialization & Setup
fprintf('Initializing Nominal Plant Environment...\n');
run('MixerValidationBase.m'); % Ensure this initializes 'modelName', 'windInput', etc.

% --- Select Folder ---
startPath = fullfile(pwd, 'Results', 'Generated_Mixers');
if ~isfolder(startPath), mkdir(startPath); end

fprintf('Please select the FOLDER containing Mixer .mat files...\n');
folderPath = uigetdir(startPath, 'Select Folder with Generated Mixers');

if isequal(folderPath, 0)
    fprintf('No folder selected. Exiting.\n');
    return;
end

% Find all .mat files
fileList = dir(fullfile(folderPath, '*.mat'));
if isempty(fileList)
    error('No .mat files found in: %s', folderPath);
end

fprintf('Found %d files to process in: %s\n', length(fileList), folderPath);

%% 2. Trajectory Generation (Common for ALL runs)
% We generate the trajectory ONCE so every drone flies the exact same path.
fprintf('Generating Common 6-DOF Step Trajectories...\n');
step_amps.x = 1;      
step_amps.y = 1;      
step_amps.z = -1;     
step_amps.roll  = deg2rad(2); 
step_amps.pitch = deg2rad(2);
step_amps.yaw   = deg2rad(5);

trajectory_timeseries = generate_6dof_step_trajectory(...
    'StepDuration', 10, ...
    'RestDuration', 10, ...
    'InitialPadding', 20, ...
    'Amplitudes', step_amps, ...
    'EnablePlotting', false); 

%% 3. Prepare Common Simulation Inputs
% Create base object to reuse settings
baseIn = Simulink.SimulationInput(modelName);
baseIn = baseIn.setModelParameter('LoadInitialState', 'off');
baseIn = baseIn.setModelParameter('LoadExternalInput', 'off');
baseIn = baseIn.setVariable('windInput', windInput);
baseIn = baseIn.setVariable('uavType', uavType);
baseIn = baseIn.setVariable('windFile', windFile);
baseIn = baseIn.setVariable('Simulation', Simulation);
baseIn = baseIn.setVariable('Initial', Initial);
baseIn = baseIn.setVariable('Aero', Aero);
baseIn = baseIn.setVariable('trajectory_timeseries', trajectory_timeseries);

% Prepare Output Folder
outputFolder = fullfile(pwd, 'Results', 'Mixer_Validation_Batch');
if ~isfolder(outputFolder), mkdir(outputFolder); end

%% 4. Main Batch Loop
totalFiles = length(fileList);

for k = 1:totalFiles
    fileName = fileList(k).name;
    fullFilePath = fullfile(folderPath, fileName);
    
    fprintf('\n------------------------------------------------\n');
    fprintf('Processing File %d/%d: %s\n', k, totalFiles, fileName);
    
    try
        % --- A. Load Mixer Data ---
        loadedData = load(fullFilePath, 'Mixers', 'rlsFullFile');
        
        if ~isfield(loadedData, 'rlsFullFile') || ~isfield(loadedData, 'Mixers')
            warning('Skipping %s: Missing "Mixers" or "rlsFullFile".', fileName);
            continue;
        end
        
        rls_mixer_matrix = loadedData.Mixers.RLS;
        % nom_mixer_matrix = loadedData.Mixers.Nominal; % Not used for Sim (we use Motor.mixingMatrix)
        source_rls_file  = loadedData.rlsFullFile;

        % --- B. Reformat Mixers for Simulink ---
        % Format: [Tx, Ty, Tz, -Fz, Fx, Fy]
        rls_mixer_sim = [rls_mixer_matrix(:,4:6), -rls_mixer_matrix(:,3), rls_mixer_matrix(:,1:2)];
        
        % --- C. Load Specific Drone (Plant) ---
        if ~exist(source_rls_file, 'file')
            warning('Skipping %s: Linked drone file not found (%s).', fileName, source_rls_file);
            continue;
        end
        
        droneData = load(source_rls_file, 'Uav', 'Motor');
        Uav_RLS   = droneData.Uav;
        Motor_RLS = droneData.Motor;
        
        % Use the Motor's internal mixing matrix as the "Nominal" baseline
        % (This ensures we compare against what the drone *thought* it had)
        nom_mixer_sim = Motor_RLS.mixingMatrix;

        % --- D. Configure Simulation Inputs ---
        simIn = Simulink.SimulationInput.empty(0, 2); % Clear array
        
        % 1. Nominal Case
        simIn(1) = baseIn;
        simIn(1) = simIn(1).setVariable('Uav', Uav_RLS);
        simIn(1) = simIn(1).setVariable('Motor', Motor_RLS);
        simIn(1) = simIn(1).setVariable('Mixer', nom_mixer_sim);
        simIn(1).UserString = 'Nominal Mixer';
        
        % 2. RLS Case
        simIn(2) = baseIn;
        simIn(2) = simIn(2).setVariable('Uav', Uav_RLS);
        simIn(2) = simIn(2).setVariable('Motor', Motor_RLS);
        simIn(2) = simIn(2).setVariable('Mixer', rls_mixer_sim);
        simIn(2).UserString = 'RLS Mixer';
        
        % --- E. Run Simulation ---
        fprintf('  -> Running Simulations...\n');
        simOutArray = sim(simIn);
        
        simOut_Nominal = simOutArray(1);
        simOut_RLS     = simOutArray(2);
        
        % --- F. Save Results ---
        % Create unique output name based on input filename
        [~, nameBody, ~] = fileparts(fileName);
        outputFile = ['Validation_' nameBody '.mat'];
        savePath = fullfile(outputFolder, outputFile);
        
        save(savePath, ...
            'simOut_Nominal', ...
            'simOut_RLS', ...
            'simIn', ...
            'Uav_RLS', ...
            'Motor_RLS', ...
            'rls_mixer_matrix', ...
            'nom_mixer_sim', ...
            'rls_mixer_sim', ...
            'trajectory_timeseries', ...
            'source_rls_file');
        
        fprintf('  -> Saved: %s\n', outputFile);
        
    catch ME
        fprintf('  !!! ERROR processing %s: %s\n', fileName, ME.message);
    end
end

fprintf('\n------------------------------------------------\n');
fprintf('BATCH COMPLETE. All results saved to:\n%s\n', outputFolder);

end