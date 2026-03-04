function runSimulation_ExampleCase()
%% VALIDATION CASE: Comparative Test (Nominal vs. RLS Mixer)
% This script loads a generated Mixer file and runs TWO simulations:
% 1. NOMINAL: The perturbed drone controlled by the standard (unaware) mixer.
% 2. RLS:     The perturbed drone controlled by the optimized RLS mixer.
%
% Both simulations run against the exact same "Perturbed" Plant.

clearvars; clc;

%% 1. Initialization
fprintf('Initializing Nominal Plant...\n');
run('MixerValidationBase.m'); % Ensure this initializes 'modelName'

% --- Select Mixer File ---
startPath = fullfile(pwd, 'Results', 'Generated_Mixers');
if ~isfolder(startPath), mkdir(startPath); end

fprintf('Please select a Mixer .mat file...\n');
[fileName, pathName] = uigetfile(fullfile(startPath, '*.mat'), 'Select Mixer Result');
if isequal(fileName, 0), return; end

mixerFilePath = fullfile(pathName, fileName);
fprintf('Loading Mixer data from: %s\n', fileName);

% Load Data
loadedData = load(mixerFilePath, 'Mixers', 'rlsFullFile');
if ~isfield(loadedData, 'rlsFullFile') || ~isfield(loadedData, 'Mixers')
    error('Selected file does not contain "Mixers" or "rlsFullFile".');
end

rls_mixer_matrix = loadedData.Mixers.RLS;       % The Optimized Mixer
nom_mixer_matrix = loadedData.Mixers.Nominal;   % The Baseline Mixer
source_rls_file  = loadedData.rlsFullFile;      % Path to the specific drone

rls_mixer_sim = [rls_mixer_matrix(:,4:6), -rls_mixer_matrix(:,3), rls_mixer_matrix(:,1:2)];
nom_mixer_sim = [nom_mixer_matrix(:,4:6), -nom_mixer_matrix(:,3), nom_mixer_matrix(:,1:2)];


%% 2. Load the Specific Drone (The Plant)
fprintf('Loading RLS Drone configuration from: %s\n', source_rls_file);
if ~exist(source_rls_file, 'file')
    error('The file referenced by rlsFullFile does not exist:\n%s', source_rls_file);
end

% Load Uav and Motor from the historical RLS run
droneData = load(source_rls_file, 'Uav', 'Motor');
Uav_RLS   = droneData.Uav;
Motor_RLS = droneData.Motor;
nom_mixer_sim = Motor_RLS.mixingMatrix;

% (Optional) Verify loaded data
fprintf('  -> Loaded Drone COM Z: %.4f\n', Uav_RLS.COM(3));
fprintf('  -> Loaded Motor K_E:   %.4e\n', Motor_RLS.K_E);

%% 3. Trajectory Generation
fprintf('Generating 6-DOF Step Trajectories...\n');
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

%% 4. Prepare Simulation Inputs (Array of 2)
% We create an array of SimulationInput objects to run them cleanly.

% --- Create Base Input ---
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

% --- SIM 1: NOMINAL MIXER CASE ---
simIn(1) = baseIn;
simIn(1) = simIn(1).setVariable('Uav', Uav_RLS);     % Plant: Perturbed
simIn(1) = simIn(1).setVariable('Motor', Motor_RLS); % Plant: Perturbed
simIn(1) = simIn(1).setVariable('Mixer', nom_mixer_sim); % Controller: NOMINAL
simIn(1).UserString = 'Nominal Mixer'; % Label for debugging

% --- SIM 2: RLS MIXER CASE ---
simIn(2) = baseIn;
simIn(2) = simIn(2).setVariable('Uav', Uav_RLS);     % Plant: Perturbed
simIn(2) = simIn(2).setVariable('Motor', Motor_RLS); % Plant: Perturbed
simIn(2) = simIn(2).setVariable('Mixer', rls_mixer_sim); % Controller: RLS
simIn(2).UserString = 'RLS Mixer';

fprintf('Simulation Inputs Prepared: [1] Nominal, [2] RLS\n');

%% 5. Run Simulations
fprintf('Running Simulations (Parallel if available)...\n');
% sim() automatically handles arrays of inputs, running them in parallel 
% if Parallel Computing Toolbox is active, or sequentially otherwise.
simOutArray = sim(simIn); 

% Split results for clarity
simOut_Nominal = simOutArray(1);
simOut_RLS     = simOutArray(2);

%% 6. Save Results
outputFolder = fullfile(projectRoot, 'Results', 'Mixer_Validation');
if ~isfolder(outputFolder), mkdir(outputFolder); end

[~, name, ~] = fileparts(fileName);
outputFile = ['Validation_' name '.mat'];
savePath = fullfile(outputFolder, outputFile);

% Save both simulation outputs separately for easy plotting
save(savePath, ...
    'simOut_Nominal', ...
    'simOut_RLS', ...
    'simIn', ...
    'Uav_RLS', ...
    'Motor_RLS', ...
    'nom_mixer_matrix', ...
    'rls_mixer_matrix', ...
    'trajectory_timeseries', ...
    'source_rls_file');

fprintf('SUCCESS. Comparison results saved to:\n%s\n', savePath);
fprintf('  - simOut_Nominal: Baseline Performance\n');
fprintf('  - simOut_RLS:     Optimized Performance\n');

end