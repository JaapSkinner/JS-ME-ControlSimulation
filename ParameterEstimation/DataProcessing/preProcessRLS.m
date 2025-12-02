%% PREPROCESS RLS SIMULATION DATA (V13 - Fixed isfield vs isprop)
%
% This script implements the definitive two-stage analysis workflow.
%
% V13 Changes:
% - Replaced incorrect 'isfield(simOut_eval, 'RLSData')' check with
%   'isprop(simOut_eval, 'RLSData')'.
% - 'isfield' is for STRUCTs, but 'simOut_eval' is an OBJECT.
% - Applied the same fix to the 'S_est.simOut' check.
%
% ... (rest of script description) ...
%
clear; clc; close all;

FOLDERVERSION = 'RLS_PreprocessedHEX';
%% --- 1. Run Initialization Scripts ---
fprintf('Step 1: Running initialization scripts (ParameterEstimationBase, etc.)...\n');
try
    run('ParameterEstimationBase.m');
    run('InitUKF.m');
    run('mlebusgen.m');
    run('generate_trajectory.m');
    addpath(submodulePath);
    
    if ~exist('RLS_INITIAL_GUESS', 'var')
        warning('RLS_INITIAL_GUESS variable not found after init. Sim may fail.');
        RLS_INITIAL_GUESS = []; 
    end
    
    fprintf('Initialization complete.\n');
catch ME
    fprintf(2, 'ERROR during initialization: %s\n', ME.message);
    error('Could not run base scripts. Ensure they are in the path.');
end

%% --- 2. Select Evaluation Simulink File ---
fprintf('Step 2: Select the *Evaluation Trajectory* Simulink file (.slx)...\n');
[evalSimFile, evalSimFolder] = uigetfile(fullfile(projectRoot, '*.slx'), 'Select Evaluation Simulink File');
if isequal(evalSimFile, 0)
    disp('User selected Cancel. Script terminated.');
    return;
end
evalSimPath = fullfile(evalSimFolder, evalSimFile);
[~, evalSimName, ~] = fileparts(evalSimPath); % Get sim name without .slx
fprintf('Selected evaluation file: %s\n', evalSimPath);

%% --- 3. Select Input Folder (Estimation Files) ---
fprintf('Step 3: Select the folder containing RLS *Estimation* result files (RLSData)...\n');
try
    startPath = fullfile(projectRoot, 'Results', 'ParameterEstimation', 'RLSData');
    if ~isfolder(startPath)
        startPath = fullfile(projectRoot, 'Results', 'ParameterEstimation');
    end
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end

resultsPath = uigetdir(startPath, 'Select the Folder Containing RLS Monte Carlo Results');
if isequal(resultsPath, 0)
    disp('User selected Cancel. Script terminated.');
    return;
end

resultFiles = dir(fullfile(resultsPath, 'estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0
    error('No estimation_*.mat files found in the selected directory: %s', resultsPath);
end
fprintf('Found %d estimation files. Starting preprocessing...\n', numFiles);

%% --- 4. Create Output Folder ---
[parentFolder, ~] = fileparts(resultsPath);
outputPath = fullfile(parentFolder, FOLDERVERSION); 

if ~isfolder(outputPath)
    fprintf('Creating output directory: %s\n', outputPath);
    mkdir(outputPath);
else
    fprintf('Output directory already exists: %s\n', outputPath);
end

%% --- 5. Process Each Estimation File ---
processedCount = 0;
TARGET_TIME = 50.0; % Time to sample the estimated parameters
errorLog = {};     % Initialize error log

for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(resultsPath, currentFileName);
    fprintf('Processing file (%d/%d): %s\n', i, numFiles, currentFileName);
    
    % Clear loop-specific variables
    clear S_est rlsDataContainer motorParamsTs forceEffTs torqueEffTs ...
          B_nom features_i Motor Uav N_ROTORS ...
          B_Matrix M_GainU M_coeffW M_CoeffW2 ...
          simOut_eval eval_Omega eval_Real_Wrench ...
          Omega_data_eval Omega_squared_data_eval ...
          Predicted_Wrench_data Nom_Predicted_Wrench_data ...
          Predicted_Wrench Nom_Predicted_Wrench Omega Real_Wrench;
      
    try
        % --- a. Load estimation data and ground truth ---
        S_est = load(fullFilePath);
        
        % Check if 'simOut' (the variable) exists in the loaded struct
        if ~isfield(S_est, 'simOut')
            msg = sprintf('File %s is missing ''simOut'' variable. Skipping.', currentFileName);
            warning(msg); errorLog{end+1} = msg; continue;
        end
        
        % *** V13 FIX: Check if 'RLSData' is a *property* of the 'simOut' object ***
        if ~isprop(S_est.simOut, 'RLSData')
            msg = sprintf('File %s is missing ''RLSData'' property in simOut. Skipping.', currentFileName);
            warning(msg); errorLog{end+1} = msg; continue;
        end
        
        % 'RLSData' is a struct inside the simOut object
        rlsDataContainer = S_est.simOut.RLSData; 
        
        % Check for required parameter timeseries (which *are* fields of the struct)
        required_fields = {'MotorParams', 'ForceEffectiveness', 'TorqueEffectiveness'};
        missing_field = false;
        for f_idx = 1:length(required_fields)
            field_name = required_fields{f_idx};
            if ~isfield(rlsDataContainer, field_name) || ~isa(rlsDataContainer.(field_name), 'timeseries')
                 msg = sprintf('File %s is missing RLSData.%s timeseries. Skipping.', currentFileName, field_name);
                 warning(msg); errorLog{end+1} = msg;
                 missing_field = true; break;
            end
        end
        if missing_field, continue; end
        
        % Check for ground truth and nominal objects
        if ~isfield(S_est, 'features_i') || ~isfield(S_est, 'Motor') || ~isfield(S_est, 'Uav')
             msg = sprintf('File %s is missing base objects (features_i, Motor, Uav). Skipping.', currentFileName);
             warning(msg); errorLog{end+1} = msg; continue;
        end
        if ~isfield(S_est.Uav, 'N_ROTORS')
             msg = sprintf('File %s is missing Uav.N_ROTORS. Skipping.', currentFileName);
             warning(msg); errorLog{end+1} = msg; continue;
        end
        if ~isfield(S_est, 'B_matrix_nominal')
             msg = sprintf('File %s is missing B_matrix_nominal. Skipping.', currentFileName);
             warning(msg); errorLog{end+1} = msg; continue;
        end
        
        B_nom = S_est.B_matrix_nominal;
        features_i = S_est.features_i;
        Motor = S_est.Motor;
        Uav = S_est.Uav;
        N_ROTORS = S_est.Uav.N_ROTORS;
        motorParamsTs = rlsDataContainer.MotorParams;
        forceEffTs = rlsDataContainer.ForceEffectiveness;
        torqueEffTs = rlsDataContainer.TorqueEffectiveness;
        
        % ... (Sections b, c, d, e for parameter extraction/assembly are unchanged) ...
        % --- b. Find the sample index ...
        if isempty(motorParamsTs.Time)
            msg = sprintf('File %s: MotorParams timeseries has no time data. Skipping.', currentFileName);
            warning(msg); errorLog{end+1} = msg; continue;
        end
        if motorParamsTs.Time(end) < TARGET_TIME
            msg = sprintf('Simulation in %s is shorter than %f s. Using last available sample (t=%.3f s).', ...
                    currentFileName, TARGET_TIME, motorParamsTs.Time(end));
            warning(msg); 
            [~, time_idx] = min(abs(motorParamsTs.Time - motorParamsTs.Time(end)));
            actualSampleTime = motorParamsTs.Time(end);
        else
            [~, time_idx] = min(abs(motorParamsTs.Time - TARGET_TIME));
            actualSampleTime = motorParamsTs.Time(time_idx);
        end
        fprintf('  -> Sampling parameters at t = %.3f s (target was %.3f s) using index %d.\n', actualSampleTime, TARGET_TIME, time_idx);
        
        % --- c. Extract parameter data ... ---
        try
            Estimated_MotorParams_Matrix = motorParamsTs.Data(:, :, time_idx);
            Estimated_ForceEff_Matrix = forceEffTs.Data(:, :, time_idx);
            Estimated_TorqueEff_Matrix = torqueEffTs.Data(:, :, time_idx);
        catch ME_Index
             msg = sprintf('File %s: Failed to index 3D data array at index %d. %s. Skipping.', ...
                           currentFileName, time_idx, ME_Index.message);
             warning(msg); errorLog{end+1} = msg; continue;
        end

        % --- d. Assemble Estimated B_Matrix ... ---
        if size(Estimated_ForceEff_Matrix, 1) ~= 3 || size(Estimated_ForceEff_Matrix, 2) ~= N_ROTORS
            msg = sprintf('File %s: ForceEffectiveness matrix has wrong size. Expected [3 x %d], got [%d x %d]. Skipping.', ...
                currentFileName, N_ROTORS, size(Estimated_ForceEff_Matrix, 1), size(Estimated_ForceEff_Matrix, 2));
            warning(msg); errorLog{end+1} = msg; continue;
        end
        if size(Estimated_TorqueEff_Matrix, 1) ~= 3
             msg = sprintf('File %s: TorqueEffectiveness matrix has wrong row count. Expected 3, got %d. Skipping.', ...
                currentFileName, size(Estimated_TorqueEff_Matrix, 1));
             warning(msg); errorLog{end+1} = msg; continue;
        end
        if size(Estimated_TorqueEff_Matrix, 2) == (2 * N_ROTORS)
            fprintf('  -> Torque matrix is [3 x %d], selecting W^2 terms.\n', size(Estimated_TorqueEff_Matrix, 2));
            Estimated_TorqueEff_Matrix = Estimated_TorqueEff_Matrix(:, 1:N_ROTORS);
        elseif size(Estimated_TorqueEff_Matrix, 2) ~= N_ROTORS
            msg = sprintf('File %s: TorqueEffectiveness matrix has unexpected column count. Expected %d or %d, got %d. Skipping.', ...
                currentFileName, N_ROTORS, 2*N_ROTORS, size(Estimated_TorqueEff_Matrix, 2));
            warning(msg); errorLog{end+1} = msg; continue;
        end
        B_Matrix = [Estimated_ForceEff_Matrix; Estimated_TorqueEff_Matrix];
        
        % --- e. Split and label motor parameters ... ---
        if size(Estimated_MotorParams_Matrix, 2) ~= N_ROTORS
             msg = sprintf('File %s: MotorParams matrix has wrong rotor count. Expected %d, got %d. Skipping.', ...
                currentFileName, N_ROTORS, size(Estimated_MotorParams_Matrix, 2));
             warning(msg); errorLog{end+1} = msg; continue;
        end
        if size(Estimated_MotorParams_Matrix, 1) < 3
             msg = sprintf('File %s: MotorParams matrix has too few parameters. Expected at least 3, got %d. Skipping.', ...
                currentFileName, size(Estimated_MotorParams_Matrix, 1));
             warning(msg); errorLog{end+1} = msg; continue;
        end
        M_GainU = Estimated_MotorParams_Matrix(1, :);
        M_coeffW = Estimated_MotorParams_Matrix(2, :);
        M_CoeffW2 = Estimated_MotorParams_Matrix(3, :);

        % --- f. Run Evaluation Simulation ---
        fprintf('  -> Running evaluation sim ''%s'' for this parameter set...\n', evalSimName);
        try
            simIn = Simulink.SimulationInput(evalSimName);
            
            simIn = simIn.setVariable('windInput', windInput);
            simIn = simIn.setVariable('uavType', uavType);
            simIn = simIn.setVariable('windFile', windFile);
            simIn = simIn.setVariable('Simulation', Simulation);
            simIn = simIn.setVariable('Initial', Initial);
            simIn = simIn.setVariable('Aero', Aero);
            simIn = simIn.setVariable('MLEBUS', MLEBUS);
            simIn = simIn.setVariable('UKF', UKF); 
            simIn = simIn.setVariable('InitialRLS', RLS_INITIAL_GUESS); 
            simIn = simIn.setVariable('trajectory_timeseries', trajectory_timeseries);
            simIn = simIn.setVariable('Uav', Uav);
            simIn = simIn.setVariable('Motor', Motor);
            
            simIn = simIn.setModelParameter('LoadExternalInput', 'off');
            simIn = simIn.setModelParameter('LoadInitialState', 'off');

            simOut_eval = sim(simIn);
            
            % *** V13 FIX: Check if 'RLSData' is a *property* of the 'simOut_eval' object ***
            if ~isprop(simOut_eval, 'RLSData')
                msg = sprintf('Evaluation simOut for %s is missing ''RLSData'' property. Skipping.', currentFileName);
                warning(msg); errorLog{end+1} = msg; continue;
            end
            
            % Now we can safely access .RLSData to get the struct
            eval_RLSData = simOut_eval.RLSData;
            
            % Check for the fields *inside* the RLSData struct
            if ~isfield(eval_RLSData, 'omega') || ~isfield(eval_RLSData, 'Real_Wrench')
                msg = sprintf('Evaluation simOut for %s is missing RLSData.omega/Real_Wrench. Skipping.', currentFileName);
                warning(msg); errorLog{end+1} = msg; continue;
            end
            
            eval_Omega = eval_RLSData.omega;
            eval_Real_Wrench = eval_RLSData.Real_Wrench;
            
            if isempty(eval_Omega.Time) || isempty(eval_Real_Wrench.Time)
                 msg = sprintf('Evaluation sim for %s produced empty timeseries. Skipping.', currentFileName);
                 warning(msg); errorLog{end+1} = msg; continue;
            end
            
        catch ME_sim
            msg = sprintf('ERROR running evaluation sim for %s: %s. Skipping.', currentFileName, ME_sim.message);
            warning(msg); errorLog{end+1} = msg; continue;
        end
        
        % --- g. Calculate Predicted Wrench using Evaluation Omega ---
        Omega_data_eval = squeeze(eval_Omega.Data);
        if size(Omega_data_eval, 1) > size(Omega_data_eval, 2)
            Omega_data_eval = Omega_data_eval';
        end
        Omega_squared_data_eval = Omega_data_eval .^ 2;
        
        if size(Omega_squared_data_eval, 1) ~= N_ROTORS
            msg = sprintf('File %s: N_ROTORS (%d) mismatches eval_Omega rotors (%d). Skipping.', ...
                currentFileName, N_ROTORS, size(Omega_squared_data_eval, 1));
            warning(msg); errorLog{end+1} = msg; continue;
        end

        Predicted_Wrench_data = B_Matrix * Omega_squared_data_eval;
        Nom_Predicted_Wrench_data = B_nom * Omega_squared_data_eval;
        Predicted_Wrench = timeseries(Predicted_Wrench_data', eval_Omega.Time, 'Name', 'Predicted_Wrench');
        Nom_Predicted_Wrench = timeseries(Nom_Predicted_Wrench_data', eval_Omega.Time,'Name', 'Nom_Predicted_Wrench');
        
        Omega = eval_Omega;
        Real_Wrench = eval_Real_Wrench;
        
        % --- h. Save results to new file ---
        outputFileName = ['preprocessed_' currentFileName];
        outputFullFilePath = fullfile(outputPath, outputFileName);
        
        save(outputFullFilePath, ...
             'B_Matrix', 'M_GainU', 'M_coeffW', 'M_CoeffW2', ...
             'Omega', 'Real_Wrench', ...
             'Predicted_Wrench', 'Nom_Predicted_Wrench', ...
             'features_i', 'Motor', 'Uav','B_nom');
        
        processedCount = processedCount + 1;
        
    catch ME
        % Capture general catch errors
        msg1 = sprintf('UNHANDLED ERROR processing file %s: %s', currentFileName, ME.message);
        msg2 = sprintf('At line %d in function %s. Skipping this file.', ME.stack(1).line, ME.stack(1).name);
        fprintf(2, '%s\n%s\n', msg1, msg2);
        errorLog{end+1} = [msg1, ' | ', msg2];
    end
end

%% --- 6. Final Report ---
fprintf('\n-------------------------------------------------\n');
fprintf('Preprocessing Complete.\n');
fprintf('Successfully processed and saved %d out of %d files.\n', processedCount, numFiles);

% Print error summary
if processedCount < numFiles
    fprintf(2, '\n--- %d FILES FAILED TO PROCESS ---\n', numFiles - processedCount);
    uniqueErrors = unique(errorLog);
    for k = 1:length(uniqueErrors)
        fprintf(2, 'REASON: %s\n', uniqueErrors{k});
    end
    fprintf(2, '--------------------------------------\n');
    fprintf(2, 'Please check the file(s) and error messages above.\n');
end

fprintf('Preprocessed files are located in: %s\n', outputPath);