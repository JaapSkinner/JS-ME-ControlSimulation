%% PREPROCESS RLS SIMULATION DATA
%
% ... (script description) ...
%
% V4: Handles [3 x 2*N_ROTORS] torque matrix by selecting only the
%     first N_ROTORS columns (W^2 term) and dropping W_dot term.
%     Also fixed a typo in the final save() command.
%
clear; clc; close all;

%% --- 1. Select Input Folder ---
fprintf('Step 1: Select the folder containing RLS result files...\n');
try
    % Try to find the project root and navigate to the default data folder
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation', 'RLSData');
    if ~isfolder(startPath)
        % Fallback if the specific folder doesn't exist
        startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    end
    if ~isfolder(startPath)
        % Fallback to current directory
        startPath = pwd;
    end
catch
    % Safest fallback
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

fprintf('Found %d result files. Starting preprocessing...\n', numFiles);

%% --- 2. Create Output Folder ---
[parentFolder, ~] = fileparts(resultsPath);
outputPath = fullfile(parentFolder, 'RLS_Preprocessed');

if ~isfolder(outputPath)
    fprintf('Creating output directory: %s\n', outputPath);
    mkdir(outputPath);
else
    fprintf('Output directory already exists: %s\n', outputPath);
end

%% --- 3. Process Each File ---
processedCount = 0;
TARGET_TIME = 3.0; % Time to sample the estimated parameters

for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(resultsPath, currentFileName);
    fprintf('Processing file (%d/%d): %s\n', i, numFiles, currentFileName);
    
    try
        % --- a. Load and check data ---
        S = load(fullFilePath);
        
        if ~isfield(S, 'simOut') || ~isprop(S.simOut, 'RLSData')
            warning('File %s is missing simOut.RLSData. Skipping.', currentFileName);
            continue;
        end
        
        rlsDataContainer = S.simOut.RLSData;
        
        required_fields = {'omega', 'Real_Wrench', 'MotorParams', 'ForceEffectiveness', 'TorqueEffectiveness'};
        missing_field = false;
        for f_idx = 1:length(required_fields)
            field_name = required_fields{f_idx};
            if ~isfield(rlsDataContainer, field_name) || ~isa(rlsDataContainer.(field_name), 'timeseries')
                 warning('File %s is missing RLSData.%s timeseries. Skipping.', currentFileName, field_name);
                 missing_field = true;
                 break;
            end
        end
        if missing_field, continue; end
        
        if ~isfield(S, 'features_i') || ~isfield(S, 'Motor') || ~isfield(S, 'Uav')
             warning('File %s is missing base objects (features_i, Motor, Uav). Skipping.', currentFileName);
             continue;
        end
        if ~isfield(S.Uav, 'N_ROTORS')
             warning('File %s is missing Uav.N_ROTORS. Skipping.', currentFileName);
             continue;
        end
        if ~isfield(S, 'B_matrix_nominal')
             warning('File %s is missing B_matrix_nominal. Skipping.', currentFileName);
             continue;
        end
        
        % Assign timeseries
        omegaTs = rlsDataContainer.omega;
        Real_Wrench = rlsDataContainer.Real_Wrench;
        motorParamsTs = rlsDataContainer.MotorParams;
        forceEffTs = rlsDataContainer.ForceEffectiveness;
        torqueEffTs = rlsDataContainer.TorqueEffectiveness;
        
        % Assign other objects
        B_nom = S.B_matrix_nominal;
        features_i = S.features_i;
        Motor = S.Motor;
        Uav = S.Uav;
        N_ROTORS = S.Uav.N_ROTORS;
        
        % --- b. Find the sample index closest to TARGET_TIME ---
        if isempty(motorParamsTs.Time)
            warning('File %s: MotorParams timeseries has no time data. Skipping.', currentFileName);
            continue;
        end

        if motorParamsTs.Time(end) < TARGET_TIME
            warning('Simulation in %s is shorter than %f s. Using last available sample (t=%.3f s).', ...
                    currentFileName, TARGET_TIME, motorParamsTs.Time(end));
            [~, time_idx] = min(abs(motorParamsTs.Time - motorParamsTs.Time(end)));
            actualSampleTime = motorParamsTs.Time(end);
        else
            [~, time_idx] = min(abs(motorParamsTs.Time - TARGET_TIME));
            actualSampleTime = motorParamsTs.Time(time_idx);
        end
        
        fprintf('  -> Sampling parameters at t = %.3f s (target was %.3f s) using index %d.\n', actualSampleTime, TARGET_TIME, time_idx);
        
        % --- c. Extract parameter data at the specific time index ---
        % [Params x Rotors x Time]
        try
            Estimated_MotorParams_Matrix = motorParamsTs.Data(:, :, time_idx);
            Estimated_ForceEff_Matrix = forceEffTs.Data(:, :, time_idx);
            Estimated_TorqueEff_Matrix = torqueEffTs.Data(:, :, time_idx);
        catch ME_Index
             fprintf(2, 'ERROR: Failed to index 3D data array at index %d.\n', time_idx);
             warning('File %s: Timeseries data is likely empty or has mismatched dimensions. %s. Skipping.', currentFileName, ME_Index.message);
             continue;
        end

        % --- d. Assemble B_Matrix ---
        
        % Check Force dimensions
        if size(Estimated_ForceEff_Matrix, 1) ~= 3 || size(Estimated_ForceEff_Matrix, 2) ~= N_ROTORS
            warning('File %s: ForceEffectiveness matrix has wrong size. Expected [3 x %d], got [%d x %d]. Skipping.', ...
                currentFileName, N_ROTORS, size(Estimated_ForceEff_Matrix, 1), size(Estimated_ForceEff_Matrix, 2));
            continue;
        end

        % *** NEW LOGIC (V4) ***
        % Check Torque dimensions and handle W^2 / Wdot terms
        if size(Estimated_TorqueEff_Matrix, 1) ~= 3
             warning('File %s: TorqueEffectiveness matrix has wrong row count. Expected 3, got %d. Skipping.', ...
                currentFileName, size(Estimated_TorqueEff_Matrix, 1));
             continue;
        end
        
        if size(Estimated_TorqueEff_Matrix, 2) == (2 * N_ROTORS)
            fprintf('  -> Torque matrix is [3 x %d], RLS likely included W_dot terms.\n', size(Estimated_TorqueEff_Matrix, 2));
            % Select only the first N_ROTORS columns (W^2 term)
            Estimated_TorqueEff_Matrix = Estimated_TorqueEff_Matrix(:, 1:N_ROTORS);
            fprintf('  -> Sliced Torque matrix to [3 x %d] (W^2 term only).\n', N_ROTORS);
            
        elseif size(Estimated_TorqueEff_Matrix, 2) ~= N_ROTORS
            % It's not [3 x N_ROTORS] and it's not [3 x 2*N_ROTORS], so it's an error
            warning('File %s: TorqueEffectiveness matrix has unexpected column count. Expected %d or %d, got %d. Skipping.', ...
                currentFileName, N_ROTORS, 2*N_ROTORS, size(Estimated_TorqueEff_Matrix, 2));
            continue;
        end
        % *** END NEW LOGIC ***

        B_Matrix = [Estimated_ForceEff_Matrix; Estimated_TorqueEff_Matrix]; % [6 x N_ROTORS]
        
        % --- e. Split and label motor parameters ---
        % Estimated_MotorParams_Matrix is [4 x N_ROTORS]
        if size(Estimated_MotorParams_Matrix, 2) ~= N_ROTORS
             warning('File %s: MotorParams matrix has wrong rotor count. Expected %d, got %d. Skipping.', ...
                currentFileName, N_ROTORS, size(Estimated_MotorParams_Matrix, 2));
            continue;
        end
        if size(Estimated_MotorParams_Matrix, 1) < 3
             warning('File %s: MotorParams matrix has too few parameters. Expected at least 3, got %d. Skipping.', ...
                currentFileName, size(Estimated_MotorParams_Matrix, 1));
            continue;
        end

        M_GainU = Estimated_MotorParams_Matrix(1, :);   % Row 1
        M_coeffW = Estimated_MotorParams_Matrix(2, :);  % Row 2
        M_CoeffW2 = Estimated_MotorParams_Matrix(3, :); % Row 3
        
        % --- f. Calculate Predicted Wrench ---
        % [N_ROTORS x Time] OMEGA DATA
        Omega_data = squeeze(omegaTs.Data); % [N_ROTORS x Time]
        
        if isempty(Omega_data)
            warning('File %s: omegaTs.Data is empty. Cannot calculate Predicted_Wrench. Skipping.', currentFileName);
            continue;
        end

        % Ensure Omega data is [N_ROTORS x Time]
        if size(Omega_data, 1) ~= N_ROTORS
            if size(Omega_data, 2) == N_ROTORS
                % This case should be rare with 3D data, but good to have
                warning('File %s: Omega data appears to be [Time x N_ROTORS]. Transposing.', currentFileName);
                Omega_data = Omega_data';
            else
                warning('File %s: Omega data has unexpected dimensions. Expected %d rotors, got %d. Skipping.', ...
                        currentFileName, N_ROTORS, size(Omega_data, 1));
                continue;
            end
        end
        
        Omega_squared_data = Omega_data .^ 2;
                       
        % Calculate wrench: Wrench = B * (Omega^2)
        % B_Matrix: [6 x N_ROTORS]
        % Omega_squared_data: [N_ROTORS x Time]
        % Result: [6 x Time]
        Predicted_Wrench_data = B_Matrix * Omega_squared_data;
        Nom_Predicted_Wrench_data = B_nom * Omega_squared_data;
        
        % Create a new timeseries
        % The constructor needs data as [Time x States]
        % Our Predicted_Wrench_data is [6 x Time], so we must transpose it.
        Predicted_Wrench = timeseries(Predicted_Wrench_data', omegaTs.Time, 'Name', 'Predicted_Wrench');
        Nom_Predicted_Wrench = timeseries(Nom_Predicted_Wrench_data', omegaTs.Time,'Name', 'Nom_Predicted_Wrench');
        
        % Rename omegaTs to Omega for consistent saving
        Omega = omegaTs;
        
        % --- g. Save results to new file ---
        outputFileName = ['preprocessed_' currentFileName];
        outputFullFilePath = fullfile(outputPath, outputFileName);
        
        % Fixed typo here (V4): Nom_Predicted_WWrench -> Nom_Predicted_Wrench
        save(outputFullFilePath, 'B_Matrix', 'M_GainU', 'M_coeffW', 'M_CoeffW2', ...
             'Omega', 'Real_Wrench', 'Predicted_Wrench', ...
             'features_i', 'Motor', 'Uav','B_nom','Nom_Predicted_Wrench');
        
        processedCount = processedCount + 1;
        
    catch ME
        % Catch errors during loading or processing of a single file
        fprintf(2, 'ERROR processing file %s: %s\n', currentFileName, ME.message);
        fprintf(2, 'At line %d in function %s\n', ME.stack(1).line, ME.stack(1).name);
        fprintf(2, 'Skipping this file.\n');
    end
end

%% --- 4. Final Report ---
fprintf('\n-------------------------------------------------\n');
fprintf('Preprocessing Complete.\n');
fprintf('Successfully processed and saved %d out of %d files.\n', processedCount, numFiles);
fprintf('Preprocessed files are located in: %s\n', outputPath);