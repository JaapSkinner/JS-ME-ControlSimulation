%% PREPROCESS UKF AND RLS SIMULATION DATA
%
% This script loads raw simulation results from two selected folders
% (UKF Data and RLS Data), processes them to extract estimated parameters,
% and saves the results into a new 'UKF_RLS_Preprocessed' folder.
%
% Processing Steps:
%   1. Asks user to select the folder containing UKF 'estimation_*.mat' files.
%   2. Asks user to select the folder containing RLS 'estimation_*.mat' files.
%   3. Creates a new folder 'UKF_RLS_Preprocessed' at the same level.
%   4. For each file found in the UKF folder:
%      a. Identifies the corresponding RLS file by matching 'SampleXXX'.
%      b. Loads the UKF .mat file and processes statistics (Mean/Var) for
%         last 5 seconds.
%      c. Loads the corresponding RLS .mat file.
%      d. Extracts RLS estimates at a specific time snapshot (default 50s).
%      e. Calculates 'Predicted_Wrench' for UKF, RLS, and Nominal models
%         using the Omega timeseries from the simulation.
%      f. Saves combined results to the output folder.
%
clear; clc; close all;

% --- Configuration ---
OUTPUT_FOLDER = 'UKF_RLS_Preprocessed';
RLS_SAMPLE_TIME = 50.0; % Time in seconds to sample the RLS data

%% --- 1. Select Input Folders ---

% 1.1 Select UKF Folder
fprintf('Step 1: Select the folder containing UKF result files...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath); startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation'); end
    if ~isfolder(startPath); startPath = pwd; end
catch
    startPath = pwd;
end

resultsPath = uigetdir(startPath, 'Select Folder Containing UKF Results');
if isequal(resultsPath, 0)
    disp('User selected Cancel for UKF folder. Script terminated.');
    return;
end

% 1.2 Select RLS Folder
fprintf('Step 2: Select the folder containing RLS result files...\n');
rlsResultsPath = uigetdir(startPath, 'Select Folder Containing RLS Results');
if isequal(rlsResultsPath, 0)
    disp('User selected Cancel for RLS folder. Script terminated.');
    return;
end

resultFiles = dir(fullfile(resultsPath, 'estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0
    error('No estimation_*.mat files found in the UKF directory: %s', resultsPath);
end

fprintf('Found %d result files. Starting preprocessing...\n', numFiles);

%% --- 2. Create Output Folder ---
[parentFolder, ~] = fileparts(resultsPath);
outputPath = fullfile(parentFolder, OUTPUT_FOLDER);
if ~isfolder(outputPath)
    fprintf('Creating output directory: %s\n', outputPath);
    mkdir(outputPath);
else
    fprintf('Output directory already exists: %s\n', outputPath);
end

%% --- 3. Process Each File ---
processedCount = 0;
for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(resultsPath, currentFileName);
    
    fprintf('Processing file (%d/%d): %s\n', i, numFiles, currentFileName);
    
    % --- Smart File Matching ---
    % UKF and RLS files might have different timestamps. Match by 'SampleXXX'.
    sampleIdMatch = regexp(currentFileName, 'Sample\d+', 'match');
    
    if isempty(sampleIdMatch)
        fprintf(2, '  [SKIP] Could not extract "SampleXXX" ID from filename.\n');
        continue;
    end
    
    sampleID = sampleIdMatch{1};
    
    % Find RLS file with the same Sample ID
    rlsFilePattern = fullfile(rlsResultsPath, ['*' sampleID '*.mat']);
    rlsDirResult = dir(rlsFilePattern);
    
    if isempty(rlsDirResult)
        fprintf(2, '  [SKIP] No corresponding RLS file found for ID: %s\n', sampleID);
        continue;
    elseif length(rlsDirResult) > 1
        fprintf(2, '  [SKIP] Ambiguous: Multiple RLS files found for ID: %s\n', sampleID);
        continue;
    end
    
    rlsFileName = rlsDirResult(1).name;
    rlsFullFilePath = fullfile(rlsResultsPath, rlsFileName);
    
    % fprintf('  -> Matched RLS file: %s\n', rlsFileName); 

    try
        % =================================================================
        % PART A: PROCESS UKF DATA (Original Logic)
        % =================================================================
        S = load(fullFilePath);
        
        % Ensure all required data is present
        if ~isfield(S, 'simOut') || ~isprop(S.simOut, 'UKFData')
            fprintf(2, '  [SKIP] File missing simOut.UKFData.\n');
            continue;
        end
        
        ukfDataContainer = S.simOut.UKFData;
        
        if ~isfield(ukfDataContainer, 'UKF_DATA') || ~isa(ukfDataContainer.UKF_DATA, 'timeseries')
             fprintf(2, '  [SKIP] File missing UKF_DATA timeseries.\n');
             continue;
        end
        if ~isfield(ukfDataContainer, 'Omega') || ~isa(ukfDataContainer.Omega, 'timeseries')
             fprintf(2, '  [SKIP] File missing Omega timeseries.\n');
             continue;
        end
        if ~isfield(S, 'features_i') || ~isfield(S, 'Motor') || ~isfield(S, 'Uav')
             fprintf(2, '  [SKIP] File missing base objects (features/Motor/Uav).\n');
             continue;
        end
        if ~isfield(S.Uav, 'N_ROTORS')
             fprintf(2, '  [SKIP] File missing Uav.N_ROTORS.\n');
             continue;
        end
        if ~isfield(S, 'B_matrix_nominal')
             fprintf(2, '  [SKIP] File missing B_matrix_nominal.\n');
             continue;
        end
        
        ukfTs = ukfDataContainer.UKF_DATA;
        Omega = ukfDataContainer.Omega;
        Real_Wrench = ukfDataContainer.Real_Wrench;
        B_nom = S.B_matrix_nominal;
        
        features_i = S.features_i;
        Motor = S.Motor;
        Uav = S.Uav;
        N_ROTORS = S.Uav.N_ROTORS; 

        % --- Get time window (last 5 seconds) ---
        endTime = ukfTs.Time(end);
        startTime = endTime - 3.0;
        
        if startTime < ukfTs.Time(1)
            startTime = ukfTs.Time(1);
            warning('Simulation in %s is shorter than 5s. Using all available data.', currentFileName);
        end
        
        ts_subset = getsampleusingtime(ukfTs, startTime, endTime);
        
        if isempty(ts_subset.Data)
            fprintf(2, '  [SKIP] Empty dataset for the selected time window.\n');
            continue;
        end
        
        % --- Select states 22:end and average ---
        numStates = size(ts_subset.Data, 2);
        if numStates < 22
            fprintf(2, '  [SKIP] Insufficient states (%d). Expected >= 22.\n', numStates);
            continue;
        end
        
        parametersWindow = ts_subset.Data(:, 22:end);
        Estimated_Parameters_Avg = mean(parametersWindow, 1);
        Estimated_Parameters_Var = var(parametersWindow, 0, 1);
        
        % --- Split and label UKF estimated parameters ---
        num_B_states = 6 * N_ROTORS;
        num_GainU_states = N_ROTORS;
        num_coeffW_states = N_ROTORS;
        num_CoeffW2_states = N_ROTORS;
        
        total_expected_params = num_B_states + num_GainU_states + num_coeffW_states + num_CoeffW2_states;
        
        if size(Estimated_Parameters_Avg, 2) < total_expected_params
            fprintf(2, '  [SKIP] Expected %d parameter states, found %d.\n', total_expected_params, size(Estimated_Parameters_Avg, 2));
            continue;
        end
        
        idx_B_end = num_B_states;
        idx_GainU_end = idx_B_end + num_GainU_states;
        idx_coeffW_end = idx_GainU_end + num_coeffW_states;
        idx_CoeffW2_end = idx_coeffW_end + num_CoeffW2_states;
        
        % UKF Estimates
        B_matrix_vector = Estimated_Parameters_Avg(1 : idx_B_end);
        B_Matrix_UKF = reshape(B_matrix_vector, [6, N_ROTORS]);
        
        M_GainU_UKF = Estimated_Parameters_Avg(idx_B_end + 1 : idx_GainU_end);
        M_coeffW_UKF = Estimated_Parameters_Avg(idx_GainU_end + 1 : idx_coeffW_end);
        M_CoeffW2_UKF = Estimated_Parameters_Avg(idx_coeffW_end + 1 : idx_CoeffW2_end);
        
        % UKF Variances
        B_matrix_vector_Var = Estimated_Parameters_Var(1 : idx_B_end);
        B_Matrix_Var_UKF = reshape(B_matrix_vector_Var, [6, N_ROTORS]);
        M_GainU_Var_UKF = Estimated_Parameters_Var(idx_B_end + 1 : idx_GainU_end);
        M_coeffW_Var_UKF = Estimated_Parameters_Var(idx_GainU_end + 1 : idx_coeffW_end);
        M_CoeffW2_Var_UKF = Estimated_Parameters_Var(idx_coeffW_end + 1 : idx_CoeffW2_end);

        % =================================================================
        % PART B: PROCESS RLS DATA (New Logic)
        % =================================================================
        S_est = load(rlsFullFilePath);
        
        if ~isprop(S_est.simOut, 'RLSData')
             fprintf(2, '  [SKIP] RLS file missing simOut.RLSData.\n');
             continue;
        end
        
        rlsDataContainer = S_est.simOut.RLSData; 
        motorParamsTs = rlsDataContainer.MotorParams;
        forceEffTs = rlsDataContainer.ForceEffectiveness;
        torqueEffTs = rlsDataContainer.TorqueEffectiveness;
        
        % Find the index closest to RLS_SAMPLE_TIME (e.g., 50s)
        [~, time_idx] = min(abs(motorParamsTs.Time - RLS_SAMPLE_TIME));
        
        % Check if the index is valid and not just the last point if sim ended early
        if motorParamsTs.Time(end) < (RLS_SAMPLE_TIME - 1.0)
            warning('RLS Simulation for %s ended at %.2fs, before sample time %.2fs. Using last value.', ...
                rlsFileName, motorParamsTs.Time(end), RLS_SAMPLE_TIME);
            time_idx = length(motorParamsTs.Time);
        end

        % Extract RLS Matrices at the specific time index
        MotorParams_RLS = motorParamsTs.Data(:, :, time_idx);
        Estimated_ForceEff_Matrix = forceEffTs.Data(:, :, time_idx);
        Estimated_TorqueEff_Matrix = torqueEffTs.Data(:, :, time_idx);
        
        % Construct RLS B Matrix
        B_Matrix_RLS = [Estimated_ForceEff_Matrix; Estimated_TorqueEff_Matrix];

        % =================================================================
        % PART C: CALCULATE PREDICTED WRENCHES
        % =================================================================
        
        Omega_data = Omega.Data; % [Time x N_ROTORS]
        Omega_squared_data = Omega_data .^ 2;
        
        % 1. UKF Prediction
        Predicted_Wrench_UKF_data = (B_Matrix_UKF * Omega_squared_data')';
        Predicted_Wrench_UKF = timeseries(Predicted_Wrench_UKF_data, Omega.Time, 'Name', 'Predicted_Wrench_UKF');
        
        % 2. Nominal Prediction
        Nom_Predicted_Wrench_data = (B_nom * Omega_squared_data')';
        Nom_Predicted_Wrench = timeseries(Nom_Predicted_Wrench_data, Omega.Time,'Name', 'Nom_Predicted_Wrench');
        
        % 3. RLS Prediction
        Predicted_Wrench_RLS_data = (B_Matrix_RLS * Omega_squared_data')';
        Predicted_Wrench_RLS = timeseries(Predicted_Wrench_RLS_data, Omega.Time, 'Name', 'Predicted_Wrench_RLS');

        % =================================================================
        % PART D: SAVE RESULTS
        % =================================================================
        outputFileName = ['preprocessed_' currentFileName];
        outputFullFilePath = fullfile(outputPath, outputFileName);
        
        save(outputFullFilePath, ...
             ... % Shared Data
             'Omega', 'Real_Wrench', 'features_i', 'Motor', 'Uav', ...
             ... % Nominal
             'B_nom', 'Nom_Predicted_Wrench', ...
             ... % UKF Results
             'B_Matrix_UKF', 'M_GainU_UKF', 'M_coeffW_UKF', 'M_CoeffW2_UKF', ...
             'B_Matrix_Var_UKF', 'M_GainU_Var_UKF', 'M_coeffW_Var_UKF', 'M_CoeffW2_Var_UKF', ...
             'Predicted_Wrench_UKF', ...
             ... % RLS Results
             'B_Matrix_RLS', 'MotorParams_RLS', 'Predicted_Wrench_RLS');
        
        processedCount = processedCount + 1;
        
    catch ME
        fprintf(2, '  [ERROR] %s\n', ME.message);
        fprintf(2, '  [SKIP] Failed to process this file.\n');
    end
end

%% --- 4. Final Report ---
fprintf('\n-------------------------------------------------\n');
fprintf('Preprocessing Complete.\n');
fprintf('Successfully processed and saved %d out of %d files.\n', processedCount, numFiles);
fprintf('Preprocessed files are located in: %s\n', outputPath);