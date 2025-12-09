%% PREPROCESS UKF AND RLS SIMULATION DATA (UPDATED)
%
% This script loads raw simulation results from two selected folders
% (UKF Data and RLS Data), processes them to extract estimated parameters
% using the updated state definitions, and saves the results into a 
% new 'UKF_RLS_Preprocessed' folder.
%
% UPDATES:
%   - Matches state indexing from "Single Sample" script (14 + N_ROTORS).
%   - Uses Force/Torque Effectiveness for RLS B-Matrix construction.
%
clear; clc; close all;

%% --- Configuration ---
OUTPUT_FOLDER = 'UKF_RLS_Preprocessed_v2';
RLS_SAMPLE_TIME = 17.0; % Time in seconds to sample the RLS data
TRIM_WINDOW = 5.0;      % Seconds from end to average for UKF

%% --- 1. Select Input Folders ---
% 1.1 Select UKF Folder
fprintf('Step 1: Select the folder containing UKF result files...\n');
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end

resultsPath = uigetdir(startPath, 'Select Folder Containing UKF Results');
if isequal(resultsPath, 0)
    disp('User cancelled UKF folder selection.'); return;
end

% 1.2 Select RLS Folder
fprintf('Step 2: Select the folder containing RLS result files...\n');
rlsResultsPath = uigetdir(startPath, 'Select Folder Containing RLS Results');
if isequal(rlsResultsPath, 0)
    disp('User cancelled RLS folder selection.'); return;
end

resultFiles = dir(fullfile(resultsPath, 'estimation_*.mat'));
numFiles = length(resultFiles);
if numFiles == 0
    error('No estimation_*.mat files found in: %s', resultsPath);
end

fprintf('Found %d result files. Starting preprocessing...\n', numFiles);

%% --- 2. Create Output Folder ---
[parentFolder, ~] = fileparts(resultsPath);
outputPath = fullfile(parentFolder, OUTPUT_FOLDER);
if ~isfolder(outputPath)
    mkdir(outputPath);
    fprintf('Created output directory: %s\n', outputPath);
else
    fprintf('Output directory exists: %s\n', outputPath);
end

%% --- 3. Process Each File ---
processedCount = 0;

for i = 1:numFiles
    currentFileName = resultFiles(i).name;
    fullFilePath = fullfile(resultsPath, currentFileName);
    
    fprintf('Processing (%d/%d): %s ... ', i, numFiles, currentFileName);
    
    % --- Smart File Matching ---
    sampleIdMatch = regexp(currentFileName, 'Sample\d+', 'match');
    if isempty(sampleIdMatch)
        fprintf('[SKIP] No "SampleXXX" ID found.\n'); continue;
    end
    sampleID = sampleIdMatch{1};
    
    % Find corresponding RLS file
    rlsDirResult = dir(fullfile(rlsResultsPath, ['*' sampleID '*.mat']));
    
    if isempty(rlsDirResult)
        fprintf('[SKIP] RLS file missing for %s.\n', sampleID); continue;
    elseif length(rlsDirResult) > 1
        fprintf('[SKIP] Multiple RLS files for %s.\n', sampleID); continue;
    end
    
    rlsFileName = rlsDirResult(1).name;
    rlsFullFilePath = fullfile(rlsResultsPath, rlsFileName);
    
    try
        % =================================================================
        % PART A: PROCESS UKF DATA
        % =================================================================
        S = load(fullFilePath);
        
        % Validation
        if ~isfield(S, 'simOut') || ~isprop(S.simOut, 'UKFData') || ...
           ~isfield(S, 'Uav') || ~isfield(S.Uav, 'N_ROTORS')
            fprintf('[SKIP] Corrupt/Old UKF file structure.\n'); continue;
        end
        
        ukfDataContainer = S.simOut.UKFData;
        ukfTs = ukfDataContainer.UKF_DATA;
        Omega = ukfDataContainer.Omega;
        Real_Wrench = ukfDataContainer.Real_Wrench;
        B_nom = S.B_matrix_nominal;
        N_ROTORS = S.Uav.N_ROTORS;
        
        % --- Time Window Selection (Last 5s) ---
        endTime = ukfTs.Time(end);
        startTime = max(ukfTs.Time(1), endTime - TRIM_WINDOW);
        ts_subset = getsampleusingtime(ukfTs, startTime, endTime);
        
        if isempty(ts_subset.Data)
            fprintf('[SKIP] Empty time window.\n'); continue;
        end
        
        % --- Extract Parameters (Updated Indexing) ---
        % In Single Sample Script: parametersWindow = ts_subset.Data(:, 14+N_ROTORS:end);
        % This logic assumes the state vector is: [1..13+N] = Base States, [14+N..End] = Params
        
        idx_params_start = 14 + N_ROTORS; 
        
        if size(ts_subset.Data, 2) < idx_params_start
            fprintf('[SKIP] State vector smaller than expected index (%d).\n', idx_params_start);
            continue;
        end
        
        % Extract parameter columns
        parametersWindow = ts_subset.Data(:, idx_params_start:end);
        
        % Calculate Statistics
        est_params_avg = mean(parametersWindow, 1);
        est_params_var = var(parametersWindow, 0, 1);
        
        % --- Extract B-Matrix ---
        % B-Matrix is the first 6*N states of the parameter block
        idx_B_len = 6 * N_ROTORS;
        
        if length(est_params_avg) < idx_B_len
             fprintf('[SKIP] Param vector too short for B-Matrix.\n'); continue;
        end
        
        B_vector = est_params_avg(1 : idx_B_len);
        B_Matrix_UKF = reshape(B_vector, [6, N_ROTORS]);
        
        B_vector_var = est_params_var(1 : idx_B_len);
        B_Matrix_Var_UKF = reshape(B_vector_var, [6, N_ROTORS]);
        
        % (Optional) Extract Remaining Parameters if they exist
        % Assuming order: B_Matrix -> GainU -> CoeffW -> CoeffW2
        rem_params = est_params_avg(idx_B_len+1:end);
        % If you need these, you can assign them based on N_ROTORS lengths
        % leaving as placeholder arrays to prevent script errors if lengths vary
        
        % =================================================================
        % PART B: PROCESS RLS DATA
        % =================================================================
        S_rls = load(rlsFullFilePath);
        
        if ~isprop(S_rls.simOut, 'RLSData')
             fprintf('[SKIP] RLS file missing simOut.RLSData.\n'); continue;
        end
        
        rlsData = S_rls.simOut.RLSData;
        forceEffTs = rlsData.ForceEffectiveness;
        torqueEffTs = rlsData.TorqueEffectiveness;
        motorParamsTs = rlsData.MotorParams; % Needed for time reference
        
        % Find Index for Sample Time
        [~, time_idx] = min(abs(motorParamsTs.Time - RLS_SAMPLE_TIME));
        if motorParamsTs.Time(end) < (RLS_SAMPLE_TIME - 1.0)
            time_idx = length(motorParamsTs.Time);
            % warning('RLS short data'); % Silent warning for batch processing
        end
        
        % Extract Matrices
        Est_ForceEff = forceEffTs.Data(:, :, time_idx);
        Est_TorqueEff = torqueEffTs.Data(:, :, time_idx);
        B_Matrix_RLS = [Est_ForceEff; Est_TorqueEff];
        
        % Keep the MotorParams snapshot just in case
        MotorParams_RLS = motorParamsTs.Data(:, :, time_idx);

        % =================================================================
        % PART C: PREDICT WRENCHES
        % =================================================================
        Omega_sq = Omega.Data .^ 2;
        
        % 1. Nominal
        Wrench_Nom_Data = (B_nom * Omega_sq')';
        Nom_Predicted_Wrench = timeseries(Wrench_Nom_Data, Omega.Time, 'Name', 'Nominal');
        
        % 2. UKF
        Wrench_UKF_Data = (B_Matrix_UKF * Omega_sq')';
        Predicted_Wrench_UKF = timeseries(Wrench_UKF_Data, Omega.Time, 'Name', 'UKF');
        
        % 3. RLS
        Wrench_RLS_Data = (B_Matrix_RLS * Omega_sq')';
        Predicted_Wrench_RLS = timeseries(Wrench_RLS_Data, Omega.Time, 'Name', 'RLS');
        
        % =================================================================
        % PART D: SAVE
        % =================================================================
        outputFileName = ['preprocessed_' currentFileName];
        outputFullFilePath = fullfile(outputPath, outputFileName);
        
        % Determine what other structs to save (Context)
        features_i = S.features_i;
        Motor = S.Motor;
        Uav = S.Uav;
        
        save(outputFullFilePath, ...
             'Omega', 'Real_Wrench', 'features_i', 'Motor', 'Uav', ...
             'B_nom', 'Nom_Predicted_Wrench', ...
             'B_Matrix_UKF', 'B_Matrix_Var_UKF', 'Predicted_Wrench_UKF', ...
             'B_Matrix_RLS', 'Predicted_Wrench_RLS', 'MotorParams_RLS');
             
        fprintf('Done.\n');
        processedCount = processedCount + 1;
        
    catch ME
        fprintf('\n  [ERROR] %s\n', ME.message);
    end
end

%% --- 4. Final Report ---
fprintf('\n-------------------------------------------------\n');
fprintf('Preprocessing Complete.\n');
fprintf('Processed: %d / %d files.\n', processedCount, numFiles);
fprintf('Output: %s\n', outputPath);