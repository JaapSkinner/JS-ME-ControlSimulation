%% PREPROCESS UKF SIMULATION DATA
%
% This script loads raw simulation results from a selected folder
% (expected to be 'UKFData'), processes the UKF data to extract
% estimated parameters, and saves the results into a new
% 'UKF_Preprocessed' folder.
%
% Processing Steps:
%   1. Asks user to select the folder containing 'estimation_*.mat' files.
%   2. Creates a new folder 'UKF_Preprocessed' at the same level.
%   3. For each file:
%      a. Loads the .mat file.
%      b. Accesses simOut.UKFData.UKF_DATA (a timeseries object).
%      c. Finds the time window for the last 5 seconds of the simulation.
%      d. Extracts data for states 22:end within this time window.
%      e. Calculates the mean and variance of this data.
%      f. Splits the averaged vector into B_Matrix, M_GainU, M_coeffW,
%         and M_CoeffW2 (and their variances) based on Uav.N_ROTORS.
%      g. Calculates a 'Predicted_Wrench' timeseries using the estimated B_Matrix
%         and the Omega timeseries (assuming Wrench = B * Omega^2).
%      h. Saves these new parameters, 'Omega', 'Real_Wrench', 'Predicted_Wrench',
%         'features_i', 'Motor', and 'Uav' to a new file in the
%         'UKF_Preprocessed' folder.
%
clear; clc; close all;

%% --- 1. Select Input Folder ---
fprintf('Step 1: Select the folder containing UKF result files...\n');
try
    % Try to find the project root and navigate to the default data folder
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
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

resultsPath = uigetdir(startPath, 'Select the Folder Containing UKF Monte Carlo Results');
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
% We'll create the new folder alongside the one we selected
[parentFolder, ~] = fileparts(resultsPath);
outputPath = fullfile(parentFolder, 'UKF_Preprocessed4');

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

    try
        % Load the file into a struct 'S' to keep workspace clean
        S = load(fullFilePath);

        % --- a. Extract data from loaded struct ---
        % Ensure all required data is present
        if ~isfield(S, 'simOut') || ~isprop(S.simOut, 'UKFData')
            warning('File %s is missing simOut.UKFData. Skipping.', currentFileName);
            continue;
        end
        
        ukfDataContainer = S.simOut.UKFData;
        
        % Check for the timeseries and other objects
        if ~isfield(ukfDataContainer, 'UKF_DATA') || ~isa(ukfDataContainer.UKF_DATA, 'timeseries')
             warning('File %s is missing UKF_DATA timeseries. Skipping.', currentFileName);
             continue;
        end
        if ~isfield(ukfDataContainer, 'Omega') || ~isa(ukfDataContainer.Omega, 'timeseries')
             warning('File %s is missing Omega timeseries. Skipping.', currentFileName);
             continue;
        end
        if ~isfield(S, 'features_i') || ~isfield(S, 'Motor') || ~isfield(S, 'Uav')
             warning('File %s is missing one or more base objects (features_i, Motor, Uav). Skipping.', currentFileName);
             continue;
        end
        % Check for N_ROTORS needed for splitting
        if ~isfield(S.Uav, 'N_ROTORS')
             warning('File %s is missing Uav.N_ROTORS. Skipping.', currentFileName);
             continue;
        end
        % Check for nominal B Matrix
        if ~isfield(S, 'B_matrix_nominal')
             warning('File %s is missing B_matrix_nom. Skipping.', currentFileName);
             continue;
        end

        ukfTs = ukfDataContainer.UKF_DATA;
        Omega = ukfDataContainer.Omega;
        Real_Wrench = ukfDataContainer.Real_Wrench;
        B_nom = S.B_matrix_nominal;
        
        features_i = S.features_i;
        Motor = S.Motor;
        Uav = S.Uav;
        N_ROTORS = S.Uav.N_ROTORS; % Get N_ROTORS for parameter splitting

        % --- b. Get time window (last 5 seconds) ---
        endTime = ukfTs.Time(end);
        startTime = endTime - 3.0;
        
        % Handle simulations that are shorter than 5 seconds
        if startTime < ukfTs.Time(1)
            startTime = ukfTs.Time(1);
            warning('Simulation in %s is shorter than 5s. Using all available data.', currentFileName);
        end

        % --- c. Get data from the time window ---
        % getsampleusingtime is inclusive of start and end times
        ts_subset = getsampleusingtime(ukfTs, startTime, endTime);
        
        if isempty(ts_subset.Data)
            warning('File %s resulted in empty dataset for the last 5s. Skipping.', currentFileName);
            continue;
        end

        % --- d. Select states 22:end and average ---
        % The data is [Time x States]. We select columns 22 to end.
        numStates = size(ts_subset.Data, 2);
        if numStates < 22
            warning('File %s has only %d states. Cannot extract states 22:end. Skipping.', currentFileName, numStates);
            continue;
        end
        
        parametersWindow = ts_subset.Data(:, 22:end);
        
        % Calculate the mean along the time dimension (dim 1)
        % This results in a 1-row vector of averages for each parameter
        Estimated_Parameters_Avg = mean(parametersWindow, 1);
        
        % Calculate the variance (N-1) along the time dimension (dim 1)
        Estimated_Parameters_Var = var(parametersWindow, 0, 1);

        % --- e. Split and label estimated parameters ---
        
        % Define the number of states for each parameter group
        num_B_states = 6 * N_ROTORS;
        num_GainU_states = N_ROTORS;
        num_coeffW_states = N_ROTORS;
        num_CoeffW2_states = N_ROTORS; % Assuming MCoeffW2 is also N_ROTORS long
        
        total_expected_params = num_B_states + num_GainU_states + num_coeffW_states + num_CoeffW2_states;
        
        if size(Estimated_Parameters_Avg, 2) < total_expected_params
            warning('File %s: Expected %d parameter states (from state 22) but found only %d. Skipping.', ...
                    currentFileName, total_expected_params, size(Estimated_Parameters_Avg, 2));
            continue;
        end
        
        % Calculate indices for splitting the averaged vector
        % These are relative to the start of Estimated_Parameters_Avg (state 22)
        idx_B_end = num_B_states;
        idx_GainU_end = idx_B_end + num_GainU_states;
        idx_coeffW_end = idx_GainU_end + num_coeffW_states;
        idx_CoeffW2_end = idx_coeffW_end + num_CoeffW2_states;
        
        % Extract and reshape
        B_matrix_vector = Estimated_Parameters_Avg(1 : idx_B_end);
        B_Matrix = reshape(B_matrix_vector, [6, N_ROTORS]);
        
        M_GainU = Estimated_Parameters_Avg(idx_B_end + 1 : idx_GainU_end);
        M_coeffW = Estimated_Parameters_Avg(idx_GainU_end + 1 : idx_coeffW_end);
        M_CoeffW2 = Estimated_Parameters_Avg(idx_coeffW_end + 1 : idx_CoeffW2_end);

        % Extract and reshape variance
        B_matrix_vector_Var = Estimated_Parameters_Var(1 : idx_B_end);
        B_Matrix_Var = reshape(B_matrix_vector_Var, [6, N_ROTORS]);
        
        M_GainU_Var = Estimated_Parameters_Var(idx_B_end + 1 : idx_GainU_end);
        M_coeffW_Var = Estimated_Parameters_Var(idx_GainU_end + 1 : idx_coeffW_end);
        M_CoeffW2_Var = Estimated_Parameters_Var(idx_coeffW_end + 1 : idx_CoeffW2_end);

        % --- f. Calculate Predicted Wrench ---
        % Use the estimated B_Matrix and the full Omega timeseries.
        % Per user's correction, the B_Matrix maps omegas to wrench.
        % We assume the common physical model: Wrench = B_Matrix * (Omega^2)
        % The estimated M_... parameters are for motor dynamics and not used here.
        
        Omega_data = Omega.Data; % [Time x N_ROTORS]
        
        % Square the omegas (element-wise)
        Omega_squared_data = Omega_data .^ 2;
                       
        % Calculate wrench: Wrench = B * (Omega^2)
        % B_Matrix is [6 x N_ROTORS]
        % Omega_squared_data is [Time x N_ROTORS]
        % We need (B * (Omega_squared)')' to get [Time x 6]
        Predicted_Wrench_data = (B_Matrix * Omega_squared_data')';
        Nom_Predicted_Wrench_data = (B_nom * Omega_squared_data')';
        % Create a new timeseries for the predicted wrench
        % Use the time vector from the Omega timeseries
        Predicted_Wrench = timeseries(Predicted_Wrench_data, Omega.Time, 'Name', 'Predicted_Wrench');
        Nom_Predicted_Wrench = timeseries(Nom_Predicted_Wrench_data, Omega.Time,'Name', 'Nom_Predicted_Wrench');
        % --- g. Save results to new file ---
        outputFileName = ['preprocessed_' currentFileName];
        outputFullFilePath = fullfile(outputPath, outputFileName);
        
        % Save the new, labeled parameters and their variance
        save(outputFullFilePath, 'B_Matrix', 'M_GainU', 'M_coeffW', 'M_CoeffW2', ...
             'B_Matrix_Var', 'M_GainU_Var', 'M_coeffW_Var', 'M_CoeffW2_Var', ...
             'Omega', 'Real_Wrench', 'Predicted_Wrench', ...
             'features_i', 'Motor', 'Uav','B_nom','Nom_Predicted_Wrench');
        
        processedCount = processedCount + 1;

    catch ME
        % Catch errors during loading or processing of a single file
        fprintf(2, 'ERROR processing file %s: %s\f', currentFileName, ME.message);
        fprintf(2, 'Skipping this file.\n');
    end
end

%% --- 4. Final Report ---
fprintf('\n-------------------------------------------------\n');
fprintf('Preprocessing Complete.\n');
fprintf('Successfully processed and saved %d out of %d files.\n', processedCount, numFiles);
fprintf('Preprocessed files are located in: %s\n', outputPath);