%% CHECK NOMINAL B MATRIX CONSISTENCY
% Checks if 'B_matrix_nominal' is identical across all result files.

clear; clc;

% 1. Select the folder containing the .mat files
resultsPath = uigetdir(pwd, 'Select Folder with estimation_*.mat files');
if isequal(resultsPath, 0), return; end

files = dir(fullfile(resultsPath, '*.mat'));
numFiles = length(files);

if numFiles == 0
    error('No .mat files found in %s', resultsPath);
end

fprintf('Checking %d files in: %s\n\n', numFiles, resultsPath);

% 2. Load the First File as the Reference
firstFile = fullfile(resultsPath, files(1).name);
refData = load(firstFile, 'B_matrix_nominal');

if ~isfield(refData, 'B_matrix_nominal')
    error('CRITICAL: First file (%s) is missing B_matrix_nominal!', files(1).name);
end

B_ref = refData.B_matrix_nominal;
fprintf('Reference loaded from: %s\n', files(1).name);
disp('Reference B_matrix_nominal:');
disp(B_ref);

% 3. Loop and Compare
mismatchCount = 0;
tolerance = 1e-12; % Floating point tolerance

fprintf('--- Starting Comparison ---\n');

for i = 2:numFiles
    currentName = files(i).name;
    currentPath = fullfile(resultsPath, currentName);
    
    % Load only the specific variable to save memory/time
    try
        currData = load(currentPath, 'B_matrix_nominal');
        
        if ~isfield(currData, 'B_matrix_nominal')
            fprintf(2, '[MISSING] %s does not contain B_matrix_nominal\n', currentName);
            mismatchCount = mismatchCount + 1;
            continue;
        end
        
        B_curr = currData.B_matrix_nominal;
        
        % Check Dimensions
        if ~isequal(size(B_ref), size(B_curr))
            fprintf(2, '[SIZE FAIL] %s: Size mismatch. Ref: [%s], Curr: [%s]\n', ...
                currentName, num2str(size(B_ref)), num2str(size(B_curr)));
            mismatchCount = mismatchCount + 1;
            continue;
        end
        
        % Check Values
        maxDiff = max(abs(B_ref(:) - B_curr(:)));
        
        if maxDiff > tolerance
            fprintf(2, '[VALUE FAIL] %s: Max diff = %g\n', currentName, maxDiff);
            % Optional: Show the differing matrix
            % disp(B_curr); 
            mismatchCount = mismatchCount + 1;
        else
            % Optional: Print nothing for success to keep console clean
            % fprintf('[OK] %s\n', currentName);
        end
        
    catch ME
        fprintf(2, '[ERROR] Could not read %s: %s\n', currentName, ME.message);
        mismatchCount = mismatchCount + 1;
    end
end

% 4. Final Summary
fprintf('\n--- Summary ---\n');
if mismatchCount == 0
    fprintf('✅ SUCCESS: All %d files have IDENTICAL Nominal B Matrices.\n', numFiles);
    fprintf('Conclusion: Your data generation is correct.\n');
else
    fprintf('❌ FAILURE: Found %d files with different B Matrices.\n', mismatchCount);
    fprintf('Conclusion: Your generation script is saving the PERTURBED matrix as the Nominal matrix.\n');
end