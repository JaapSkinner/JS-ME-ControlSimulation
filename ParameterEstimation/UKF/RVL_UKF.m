%% BATCH SIMULATION RUNNER
% Executes a batch of simulations using the pre-generated parameter sets.
% Ensure 'GenerateParameterSets.m' has been run first.

clear; clc;

% Configuration
NUMLOOPS = 50; % Must match NUM_SAMPLES in GenerateParameterSets.m

fprintf('Starting Batch Run of %d Simulations...\n', NUMLOOPS);
fprintf('--------------------------------------------------\n');

startTime = tic;

for i = 1:NUMLOOPS
    % 1. Construct the command string for evalc
    % We pass the index 'i' to the function
    runCmd = sprintf('runSimulationVariedFcnUKF(%d);', i);
    
    % 2. Run safely with output suppression
    try
        warningState = warning('off', 'all');
        
        % evalc captures all command window text (fprintf, disp) into 'T'
        % effectively silencing the function execution.
        [T] = evalc(runCmd); 
        
        warning(warningState); % Restore warnings
        
        % 3. Print Progress
        elapsedTime = toc(startTime);
        avgTimePerRun = elapsedTime / i;
        estTimeLeft = avgTimePerRun * (NUMLOOPS - i);
        
        fprintf('... Progress: %3.0f%% (%02d / %d) | ETA: %.1f min | Status: OK\n', ...
            (i/NUMLOOPS)*100, i, NUMLOOPS, estTimeLeft/60);
            
    catch ME
        warning(warningState); % Restore warnings
        fprintf(2, '\n[ERROR] Simulation %d Failed: %s\n', i, ME.message);
        fprintf(2, 'Continuing to next sample...\n');
    end
end

fprintf('--------------------------------------------------\n');
fprintf('Batch Run Complete in %.2f minutes.\n', toc(startTime)/60);