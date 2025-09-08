paramNames = {
    'Motor.K_V';
    'Motor.K_E';
    'Motor.B';
    'Motor.Volt_offset';
    'Motor.volt_slope';
    'Motor.R';
    'Motor.I_0';
    'Uav.D_UAV';
    'Uav.D_PROP';
    'Uav.M';
    'Uav.I';
    'Uav.RHO_AIR';
    'Uav.R_PROP';
    'Uav.A_UAV';
    'Uav.A_PROP';
    'Uav.ZETA';
};
baseDir = fileparts(mfilename('fullpath'));
tmpDir = fullfile(baseDir, 'tmp_results');
nParams = numel(paramNames);
nSteps = 20;
range = linspace(0, 50, nSteps);  % percentage deviation from nominal
results = {};
meta = {};
clc;
if ~exist(tmpDir, 'dir')
    mkdir(tmpDir);
end
%%

for i = 1:nParams
    for j = 1:nSteps
        try
            fprintf('Sweep %d/%d\n', (i-1)*nSteps + j, nParams*nSteps);
            varianceList = zeros(nParams, 1);
            varianceList(i) = range(j);

            simOut = RunParameterEstimationSim(paramNames, varianceList);
            metrics = evaluateUAVPerformance(simOut);

            fname = fullfile(tmpDir, sprintf('result_%s_%02d.mat', strrep(paramNames{i}, '.', '_'), j));
            save(fname, 'metrics', 'varianceList');
            
            meta{end+1,1} = paramNames{i};
            meta{end,2} = range(j);
            meta{end,3} = fname;

            % fprintf('Sweep %d/%d - Param: %s, Variance: %.2f%%, CrossTrackRMS: %.4f, RotationErrorRMS: %.4f\n', ...
            % j, nSteps, paramNames{i}, varianceList(i), metrics.CrossTrackRMS, metrics.RotationErrorRMS);
        catch ME
            warning('Simulation failed for param %s at step %d: %s', paramNames{i}, j, ME.message);
            
            % Print all causes if any
            if ~isempty(ME.cause)
                for k = 1:numel(ME.cause)
                    fprintf('  Cause %d: %s\n', k, ME.cause{k}.message);
                end
            end
        
            % Print full stack
            fprintf('  Full error stack:\n');
            for k = 1:numel(ME.stack)
                fprintf('    %s (line %d)\n', ME.stack(k).file, ME.stack(k).line);
            end
        
            % Optional: rethrow or log more
        end
    end
    disp('Pausing for cleanup...');
    pause(2);
    drawnow
end

%% 

proj = matlab.project.currentProject;
projectRoot = proj.RootFolder;
tmpDir = fullfile(projectRoot, 'ParameterEstimation', 'tmp_results');
combinedResults = table();

files = dir(fullfile(tmpDir, 'result_*.mat'));

for i = 1:numel(files)
    fileName = fullfile(files(i).folder, files(i).name);
    data = load(fileName);

    if isfield(data, 'metrics') && isfield(data, 'varianceList')
        m = data.metrics;
        v = data.varianceList;

        newRow = table({v}, ...
            m.YDeviationRMS, m.RotationErrorRMS, ...
            m.ThrustErrorMean, m.TorqueErrorRMS, ...
            'VariableNames', {'VarianceList', 'YDeviationRMS', ...
            'RotationErrorRMS', 'ThrustErrorMean', 'TorqueErrorRMS'});

        combinedResults = [combinedResults; newRow];
    else
        warning('Missing required data in: %s', fileName);
    end

    % delete(fileName);
end
% 
% % Optional: delete temp dir
% if exist(tmpDir, 'dir')
%     rmdir(tmpDir, 's');
% end

% Save final result
save('ParameterEstimation/sweep_combined_results.mat', 'combinedResults');
disp('Sweep complete. Results saved to sweep_combined_results.mat');
