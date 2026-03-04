function evaluate_batch_performance()
%% BATCH EVALUATION: Find Best RLS Improvement
% This script loops through a folder of "Validation_*.mat" files.
% For each file, it calculates the RMSE Improvement (Nominal vs. RLS).
% Finally, it prints a detailed report for the single BEST file found.

clearvars; clc;

%% 1. Select Folder
fprintf('Select the folder containing Validation Results (Validation_*.mat)...\n');
baseDir = fullfile(pwd, 'Results', 'Mixer_Validation_Batch');
if ~isfolder(baseDir), baseDir = pwd; end

folderPath = uigetdir(baseDir, 'Select Validation Results Folder');
if isequal(folderPath, 0), return; end

fileList = dir(fullfile(folderPath, 'Validation_*.mat'));
if isempty(fileList)
    error('No Validation_*.mat files found in: %s', folderPath);
end

fprintf('Found %d validation files. Processing...\n\n', length(fileList));

%% 2. Batch Processing Loop
bestFile.name = '';
bestFile.avgImprovement = -inf;
bestFile.metrics = [];

% Table Header for progress
fprintf('%-40s | %s\n', 'File Name', 'Avg Improvement');
fprintf('%s\n', repmat('-', 1, 60));

for k = 1:length(fileList)
    fileName = fileList(k).name;
    filePath = fullfile(folderPath, fileName);
    
    try
        % --- Process Single File ---
        metrics = process_single_file(filePath);
        
        % Calculate Score (Average Improvement across all 6 axes)
        avgImp = mean([metrics.improvement]);
        
        % Print summary line
        fprintf('%-40s | %6.2f%%\n', fileName, avgImp);
        
        % Check if this is the new champion
        if avgImp > bestFile.avgImprovement
            bestFile.name = fileName;
            bestFile.avgImprovement = avgImp;
            bestFile.metrics = metrics;
        end
        
    catch ME
        fprintf('%-40s | ERROR: %s\n', fileName, ME.message);
    end
end

%% 3. Print Final Report
if isempty(bestFile.name)
    fprintf('\nNo valid results processed.\n');
    return;
end

fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('WINNER: BEST IMPROVEMENT REPORT\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('File: %s\n', bestFile.name);
fprintf('Overall Average Improvement: %.2f%%\n\n', bestFile.avgImprovement);

fprintf('%-6s | %-10s | %-10s | %-12s\n', 'Axis', 'Nom RMSE', 'RLS RMSE', 'Improvement');
fprintf('%s\n', repmat('-', 1, 46));

axisNames = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
for i = 1:6
    m = bestFile.metrics(i);
    fprintf('%-6s | %10.4f | %10.4f | %10.1f%%\n', ...
        axisNames{i}, m.rmse_nom, m.rmse_rls, m.improvement);
end
fprintf('\n');

end

%% --- LOCAL WORKER FUNCTIONS ---

function metrics = process_single_file(filePath)
    % Loads file, extracts data, calculates RMSE for 6 axes
    
    data = load(filePath, 'simOut_Nominal', 'simOut_RLS', 'simIn', 'trajectory_timeseries');
    
    % 1. Extract Reference
    names_ref = {'trajectory_timeseries', 'Ref_Trajectory', 'Reference'};
    if isfield(data, 'trajectory_timeseries')
        ts_ref = data.trajectory_timeseries;
    else
        ts_ref = find_signal(data.simOut_Nominal, names_ref);
    end
    [t_ref, pos_ref] = sanitize_data(ts_ref, 1:3); 
    [~,     att_ref] = sanitize_data(ts_ref, 4:6);
    
    % 2. Extract Nominal & RLS
    names_states = {'UAV_State', 'States', 'x_state', 'Plant_States'};
    
    [t_nom, pos_nom, att_nom] = extract_states(data.simOut_Nominal, names_states);
    [t_rls, pos_rls, att_rls] = extract_states(data.simOut_RLS, names_states);
    
    % 3. Calculate Metrics (Overall)
    metrics = struct('axis', {}, 'rmse_nom', {}, 'rmse_rls', {}, 'improvement', {});
    
    for i = 1:6
        % Get Reference Data
        if i <= 3, ref = pos_ref(:,i); else, ref = att_ref(:,i-3); end
        
        % Get Nominal Data & Interpolate
        if i <= 3, dat_n = pos_nom(:,i); else, dat_n = att_nom(:,i-3); end
        nom_interp = interp1(t_nom, dat_n, t_ref, 'linear', 'extrap');
        
        % Get RLS Data & Interpolate
        if i <= 3, dat_r = pos_rls(:,i); else, dat_r = att_rls(:,i-3); end
        rls_interp = interp1(t_rls, dat_r, t_ref, 'linear', 'extrap');
        
        % Calc RMSE
        rmse_n = sqrt(mean((ref - nom_interp).^2));
        rmse_r = sqrt(mean((ref - rls_interp).^2));
        imp    = (rmse_n - rmse_r) / rmse_n * 100;
        
        metrics(i).axis = i;
        metrics(i).rmse_nom = rmse_n;
        metrics(i).rmse_rls = rmse_r;
        metrics(i).improvement = imp;
    end
end

function [t, pos, att] = extract_states(simOut, names)
    ts = find_signal(simOut, names);
    if isempty(ts), error('States not found'); end
    [t, d] = sanitize_data(ts);
    pos = d(:, 1:3);
    att = d(:, 7:9);
end

function [t, d] = sanitize_data(ts, indices)
    t = ts.Time;
    raw = squeeze(ts.Data);
    [R, C] = size(raw);
    if R ~= length(t) && C == length(t), raw = raw'; end
    if nargin > 1
        max_col = size(raw, 2);
        valid = indices(indices <= max_col);
        d = raw(:, valid);
    else
        d = raw;
    end
end

function ts = find_signal(simOut, signalNames)
    if ~iscell(signalNames), signalNames = {signalNames}; end
    ts = [];
    for k = 1:length(signalNames)
        name = signalNames{k};
        if isa(simOut, 'Simulink.SimulationOutput')
            if isprop(simOut, name)
                prop = simOut.get(name);
                if isa(prop,'timeseries'), ts=prop; return; end
                if isa(prop,'struct'), ts=timeseries(prop.signals.values,prop.time); return; end
            end
            dSets = {'logsout','yout'};
            for d=1:2
                if isprop(simOut, dSets{d})
                   ds = simOut.get(dSets{d});
                   if isa(ds,'Simulink.SimulationData.Dataset')
                       el = ds.get(name);
                       if ~isempty(el), ts=el.Values; return; end
                   end
                end
            end
        end
    end
end