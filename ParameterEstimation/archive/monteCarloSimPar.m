% --- run this in your main MATLAB session ---
job = batch(@runMonteCarloChunks, 0, {}, ...
    'AttachedFiles', {'ParameterEstimationBaseOL.m', 'sampleParameters.m'}, ...
    'CurrentFolder', pwd);

% you can check progress
diary(job); % see job output
wait(job);  % block until it finishes
delete(job);
