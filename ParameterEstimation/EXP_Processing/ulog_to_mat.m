%% Vector-Aware ULog Parser (Final Fix)
% - Handles Array Columns (gyro_rad, output) directly.
% - Fills missing topics (RPM, Mocap) with Zeros.
% - Extracts actuator_outputs (PWM) or actuator_motors (Norm)
clear; clc;

%% 1. FILE SELECTION
if ispc; homeDir = getenv('USERPROFILE'); else; homeDir = getenv('HOME'); end
try; proj = currentProject; projRoot = proj.RootFolder; catch; projRoot = pwd; end

fprintf('Select ULog file...\n');
[fileName, pathName] = uigetfile({'*.ulg', 'PX4 ULog'}, 'Select Log', homeDir);
if isequal(fileName, 0); return; end

fprintf('Select Save Folder...\n');
savePath = uigetdir(projRoot, 'Select Save Folder');
if isequal(savePath, 0); savePath = projRoot; end
saveFileName = fullfile(savePath, [fileName(1:end-4) '_processed.mat']);

%% 2. LOAD
fprintf('------------------------------------------------\n');
fprintf('Loading: %s\n', fileName);
ulog = ulogreader(fullfile(pathName, fileName));

% Helper to read full timetable (including array columns)
readTT = @(name) safeReadTT(ulog, name);

% Load raw tables
fprintf('Reading Tables...\n');
raw_imu   = readTT('sensor_combined');
raw_out   = readTT('actuator_outputs'); % Primary actuators
raw_ctrl  = readTT('actuator_motors');  % Secondary (Normalized)
raw_pos   = readTT('vehicle_local_position');
raw_esc   = readTT('esc_status');       % Missing in your log
raw_mocap = readTT('vehicle_visual_odometry'); % Missing in your log

%% 3. MASTER TIME SYNC
if isempty(raw_imu)
    error('CRITICAL: sensor_combined missing. Cannot sync time.');
end

% Create 250Hz Master Grid
t_source = seconds(raw_imu.timestamp);
t_start  = t_source(1);
t_end    = t_source(end);
t_master = (t_start : 0.004 : t_end)'; % Column vector

flightData.time = t_master - t_start; % Relative time starting at 0
flightData.sourceLog = fileName;

%% 4. DATA EXTRACTION (Vector Aware)

fprintf('Processing IMU (Vectors)...\n');
% Extract columns 1,2,3 from the 'gyro_rad' array
flightData.imu.gyro = extractVector(raw_imu, 'gyro_rad', [1 2 3], t_master);
% Extract columns 1,2,3 from the 'accelerometer_m_s2' array
flightData.imu.accel = extractVector(raw_imu, 'accelerometer_m_s2', [1 2 3], t_master);

fprintf('Processing Actuators (Vectors)...\n');
% Prefer 'output' from actuator_outputs (The inspection showed this exists)
act_data = extractVector(raw_out, 'output', [1 2 3 4], t_master);

% Fallback: If 'output' is empty/zeros, try 'control' from actuator_motors
if max(abs(act_data(:))) == 0 && ~isempty(raw_ctrl)
    fprintf('  > actuator_outputs empty/low. Switching to actuator_motors (control)...\n');
    act_data = extractVector(raw_ctrl, 'control', [1 2 3 4], t_master);
end
flightData.actuators.cmd = act_data;

fprintf('Processing Position (Scalars)...\n');
% Position uses scalar columns 'x', 'y', 'z'
flightData.px4_est.pos = extractScalars(raw_pos, {'x', 'y', 'z'}, t_master);
flightData.px4_est.vel = extractScalars(raw_pos, {'vx', 'vy', 'vz'}, t_master);

fprintf('Processing RPM (Missing -> Zeros)...\n');
flightData.actuators.rpm = extractVector(raw_esc, 'esc_rpm', [1 2 3 4], t_master);

fprintf('Processing Mocap (Missing -> Zeros)...\n');
% Try vector 'position' first, then scalar 'x','y','z'
mocap_vec = extractVector(raw_mocap, 'position', [1 2 3], t_master);
if max(abs(mocap_vec(:))) == 0
    mocap_vec = extractScalars(raw_mocap, {'x', 'y', 'z'}, t_master);
end
flightData.mocap.pos = mocap_vec;

%% 5. SAVE
fprintf('Saving to: %s\n', saveFileName);
save(saveFileName, 'flightData');
fprintf('Done.\n');


%% --- HELPER FUNCTIONS ---

function tt = safeReadTT(ulog, topicName)
    tt = timetable();
    % Check if topic exists in the list
    if ~any(strcmp(ulog.AvailableTopics.TopicNames, topicName))
        warning('Topic missing: %s', topicName);
        return;
    end
    try
        ds = readTopicMsgs(ulog, 'TopicNames', {topicName});
        tt = ds.TopicMessages{1};
    catch ME
        warning('Error reading %s: %s', topicName, ME.message);
    end
end

function out = extractVector(tt, varName, indices, t_query)
    % Extracts columns from an ARRAY variable (e.g. gyro_rad(:, 1:3))
    out = zeros(length(t_query), length(indices));
    
    if isempty(tt); return; end
    if ~ismember(varName, tt.Properties.VariableNames); return; end
    
    % Get Data and Time
    rawArray = tt.(varName); % This is likely Nx3 or Nx16
    t_src = seconds(tt.timestamp);
    
    % Handle Duplicates
    [t_unique, idx] = unique(t_src);
    rawArray = rawArray(idx, :); 
    
    % Check if array is wide enough
    if size(rawArray, 2) < max(indices)
        warning('Variable %s has width %d, but requested index %d.', ...
            varName, size(rawArray, 2), max(indices));
        return;
    end
    
    % Interpolate each requested column
    for k = 1:length(indices)
        colID = indices(k);
        vals = double(rawArray(:, colID));
        % Remove NaNs if any (actuator_motors had NaNs)
        valid = ~isnan(vals);
        if sum(valid) > 2
             out(:, k) = interp1(t_unique(valid), vals(valid), t_query, 'linear', 'extrap');
        end
    end
end

function out = extractScalars(tt, varNames, t_query)
    % Extracts separate scalar variables (e.g. 'x', 'y', 'z')
    out = zeros(length(t_query), length(varNames));
    
    if isempty(tt); return; end
    
    t_src = seconds(tt.timestamp);
    [t_unique, idx] = unique(t_src);
    
    for k = 1:length(varNames)
        vn = varNames{k};
        if ismember(vn, tt.Properties.VariableNames)
            vals = double(tt.(vn)(idx));
            valid = ~isnan(vals);
            if sum(valid) > 2
                out(:, k) = interp1(t_unique(valid), vals(valid), t_query, 'linear', 'extrap');
            end
        end
    end
end