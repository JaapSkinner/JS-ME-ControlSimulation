%% ULog + Mocap CSV Merger & Sync (Final Version)
% 1. Reads PX4 ULog (Vector Aware for Actuators/IMU)
% 2. Reads Mocap CSV
% 3. Auto-Syncs using Cross-Correlation on Z-Height
% 4. Reports Sync Confidence Score (detects "Drop" quality)
clear; clc;

%% 1. CONFIGURATION
% Manual Offset: If Auto-Sync fails, set this to shift Mocap in seconds.
% Positive = Shifts Mocap "Right" (Mocap started earlier than Log)
MANUAL_OFFSET = 0.0; 
ENABLE_AUTO_SYNC = true;

%% 2. FILE SELECTION
if ispc; homeDir = getenv('USERPROFILE'); else; homeDir = getenv('HOME'); end
try; proj = currentProject; projRoot = proj.RootFolder; catch; projRoot = pwd; end

% A. Select ULog
fprintf('1. Select ULog file...\n');
[ulogName, ulogPath] = uigetfile({'*.ulg', 'PX4 ULog'}, 'Select Flight Log', homeDir);
if isequal(ulogName, 0); return; end

% B. Select Mocap CSV
fprintf('2. Select Mocap CSV file...\n');
[csvName, csvPath] = uigetfile({'*.csv', 'Mocap Data'}, 'Select Mocap CSV', ulogPath);
if isequal(csvName, 0); disp('No CSV selected. Proceeding without Mocap.'); hasMocap=false; else; hasMocap=true; end

% C. Save Path
fprintf('3. Select Save Folder...\n');
savePath = uigetdir(projRoot, 'Select Save Folder');
if isequal(savePath, 0); savePath = projRoot; end
saveFileName = fullfile(savePath, [ulogName(1:end-4) '_synced.mat']);

%% 3. LOAD ULOG (Standard)
fprintf('\n--- PROCESSING ULOG ---\n');
ulog = ulogreader(fullfile(ulogPath, ulogName));
readTT = @(name) safeReadTT(ulog, name);

raw_imu   = readTT('sensor_combined');
raw_out   = readTT('actuator_outputs');
raw_ctrl  = readTT('actuator_motors'); 
raw_pos   = readTT('vehicle_local_position');
raw_esc   = readTT('esc_status'); 

% Master Time Grid (250Hz based on IMU)
if isempty(raw_imu); error('CRITICAL: sensor_combined missing.'); end
t_source = seconds(raw_imu.timestamp);
t_start = t_source(1);
t_master = (t_start : 0.004 : t_source(end))'; 
flightData.time = t_master - t_start; 

% Extract Log Data (Vector Aware)
flightData.imu.gyro  = extractVector(raw_imu, 'gyro_rad', [1 2 3], t_master);
flightData.imu.accel = extractVector(raw_imu, 'accelerometer_m_s2', [1 2 3], t_master);

act_data = extractVector(raw_out, 'output', [1 2 3 4], t_master);
if max(abs(act_data(:)))==0 && ~isempty(raw_ctrl)
    act_data = extractVector(raw_ctrl, 'control', [1 2 3 4], t_master);
end
flightData.actuators.cmd = act_data;
flightData.actuators.rpm = extractVector(raw_esc, 'esc_rpm', [1 2 3 4], t_master);
flightData.px4_est.pos   = extractScalars(raw_pos, {'x','y','z'}, t_master);

%% 4. LOAD & SYNC MOCAP CSV
if hasMocap
    fprintf('\n--- PROCESSING MOCAP CSV ---\n');
    % Load CSV
    opts = detectImportOptions(fullfile(csvPath, csvName));
    opts.VariableNamingRule = 'preserve'; 
    raw_csv = readtable(fullfile(csvPath, csvName), opts);
    
    % --- MAP COLUMNS ---
    col_t = findColumn(raw_csv, {'Time', 'time', 'Seconds', 't'});
    col_x = findColumn(raw_csv, {'x', 'X', 'Rigid Body 1 X', 'pose.position.x'});
    col_y = findColumn(raw_csv, {'y', 'Y', 'Rigid Body 1 Y', 'pose.position.y'});
    col_z = findColumn(raw_csv, {'z', 'Z', 'Rigid Body 1 Z', 'pose.position.z'});
    
    if isempty(col_t) || isempty(col_x)
        warning('Could not identify Time/Position columns in CSV. Skipping Mocap.');
        flightData.mocap.pos = zeros(length(t_master), 3);
    else
        % Extract Raw Mocap Data
        m_t = raw_csv.(col_t);
        m_pos = [raw_csv.(col_x), raw_csv.(col_y), raw_csv.(col_z)];
        
        % --- AUTO-SYNC LOGIC (With Confidence Score) ---
        sync_offset = MANUAL_OFFSET;
        
        if ENABLE_AUTO_SYNC && ~isempty(raw_pos)
            fprintf('Attempting Auto-Sync using Vertical Motion (Z)...\n');
            
            % 1. Prepare Signals
            log_z = flightData.px4_est.pos(:,3);
            m_t_rel = m_t - m_t(1);
            m_z_interp = interp1(m_t_rel, m_pos(:,3), flightData.time, 'linear', 'extrap');
            
            % 2. Normalize (for correlation coefficient)
            log_z_norm = (log_z - mean(log_z)) / std(log_z);
            m_z_norm   = (m_z_interp - mean(m_z_interp)) / std(m_z_interp);
            
            log_z_norm(isnan(log_z_norm)) = 0;
            m_z_norm(isnan(m_z_norm)) = 0;
            
            % 3. Cross Correlate
            max_lag_sec = 20; % Search window +/- 20s
            max_samples = round(max_lag_sec / 0.004);
            [xc, lags] = xcorr(log_z_norm, m_z_norm, max_samples, 'coeff'); 
            
            % 4. Find Peak (Absolute max handles inverted Z-axis)
            [score, idx] = max(abs(xc)); 
            lag_samples = lags(idx);
            lag_time = lag_samples * 0.004;
            
            % Check if correlation was negative (inverted axis)
            if xc(idx) < 0
                fprintf('  > Note: Mocap Z-axis appears inverted relative to Log Z.\n');
            end
            
            % 5. Report Confidence
            fprintf('  > Detected Lag: %.3f seconds\n', lag_time);
            fprintf('  > Sync Confidence: %.2f%%\n', score * 100);
            
            if score > 0.8
                fprintf('    [PASS] Excellent sync lock.\n');
            elseif score > 0.5
                fprintf('    [WARN] Moderate sync quality. Check visual plot.\n');
            else
                fprintf('    [FAIL] Poor sync (<50%%). "Drop" marker may be missing.\n');
            end
            
            % Apply detected lag
            sync_offset = sync_offset + lag_time;
        end
        
        fprintf('  > Applying Final Time Offset: %.3f s\n', sync_offset);
        
        % --- RESAMPLE ONTO MASTER TIME ---
        t_mocap_base = m_t - m_t(1); 
        
        flightData.mocap.pos = zeros(length(t_master), 3);
        flightData.mocap.pos(:,1) = interp1(t_mocap_base + sync_offset, m_pos(:,1), flightData.time, 'linear', 'extrap');
        flightData.mocap.pos(:,2) = interp1(t_mocap_base + sync_offset, m_pos(:,2), flightData.time, 'linear', 'extrap');
        flightData.mocap.pos(:,3) = interp1(t_mocap_base + sync_offset, m_pos(:,3), flightData.time, 'linear', 'extrap');
    end
else
    flightData.mocap.pos = zeros(length(t_master), 3);
end

%% 5. SAVE
fprintf('\nSaving to: %s ...\n', saveFileName);
save(saveFileName, 'flightData');
fprintf('Done.\n');


%% FUNCTIONS
function name = findColumn(tbl, candidates)
    name = '';
    cols = tbl.Properties.VariableNames;
    for i=1:length(candidates)
        match = find(strcmpi(cols, candidates{i}), 1);
        if ~isempty(match)
            name = cols{match};
            return;
        end
        match = find(contains(lower(cols), lower(candidates{i})), 1);
        if ~isempty(match); name = cols{match}; return; end
    end
end

function tt = safeReadTT(ulog, topicName)
    tt = timetable();
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
    out = zeros(length(t_query), length(indices));
    if isempty(tt); return; end
    if ~ismember(varName, tt.Properties.VariableNames); return; end
    rawArray = tt.(varName);
    t_src = seconds(tt.timestamp);
    [t_unique, idx] = unique(t_src);
    rawArray = rawArray(idx, :); 
    if size(rawArray, 2) < max(indices); return; end
    for k = 1:length(indices)
        colID = indices(k);
        vals = double(rawArray(:, colID));
        valid = ~isnan(vals);
        if sum(valid) > 2
             out(:, k) = interp1(t_unique(valid), vals(valid), t_query, 'linear', 'extrap');
        end
    end
end

function out = extractScalars(tt, varNames, t_query)
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