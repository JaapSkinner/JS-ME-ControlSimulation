%% ULog + Vicon CSV Merger & Sync (Octocopter Fixed)
% 1. Reads PX4 ULog (Vector Aware)
% 2. Reads Vicon CSV (NEU -> NED with Inverted Y, Pitch, and Yaw)
% 3. Auto-Syncs using Cross-Correlation on Z-Height
% 4. TRIMS data to start just before flight
clear; clc;

%% 1. CONFIGURATION
% --- Vehicle Config ---
NUM_MOTORS = 8;         % <--- UPDATED FOR OCTOCOPTER

% --- Sync Settings ---
MANUAL_OFFSET = 0.0; 
ENABLE_AUTO_SYNC = true;

% --- Vicon Settings ---
VICON_RATE = 100;       % Hz
VICON_HEADER_LINES = 5; % Junk lines before header

% --- Trimming Settings ---
TRIM_DATA = true;       
TRIM_BUFFER = 10.0;     % Seconds to keep BEFORE Arming

%% 2. FILE SELECTION
if ispc; homeDir = getenv('USERPROFILE'); else; homeDir = getenv('HOME'); end
try; proj = currentProject; projRoot = proj.RootFolder; catch; projRoot = pwd; end

% A. Select ULog
fprintf('1. Select ULog file...\n');
[ulogName, ulogPath] = uigetfile({'*.ulg', 'PX4 ULog'}, 'Select Flight Log', projRoot);
if isequal(ulogName, 0); return; end

% B. Select Mocap CSV
fprintf('2. Select Vicon CSV file...\n');
[csvName, csvPath] = uigetfile({'*.csv', 'Vicon Data'}, 'Select Vicon CSV', ulogPath);
if isequal(csvName, 0); 
    disp('No CSV selected. Proceeding without Mocap.'); 
    hasMocap=false; 
else 
    hasMocap=true; 
end

% C. Save Path
fprintf('3. Select Save Folder...\n');
savePath = uigetdir(projRoot, 'Select Save Folder');
if isequal(savePath, 0); savePath = projRoot; end
saveFileName = fullfile(savePath, [ulogName(1:end-4) '_synced.mat']);

%% 3. LOAD ULOG
fprintf('\n--- PROCESSING ULOG ---\n');
ulog = ulogreader(fullfile(ulogPath, ulogName));
readTT = @(name) safeReadTT(ulog, name);

% Original Topics
raw_imu   = readTT('sensor_combined');
raw_out   = readTT('actuator_outputs');
raw_ctrl  = readTT('actuator_motors'); 
raw_pos   = readTT('vehicle_local_position');
raw_esc   = readTT('esc_status'); 

% --- NEW TOPICS ---
raw_att     = readTT('vehicle_attitude');           
raw_ang_vel = readTT('vehicle_angular_velocity'); 

if isempty(raw_imu); error('CRITICAL: sensor_combined missing.'); end

% Master Time Grid (250Hz)
t_source = seconds(raw_imu.timestamp);
t_start_log = t_source(1);
t_master = (t_start_log : 0.004 : t_source(end))'; 
flightData.time = t_master - t_start_log; 

% Extract Log Data (IMU)
flightData.imu.gyro  = extractVector(raw_imu, 'gyro_rad', [1 2 3], t_master);
flightData.imu.accel = extractVector(raw_imu, 'accelerometer_m_s2', [1 2 3], t_master);

% --- ACTUATORS (UPDATED PRIORITY) ---
act_indices = 1:NUM_MOTORS;

fprintf('Extracting Control Inputs (actuator_motors)...\n');
% 1. Try 'actuator_motors.control' (Normalized 0-1)
act_data = extractVector(raw_ctrl, 'control', act_indices, t_master);

% 2. Fallback to 'actuator_outputs' if control is empty
if (isempty(act_data) || max(abs(act_data(:))) == 0) && ~isempty(raw_out)
    warning('actuator_motors.control empty. Falling back to actuator_outputs.');
    act_data = extractVector(raw_out, 'output', act_indices, t_master);
    
    % Check if fallback data is PWM (roughly > 800) and normalize if so
    valid_mask = act_data > 10; % simple filter for non-zero checks
    if any(valid_mask(:)) && mean(act_data(valid_mask)) > 800
        fprintf('  > Fallback data is PWM. Normalizing (1000-2000 -> 0-1)...\n');
        act_data = (act_data - 1000) / 1000;
        act_data(act_data < 0) = 0;
        act_data(act_data > 1) = 1;
    end
end
flightData.actuators.cmd = act_data;

% --- RPM EXTRACTION ---
fprintf('Extracting RPM for %d Motors...\n', NUM_MOTORS);
flightData.actuators.rpm = extractRPM(raw_esc, t_master, NUM_MOTORS);

% --- PX4 ESTIMATES (UPDATED) ---
fprintf('Extracting PX4 Estimates...\n');

% 1. Position (x,y,z)
flightData.px4_est.pos = extractScalars(raw_pos, {'x','y','z'}, t_master);

% 2. Velocity & Acceleration (Inertial NED Frame)
flightData.px4_est.vel = extractScalars(raw_pos, {'vx','vy','vz'}, t_master);
flightData.px4_est.acc = extractScalars(raw_pos, {'ax','ay','az'}, t_master);

% 3. Attitude Quaternions (q)
flightData.px4_est.q = extractVector(raw_att, 'q', [1 2 3 4], t_master);

% 4. Angular Velocity & Acceleration
flightData.px4_est.ang_vel   = extractVector(raw_ang_vel, 'xyz', [1 2 3], t_master);
flightData.px4_est.ang_accel = extractVector(raw_ang_vel, 'xyz_derivative', [1 2 3], t_master);

% 5. Calculated Euler (Convenience)
if any(flightData.px4_est.q(:))
    [y, p, r] = quat2angle(flightData.px4_est.q);
    flightData.px4_est.rpy = [r, p, y];
else
    flightData.px4_est.rpy = zeros(length(t_master), 3);
end

% 6. CALCULATE BODY FRAME VECTORS (NEW)
%    PX4 'vel' and 'acc' are in NED (Inertial). 
%    We rotate them INTO the Body Frame using the inverse of the attitude quaternion.
if any(flightData.px4_est.q(:))
    fprintf('Calculating Body Frame Velocities...\n');
    flightData.px4_est.vel_body = rotate_vector_inv(flightData.px4_est.vel, flightData.px4_est.q);
    flightData.px4_est.acc_body = rotate_vector_inv(flightData.px4_est.acc, flightData.px4_est.q);
end

%% 4. LOAD & PARSE VICON CSV (NEU -> NED)
if hasMocap
    fprintf('\n--- PROCESSING VICON CSV ---\n');
    
    % --- READ CSV ---
    opts = detectImportOptions(fullfile(csvPath, csvName));
    opts.DataLines = [VICON_HEADER_LINES + 1, Inf]; 
    opts.VariableNamingRule = 'preserve';
    
    try
        raw_table = readtable(fullfile(csvPath, csvName), opts);
    catch
        raw_table = table(); 
    end
    
    % --- IDENTIFY COLUMNS ---
    col_frame = findColumn(raw_table, {'Frame', 'frame'});
    col_tx    = findColumn(raw_table, {'TX', 'tx', 'Tx', 'Translation X'});
    col_rx    = findColumn(raw_table, {'RX', 'rx', 'Rx', 'Rotation X'});
    col_rw    = findColumn(raw_table, {'RW', 'rw', 'Rw', 'Rotation W'}); 
    
    v_frame = []; v_pos_neu = []; v_quat_neu = [];
    
    % Logic: Read Data
    if ~isempty(col_frame) && ~isempty(col_tx)
        % Name Based
        v_frame = forceNumeric(raw_table.(col_frame));
        v_pos_neu = [forceNumeric(raw_table.(col_tx)), ...
                     forceNumeric(raw_table.(findColumn(raw_table, {'TY','ty'}))), ...
                     forceNumeric(raw_table.(findColumn(raw_table, {'TZ','tz'})))];
        
        % Try Reading Quaternions
        if ~isempty(col_rx)
             ry = forceNumeric(raw_table.(findColumn(raw_table, {'RY','ry'})));
             rz = forceNumeric(raw_table.(findColumn(raw_table, {'RZ','rz'})));
             rx = forceNumeric(raw_table.(col_rx));
             if ~isempty(col_rw); rw = forceNumeric(raw_table.(col_rw)); else; rw = ones(size(rx)); end
             v_quat_neu = [rw, rx, ry, rz]; 
        end
    else
        % Index Fallback
        try
            raw_mat = readmatrix(fullfile(csvPath, csvName), 'NumHeaderLines', VICON_HEADER_LINES);
            if size(raw_mat, 2) >= 9
                v_frame = raw_mat(:, 1);
                v_quat_neu = [raw_mat(:, 6), raw_mat(:, 3), raw_mat(:, 4), raw_mat(:, 5)]; 
                v_pos_neu  = raw_mat(:, 7:9);
            else
                error('Vicon CSV has too few columns for Index Fallback.');
            end
        catch ME
             error('Mocap load failed: %s', ME.message);
        end
    end
    
    if isempty(v_frame); error('Mocap extracted empty data.'); end
    
    % 1. Clean NaNs
    valid = ~isnan(v_frame) & ~isnan(v_pos_neu(:,1));
    v_frame = v_frame(valid);
    v_pos_neu = v_pos_neu(valid, :) / 1000.0; % mm -> m
    if ~isempty(v_quat_neu); v_quat_neu = v_quat_neu(valid, :); end
    
    % 2. COORDINATE TRANSFORM (NEU -> NED)
    % Position
    v_pos_ned = v_pos_neu;
    v_pos_ned(:, 2) = -v_pos_neu(:, 2); % Invert Y
    v_pos_ned(:, 3) = -v_pos_neu(:, 3); % Invert Z
    
    % Attitude
    if ~isempty(v_quat_neu)
        [r_neu, p_neu, y_neu] = q_to_euler(v_quat_neu);
        
        % --- UPDATED TRANSFORMATION ---
        r_ned = r_neu;    
        p_ned = -p_neu;  % <--- PITCH INVERTED (User Request)
        y_ned = -y_neu;  % Yaw Inverted
        
        v_rpy_ned = [r_ned, p_ned, y_ned];
        v_quat_ned = euler_to_q(r_ned, p_ned, y_ned);
    else
        v_rpy_ned = zeros(length(v_frame), 3);
        v_quat_ned = repmat([1 0 0 0], length(v_frame), 1);
    end
    
    v_t = (v_frame - v_frame(1)) / VICON_RATE;
    
    % 3. AUTO-SYNC
    sync_offset = MANUAL_OFFSET;
    if ENABLE_AUTO_SYNC && ~isempty(flightData.px4_est.pos)
        fprintf('Attempting Auto-Sync...\n');
        log_z = flightData.px4_est.pos(:,3);
        v_z_interp = interp1(v_t, v_pos_ned(:,3), flightData.time, 'linear', 'extrap');
        
        log_z_norm = (log_z - mean(log_z)) / std(log_z);
        v_z_norm   = (v_z_interp - mean(v_z_interp)) / std(v_z_interp);
        log_z_norm(isnan(log_z_norm)) = 0; v_z_norm(isnan(v_z_norm)) = 0;
        
        [xc, lags] = xcorr(log_z_norm, v_z_norm, round(60/0.004), 'coeff');
        [score, idx] = max(abs(xc));
        
        if score > 0.3
            sync_offset = sync_offset + lags(idx) * 0.004;
            fprintf('  > Sync Lag: %.3fs (Conf: %.1f%%)\n', lags(idx)*0.004, score*100);
        else
            warning('Sync failed (Low Confidence). Using 0 offset.');
        end
    end
    
    % 4. RESAMPLE
    t_query = flightData.time - sync_offset;
    flightData.mocap.pos = interp1(v_t, v_pos_ned, t_query, 'linear', 'extrap');
    flightData.mocap.rpy = interp1(v_t, unwrap(v_rpy_ned), t_query, 'linear', 'extrap');
    flightData.mocap.q   = euler_to_q(flightData.mocap.rpy(:,1), flightData.mocap.rpy(:,2), flightData.mocap.rpy(:,3));
else
    flightData.mocap.pos = zeros(length(t_master),3);
    flightData.mocap.rpy = zeros(length(t_master),3);
    flightData.mocap.q   = zeros(length(t_master),4);
end

%% 5. TRIM DATA
if TRIM_DATA
    fprintf('\n--- TRIMMING DATA ---\n');
    cmds = flightData.actuators.cmd;
    % Simple Arm Detection (Threshold > 0.05 on any motor)
    is_armed = sum(abs(cmds), 2) > 0.05; 
    
    if any(is_armed)
        idx_arm = find(is_armed, 1, 'first');
        t_arm = flightData.time(idx_arm);
        t_new_start = max(0, t_arm - TRIM_BUFFER);
        idx_start = find(flightData.time >= t_new_start, 1, 'first');
        
        fprintf('  > Trimming to %.1fs (keeping %.1fs buffer).\n', t_new_start, TRIM_BUFFER);
        flightData.time = flightData.time(idx_start:end) - flightData.time(idx_start);
        
        fields = fieldnames(flightData);
        for i = 1:numel(fields)
            f = fields{i};
            if isstruct(flightData.(f))
                sf = fieldnames(flightData.(f));
                for j = 1:numel(sf)
                    dat = flightData.(f).(sf{j});
                    if size(dat, 1) >= idx_start
                         flightData.(f).(sf{j}) = dat(idx_start:end, :);
                    end
                end
            end
        end
    end
end

%% 6. SAVE
fprintf('\nSaving to: %s ...\n', saveFileName);
save(saveFileName, 'flightData');
fprintf('Done.\n');

%% HELPERS
function vals = forceNumeric(col)
    if iscell(col); vals = str2double(col); else; vals = col; end
end

function name = findColumn(tbl, candidates)
    name = ''; cols = tbl.Properties.VariableNames;
    for i=1:length(candidates)
        match = find(strcmpi(cols, candidates{i}), 1);
        if ~isempty(match); name = cols{match}; return; end
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

function out = extractRPM(tt, t_query, n_motors)
    out = zeros(length(t_query), n_motors); 
    if isempty(tt); return; end
    cols = tt.Properties.VariableNames;
    
    matrix_candidates = {'esc_rpm', 'rpm', 'esc_report_rpm'};
    for i=1:length(matrix_candidates)
        if ismember(matrix_candidates{i}, cols)
            raw = tt.(matrix_candidates{i});
            if size(raw, 2) >= n_motors
                t_src = seconds(tt.timestamp); [t_u, idx] = unique(t_src);
                raw = double(raw(idx, :));
                for m=1:n_motors
                    out(:, m) = interp1(t_u, raw(:,m), t_query, 'linear', 'extrap'); 
                end
                return;
            end
        end
    end
    
    t_src = seconds(tt.timestamp); [t_u, idx] = unique(t_src);
    for i = 0:(n_motors-1)
        pat1 = sprintf('esc[%d].esc_rpm', i); 
        pat2 = sprintf('esc_%d_esc_rpm', i);
        col = ''; if ismember(pat1, cols); col=pat1; elseif ismember(pat2, cols); col=pat2; end
        
        if ~isempty(col)
            vals = double(tt.(col)(idx)); valid = ~isnan(vals);
            if sum(valid)>2
                out(:, i+1) = interp1(t_u(valid), vals(valid), t_query, 'linear', 'extrap'); 
            end
        end
    end
end

function out = extractVector(tt, varName, indices, t_query)
    out = zeros(length(t_query), length(indices));
    if isempty(tt); return; end
    
    if ~ismember(varName, tt.Properties.VariableNames); return; end
    rawArray = tt.(varName);
    
    t_src = seconds(tt.timestamp); [t_unique, idx] = unique(t_src);
    rawArray = rawArray(idx, :); 
    
    if size(rawArray, 2) < max(indices); return; end
    
    for k = 1:length(indices)
        vals = double(rawArray(:, indices(k))); valid = ~isnan(vals);
        if sum(valid) > 2
             out(:, k) = interp1(t_unique(valid), vals(valid), t_query, 'linear', 'extrap');
        end
    end
end

function out = extractScalars(tt, varNames, t_query)
    out = zeros(length(t_query), length(varNames));
    if isempty(tt); return; end
    t_src = seconds(tt.timestamp); [t_unique, idx] = unique(t_src);
    for k = 1:length(varNames)
        vn = varNames{k};
        if ismember(vn, tt.Properties.VariableNames)
            vals = double(tt.(vn)(idx)); valid = ~isnan(vals);
            if sum(valid) > 2
                out(:, k) = interp1(t_unique(valid), vals(valid), t_query, 'linear', 'extrap');
            end
        end
    end
end

function [r, p, y] = q_to_euler(q)
    w = q(:,1); x = q(:,2); y = q(:,3); z = q(:,4);
    r = atan2(2*(w.*x + y.*z), 1 - 2*(x.^2 + y.^2));
    p = asin(max(-1, min(1, 2*(w.*y - z.*x))));
    y = atan2(2*(w.*z + x.*y), 1 - 2*(y.^2 + z.^2));
end

function q = euler_to_q(r, p, y)
    c1=cos(y/2); s1=sin(y/2); c2=cos(p/2); s2=sin(p/2); c3=cos(r/2); s3=sin(r/2);
    q = [c1.*c2.*c3 + s1.*s2.*s3, c1.*c2.*s3 - s1.*s2.*c3, ...
         c1.*s2.*c3 + s1.*c2.*s3, s1.*c2.*c3 - c1.*s2.*s3];
end

function v_body = rotate_vector_inv(v_inertial, q)
    q_inv = [q(:,1), -q(:,2), -q(:,3), -q(:,4)];
    w = q_inv(:,1);
    xyz = q_inv(:,2:4);
    t = 2 * cross(xyz, v_inertial, 2);
    v_body = v_inertial + (w .* t) + cross(xyz, t, 2);
end