%% Setup Replay Data (Data & Signal Processing)
% Loads processed flight data and prepares it for Simulink Replay.
% Optimized for the new 'ulog_to_mat' structure (Guaranteed Q, RPY, Body Vel).

%% 1. LOAD DATA
if ~exist('flightData', 'var')
    try; proj = currentProject; startPath = fullfile(proj.RootFolder,'ParameterEstimation'); catch; startPath = pwd; end
    [fName, pPath] = uigetfile(fullfile(startPath, '*.mat'), 'Select Processed Flight Data');
    if isequal(fName,0); error('No data selected.'); end
    load(fullfile(pPath, fName));
end

%% 2. TIME VECTOR
t = flightData.time;
sim_duration = t(end);
fprintf('Data Loaded. Duration: %.2fs\n', sim_duration);

%% 3. CREATE SIMULINK TIMESERIES

% --- A. ACTUATORS ---
% Normalized Motor Commands [0-1]
ts_u_cmd = timeseries(flightData.actuators.cmd, t, 'Name', 'U_Cmd');

% Motor RPM (Actual Speed)
if isfield(flightData.actuators, 'rpm')
    ts_rpm = timeseries(flightData.actuators.rpm, t, 'Name', 'Motor_RPM');
else
    % Zero fallback if missing (prevents block errors)
    warning('No RPM data found. Using zeros.');
    % Default to 8 motors (safe fallback for octo)
    ts_rpm = timeseries(zeros(size(t,1), 8), t, 'Name', 'Motor_RPM');
end

% --- B. SENSORS (RAW IMU) ---
% These are in the BODY frame
ts_omega = timeseries(flightData.imu.gyro, t, 'Name', 'Gyro_Body');
ts_accel = timeseries(flightData.imu.accel, t, 'Name', 'Accel_Body');

% --- C. MOCAP TRUTH ---
if isfield(flightData, 'mocap') && any(flightData.mocap.pos(:))
    ts_pos_truth = timeseries(flightData.mocap.pos, t, 'Name', 'Pos_Truth'); % NED
    ts_quat      = timeseries(flightData.mocap.q,   t, 'Name', 'Att_Quat');   % Attitude q
    ts_rpy       = timeseries(flightData.mocap.rpy, t, 'Name', 'Att_RPY');    % Euler Angles
    
    fprintf('Loaded: Mocap Truth (Pos, Quat, RPY).\n');
else
    warning('No Mocap Data found! Replay comparison will be empty.');
    % Create dummy ground truth to allow simulation to run
    ts_pos_truth = timeseries(zeros(length(t),3), t, 'Name', 'Pos_Truth');
    ts_quat      = timeseries(repmat([1 0 0 0], length(t),1), t, 'Name', 'Att_Quat');
    ts_rpy       = timeseries(zeros(length(t),3), t, 'Name', 'Att_RPY');
end

% --- D. ONBOARD ESTIMATES (NEW) ---
% We load these to compare against our RLS/UKF, or to use as state feedback.

% 1. Body Frame Velocity (Calculated in ulog_to_mat)
if isfield(flightData.px4_est, 'vel_body')
    ts_est_vel_body = timeseries(flightData.px4_est.vel_body, t, 'Name', 'Est_Vel_Body');
else
    ts_est_vel_body = timeseries(zeros(length(t),3), t, 'Name', 'Est_Vel_Body');
end

% 2. Body Frame Linear Acceleration (Calculated in ulog_to_mat)
if isfield(flightData.px4_est, 'acc_body')
    ts_est_acc_body = timeseries(flightData.px4_est.acc_body, t, 'Name', 'Est_Acc_Body');
else
    ts_est_acc_body = timeseries(zeros(length(t),3), t, 'Name', 'Est_Acc_Body');
end

% 3. Angular Acceleration (From vehicle_angular_velocity derivative)
if isfield(flightData.px4_est, 'ang_accel')
    ts_est_ang_accel = timeseries(flightData.px4_est.ang_accel, t, 'Name', 'Est_Ang_Accel');
else
    ts_est_ang_accel = timeseries(zeros(length(t),3), t, 'Name', 'Est_Ang_Accel');
end

% 4. Angular Velocity (NEW - Filtered/Estimated Body Rates)
if isfield(flightData.px4_est, 'ang_vel')
    ts_est_ang_vel = timeseries(flightData.px4_est.ang_vel, t, 'Name', 'Est_Ang_Vel');
else
    % Fallback to raw gyro if estimate is missing
    ts_est_ang_vel = ts_omega;
    ts_est_ang_vel.Name = 'Est_Ang_Vel_Fallback';
end

% 5. Attitude from Px4
if isfield(flightData.px4_est, 'rpy')
    ts_est_ang = timeseries(flightData.px4_est.rpy, t, 'Name','Est_Ang');
end

fprintf('Loaded: PX4 Body Estimates (Vel, Acc, AngVel, AngAccel).\n');

%% 4. TUSTIN DERIVATIVE FILTER SETUP
% Used for computing Angular Acceleration from Gyro in Simulink (if needed)
% H(s) = s / (tau*s + 1) -> Discretized via Tustin
replay_dt = 0.004;      % 250 Hz
fc_deriv  = 15;         % Cutoff Frequency (Hz)
tau_deriv = 1 / (2 * pi * fc_deriv);
denom_term = (2*tau_deriv + replay_dt);

% Discrete Transfer Fcn Coefficients [Num, Den]
diff_num = [2, -2] / denom_term;
diff_den = [1, (replay_dt - 2*tau_deriv) / denom_term];

fprintf('Setup Complete.\n');