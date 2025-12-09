%% Setup Replay Data (Data & Signal Processing)
% Loads processed flight data and prepares it for Simulink Replay.
% Call this script from your main run function.

% 1. Load Data (If not already provided in workspace)
if ~exist('flightData', 'var')
    if ispc; homeDir = getenv('USERPROFILE'); else; homeDir = getenv('HOME'); end
    [fName, pPath] = uigetfile(fullfile(homeDir, '*.mat'), 'Select Processed Flight Data');
    if isequal(fName,0); error('No data selected.'); end
    load(fullfile(pPath, fName));
end

% 2. Time Vector
t = flightData.time;
sim_duration = t(end);

% 3. Create Simulink Timeseries
% Actuators (Normalized 0-1) - Connect to RLS Input
ts_u_cmd = timeseries(flightData.actuators.cmd, t, 'Name', 'U_Cmd');

% Sensor Data (Body Frame) - Connect to RLS Measurements
ts_omega = timeseries(flightData.imu.gyro, t, 'Name', 'Gyro');
ts_accel = timeseries(flightData.imu.accel, t, 'Name', 'Accel');

% Ground Truth (For Validation Scope)
if max(abs(flightData.mocap.pos(:))) > 0.1
    ts_pos_truth = timeseries(flightData.mocap.pos, t, 'Name', 'Pos_Truth');
    fprintf('Loaded Replay Data: Using MOCAP for Truth.\n');
else
    ts_pos_truth = timeseries(flightData.px4_est.pos, t, 'Name', 'Pos_Truth');
    fprintf('Loaded Replay Data: Using PX4 ESTIMATE for Truth.\n');
end

% 4. Tustin Derivative Filter Setup (For Omega_Dot)
% RLS needs angular acceleration. We use a "Dirty Derivative" filter:
% H(s) = s / (tau*s + 1) -> Discretized via Tustin
replay_dt = 0.004;      % 250 Hz (Log Rate)
fc_deriv  = 15;         % Cutoff Frequency (Hz) - Tune this! (10-20Hz)
tau_deriv = 1 / (2 * pi * fc_deriv);

denom_term = (2*tau_deriv + replay_dt);

% Discrete Transfer Fcn Coefficients [Num, Den]
% Block Input: Omega (ts_omega) -> Block Output: Omega_Dot
diff_num = [2, -2] / denom_term;
diff_den = [1, (replay_dt - 2*tau_deriv) / denom_term];

fprintf('Derivative Filter: %d Hz Cutoff. \n', fc_deriv);