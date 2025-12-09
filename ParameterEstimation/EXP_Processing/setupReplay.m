%% Setup Simulink Replay
% Loads the processed .mat file and creates timeseries objects for Simulink
clear; clc;

% 1. Load Data
[fName, pName] = uigetfile('*.mat', 'Select Processed Flight Data');
if isequal(fName,0); return; end
load(fullfile(pName, fName));

% 2. Extract Time Vector (ensure it starts at 0)
t = flightData.time;

% 3. Create Simulink Signals (Timeseries)
% format: timeseries(Data, Time, 'Name', 'SignalName')

% --- INPUTS (For RLS/UKF Prediction Step) ---
% Motor Commands (Normalized 0-1)
ts_u_cmd  = timeseries(flightData.actuators.cmd, t, 'Name', 'MotorCommands');

% Angular Velocity (Gyro) [rad/s]
% Note: Check your frame! PX4 is FRD. If your model is ENU, flip Y and Z here.
ts_omega  = timeseries(flightData.imu.gyro, t, 'Name', 'Gyro');

% Linear Acceleration [m/s^2]
ts_accel  = timeseries(flightData.imu.accel, t, 'Name', 'Accel');

% --- TRUTH (For Validation) ---
% PX4 Position Estimate (to compare with your UKF)
ts_pos_truth = timeseries(flightData.px4_est.pos, t, 'Name', 'Pos_Truth');

% Mocap (if available)
ts_mocap = timeseries(flightData.mocap.pos, t, 'Name', 'Mocap_Truth');

% 4. Simulation Settings
sim_dt = 0.004; % 250Hz (Matches the re-sampling step)
sim_duration = t(end);

fprintf('Data loaded. \nReady to run Simulink for %.2f seconds.\n', sim_duration);


%% RLS Filtered Derivative Settings
% Sampling time (must match your log resampling)
dt = 0.004; % 250 Hz

% Cutoff Frequency (The Tuning Knob)
% Start at 15 Hz. 
% - If noisy: Decrease to 10 Hz.
% - If lagging: Increase to 25 Hz.
fc = 15; 
tau = 1 / (2 * pi * fc);

% --- Calculate Coefficients (Tustin Method) ---
% H(z) = ( 2*(z - 1) ) / ( (2*tau + dt)*z + (dt - 2*tau) )

denom_term = (2*tau + dt);

% Numerator: [n0, n1]
diff_num = [2, -2] / denom_term;

% Denominator: [1, d1]
diff_den = [1, (dt - 2*tau) / denom_term];

fprintf('Derivative Filter Cutoff: %d Hz\n', fc);
fprintf('Num: [%f, %f]\n', diff_num(1), diff_num(2));
fprintf('Den: [%f, %f]\n', diff_den(1), diff_den(2));