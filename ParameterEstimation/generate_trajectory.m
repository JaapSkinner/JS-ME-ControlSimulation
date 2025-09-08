% generate_trajectory.m
%
% This script creates an "exciting" trajectory for the UAV to follow.
% The trajectory is a Lissajous curve, designed to excite both translational
% and rotational dynamics, which is crucial for parameter estimation.

disp('Generating exciting trajectory...');

% --- Trajectory Parameters ---
simulation_time = 60; % [s] Total duration of the trajectory
dt = 0.01;            % [s] Sample time, should match Simulink's solver

% Amplitudes of motion
amp_x = 2.0; % [m]
amp_y = 2.0; % [m]
amp_z = 1.0; % [m]

% Frequencies of motion (using different frequencies creates complex paths)
freq_x = 0.5; % [Hz]
freq_y = 0.3; % [Hz]
freq_z = 0.25; % [Hz]

% Yaw motion parameters
yaw_rate_max = pi/4; % [rad/s]

% --- Generate the Time Vector ---
t = (0:dt:simulation_time)';

% --- Generate Position Setpoints [x, y, z] ---
% This creates a smooth, continuous path that explores the 3D space.
x_des = amp_x * sin(2 * pi * freq_x * t);
y_des = amp_y * cos(2 * pi * freq_y * t);
z_des = -amp_z * (1 - cos(2 * pi * freq_z * t)); % Starts at 0, goes down to -2m

% --- Generate Yaw Setpoint [psi] ---
% A simple ramp for yaw to ensure continuous rotation.
yaw_des = yaw_rate_max * t;
% You can also make this sinusoidal for more complex rotational excitation.
% yaw_des = (pi/2) * sin(2 * pi * 0.1 * t);


% --- Assemble the Final Trajectory Matrix ---
% The structure should be [time, data]. The data is a 4x1 vector [x; y; z; yaw].
trajectory_data = [x_des, y_des, z_des, yaw_des];

% --- Create the Timeseries Object for Simulink ---
% This is the object the 'From Workspace' block will read.
trajectory_timeseries = timeseries(trajectory_data, t);

disp('Trajectory generated and loaded into "trajectory_timeseries".');
