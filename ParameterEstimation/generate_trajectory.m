% generate_6dof_trajectory.m
%
% This script creates an "exciting" 6-DOF trajectory for a UAV to follow.
% The trajectory is based on Lissajous curves for all six degrees of
% freedom (x, y, z, roll, pitch, yaw) to ensure rich and persistent
% excitation of the vehicle's dynamics, which is crucial for robust
% parameter estimation.

disp('Generating 6-DOF exciting trajectory...');

% --- General Parameters ---
motion_duration = 60; % [s] Duration of the actual Lissajous motion
padding_duration = 2;   % [s] Duration of zero-padding at the start
dt = 0.01;              % [s] Sample time, should match Simulink's solver
enable_plotting = true; % FLAG: Set to true to show plots, false to hide

% --- Generate the Time Vectors ---
total_duration = padding_duration + motion_duration;
t = (0:dt:total_duration)'; % Full time vector for simulation
t_motion = (0:dt:motion_duration)'; % Time vector for the motion part

% --- What are Lissajous Trajectories? ---
% A Lissajous curve is the path traced by a point whose coordinates are
% sinusoidal functions of time. For 2D, this is:
%   x(t) = A * sin(a*t + phase_x)
%   y(t) = B * sin(b*t + phase_y)
% The shape of the curve is determined by the ratio of frequencies (a/b).
% By applying this concept to all 6 degrees of freedom, we create a
% complex, oscillating trajectory that persistently excites the system's
% dynamics across multiple axes simultaneously. This is ideal for system
% identification as it ensures the data collected is rich with information.
%
% Source: https://en.wikipedia.org/wiki/Lissajous_curve

% --- Trajectory Parameters ---
% Using different amplitudes, frequencies, and phases for each degree of freedom
% generates complex, coupled motions that are ideal for system identification.

% --- Positional Trajectory Parameters (x, y, z) ---
% Amplitudes [m]
pos_amp_x = 0.5;
pos_amp_y = 0.5;
pos_amp_z = 0.25;

% Frequencies [Hz] - Ratios are important for Lissajous shapes
pos_freq_x = 0.1;
pos_freq_y = 0.15;
pos_freq_z = 0.2;

% Phase Offsets [rad]
pos_phase_x = 0;
pos_phase_y = pi/2;
pos_phase_z = pi/4;

% --- Rotational Trajectory Parameters (roll, pitch, yaw) ---
% Amplitudes [rad] - Keep roll and pitch angles modest to be realistic
rot_amp_roll  = pi/48; % +/- 4 degrees
rot_amp_pitch = pi/48; % +/- 4 degrees
rot_amp_yaw   = pi/12;  % +/-  degrees

% Frequencies [Hz] - Generally higher than position frequencies
rot_freq_roll  = 0.25;
rot_freq_pitch = 0.3;
rot_freq_yaw   = 0.4;

% Phase Offsets [rad]
rot_phase_roll  = 0;
rot_phase_pitch = pi/3;
rot_phase_yaw   = 0;


% --- Generate Position Setpoints for Motion Period ---
% Phase is implemented as a time delay. For each signal, a delay is
% calculated (delta_t = phase / (2*pi*f)). The signal remains zero until
% this delay has passed, ensuring a smooth start from the padding period
% without introducing a DC offset.

% Calculate time delays from phase and frequency
pos_delay_x = pos_phase_x / (2 * pi * pos_freq_x);
pos_delay_y = pos_phase_y / (2 * pi * pos_freq_y);
pos_delay_z = pos_phase_z / (2 * pi * pos_freq_z);

% Create shifted time vectors for each signal, starting only after the delay
t_x = max(0, t_motion - pos_delay_x);
t_y = max(0, t_motion - pos_delay_y);
t_z = max(0, t_motion - pos_delay_z);

% Generate signals using the individual delayed time vectors
x_motion = pos_amp_x * sin(2 * pi * pos_freq_x * t_x);
y_motion = pos_amp_y * sin(2 * pi * pos_freq_y * t_y);
% Adjust z to start at 0 and move downwards, typical for UAV takeoff
z_motion = -pos_amp_z * (1 - cos(2 * pi * pos_freq_z * t_z));


% --- Generate Attitude Setpoints for Motion Period ---
% Calculate time delays from phase and frequency
rot_delay_roll  = rot_phase_roll  / (2 * pi * rot_freq_roll);
rot_delay_pitch = rot_phase_pitch / (2 * pi * rot_freq_pitch);
rot_delay_yaw   = rot_phase_yaw   / (2 * pi * rot_freq_yaw);

% Create shifted time vectors for each signal, starting only after the delay
t_roll  = max(0, t_motion - rot_delay_roll);
t_pitch = max(0, t_motion - rot_delay_pitch);
t_yaw   = max(0, t_motion - rot_delay_yaw);

% Generate signals using the individual delayed time vectors
roll_motion  = rot_amp_roll  * sin(2 * pi * rot_freq_roll * t_roll);
pitch_motion = rot_amp_pitch * sin(2 * pi * rot_freq_pitch * t_pitch);
yaw_motion   = rot_amp_yaw   * sin(2 * pi * rot_freq_yaw * t_yaw);


% --- Assemble the Final Trajectory Matrix ---
% 1. Create the data matrix for the motion part
motion_data = [x_motion, y_motion, z_motion, roll_motion, pitch_motion, yaw_motion];

% 2. Create the zero-padding matrix
padding_samples = round(padding_duration / dt);
padding_data = zeros(padding_samples, 6);

% 3. Concatenate padding and motion data
trajectory_data = [padding_data; motion_data];

% The structure is [time, data]. The data is a 6x1 vector:
% [x, y, z, roll, pitch, yaw]


% --- Create the Timeseries Object for Simulink ---
% This is the object the 'From Workspace' block in Simulink will read.
trajectory_timeseries = timeseries(trajectory_data, t);
trajectory_timeseries.Name = '6DOF_Lissajous_Trajectory';
trajectory_timeseries.DataInfo.Units = 'm_and_rad';


disp('6-DOF trajectory generated and loaded into "trajectory_timeseries".');
disp('You can now use this variable in your Simulink model.');

% --- Optional: Plotting ---
if enable_plotting
    figure('Name', '6-DOF Trajectory');

    % 3D Positional Trajectory
    subplot(2,2,1);
    plot3(trajectory_data(:,1), trajectory_data(:,2), trajectory_data(:,3), 'b');
    title('3D Position Trajectory');
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    grid on;
    axis equal;

    % Position vs. Time
    subplot(2,2,2);
    plot(t, trajectory_data(:,1), 'r', t, trajectory_data(:,2), 'g', t, trajectory_data(:,3), 'b');
    title('Position vs. Time');
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('x', 'y', 'z');
    grid on;

    % Attitude vs. Time
    subplot(2,2,3);
    plot(t, trajectory_data(:,4), 'r', t, trajectory_data(:,5), 'g', t, trajectory_data(:,6), 'b');
    title('Attitude vs. Time');
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
end



