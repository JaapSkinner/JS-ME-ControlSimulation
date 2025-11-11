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
enable_plotting = false; % FLAG: Set to true to show plots, false to hide
enable_saving = false;   % FLAG: Set to true to save the .mat file
enable_plot_saving = false; % FLAG: Set to true to save plots as images
save_folder = 'ParameterEstimation/figures'; % Folder to save the .mat file
save_filename = '6dof_trajectory.mat'; % File name for the .mat file
plot_save_folder = 'ParameterEstimation/figures'; % Folder to save the plot images

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
pos_amp_x = 0.05;
pos_amp_y = 0.05;
pos_amp_z = 0.025;

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
rot_amp_roll  = 0.3*pi/64; % +/- 2 degrees
rot_amp_pitch = 0.3*pi/64; % +/- 2 degrees
rot_amp_yaw   = 0.3*pi/64;  % +/- 2 degrees

% Frequencies [Hz] - Generally higher than position frequencies
rot_freq_roll  = 0.25;
rot_freq_pitch = 0.275;
rot_freq_yaw   = 0.125;

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


% --- Save Trajectory to File ---
if enable_saving
    if ~exist(save_folder, 'dir') % Check if folder exists
        mkdir(save_folder);       % Create folder if it doesn't
        disp(['Folder "' save_folder '" created.']);
    end
    save_path = fullfile(save_folder, save_filename);
    save(save_path, 'trajectory_timeseries');
    disp(['Trajectory saved to "' save_path '".']);
end


disp('6-DOF trajectory generated and loaded into "trajectory_timeseries".');
disp('You can now use this variable in your Simulink model.');

% --- Optional: Plotting ---
if enable_plotting
    % Create plot save folder if it doesn't exist
    if enable_plot_saving
        if ~exist(plot_save_folder, 'dir') % Check if folder exists
            mkdir(plot_save_folder);       % Create folder if it doesn't
            disp(['Plot save folder "' plot_save_folder '" created.']);
        end
    end

    % Set global font sizes for readability (optional, but good for thesis)
    set(groot, 'defaultAxesFontSize', 12);   % For tick labels
    set(groot, 'defaultLegendFontSize', 10); % For legend text
    set(groot, 'defaultTextFontSize', 12);   % For axis labels

    % Define a nicer color palette (RGB triplets)
    color_blue   = [0, 0.4470, 0.7410];
    color_orange = [0.8500, 0.3250, 0.0980];
    color_green  = [0.4660, 0.6740, 0.1880];

    % Figure 1: 3D Positional Trajectory
    fig1 = figure('Name', '3D Position Trajectory');
    plot3(trajectory_data(:,1), trajectory_data(:,2), trajectory_data(:,3), 'Color', color_blue, 'LineWidth', 1.5);
    % title('3D Position Trajectory', 'FontSize', 14, 'FontWeight', 'bold'); % Title removed for thesis caption
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    zlabel('Z Position [m]');
    grid on;
    axis equal;
    box on; % Add a box for a cleaner look

    % Save Figure 1
    if enable_plot_saving
        fig1_filename = fullfile(plot_save_folder, '3d_position_trajectory.png');
        print(fig1, fig1_filename, '-dpng', '-r300'); % Save as 300 DPI PNG
        disp(['Saved Figure 1 to "' fig1_filename '"']);
    end

    % Figure 2: Position States vs. Time
    fig2 = figure('Name', 'Position States vs. Time');
    hold on;
    plot(t, trajectory_data(:,1), '-', 'Color', color_blue, 'LineWidth', 1.5);
    plot(t, trajectory_data(:,2), '-', 'Color', color_orange, 'LineWidth', 1.5);
    plot(t, trajectory_data(:,3), '-', 'Color', color_green, 'LineWidth', 1.5);
    hold off;
    % title('Position States vs. Time', 'FontSize', 14, 'FontWeight', 'bold'); % Title removed for thesis caption
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('x-position', 'y-position', 'z-position', 'Location', 'best');
    grid on;
    box on;
    
    % --- Add vertical margin and limit x-axis ---
    % Find data range (excluding initial padding for min/max calculation)
    motion_pos_data = trajectory_data(padding_samples+1:end, 1:3);
    min_pos = min(motion_pos_data, [], 'all');
    max_pos = max(motion_pos_data, [], 'all');
    pos_range = max_pos - min_pos;
    pos_margin = pos_range * 0.2 + 1e-6; % Increased margin to 20%
    
    ylim([min_pos - pos_margin, max_pos + pos_margin]);
    xlim([0, 20]); % Limit x-axis to first 20 seconds
    % ---

    % Save Figure 2
    if enable_plot_saving
        fig2_filename = fullfile(plot_save_folder, 'position_states_vs_time.png');
        print(fig2, fig2_filename, '-dpng', '-r300'); % Save as 300 DPI PNG
        disp(['Saved Figure 2 to "' fig2_filename '"']);
    end

    % Figure 3: Attitude States vs. Time
    fig3 = figure('Name', 'Attitude States vs. Time');
    hold on;
    plot(t, trajectory_data(:,4), '-', 'Color', color_blue, 'LineWidth', 1.5);
    plot(t, trajectory_data(:,5), '-', 'Color', color_orange, 'LineWidth', 1.5);
    plot(t, trajectory_data(:,6), '-', 'Color', color_green, 'LineWidth', 1.5);
    hold off;
    % title('Attitude States vs. Time', 'FontSize', 14, 'FontWeight', 'bold'); % Title removed for thesis caption
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    % Use TeX interpreter for Greek letters in the legend
    legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)', 'Location', 'best', 'Interpreter', 'tex');
    grid on;
    box on;

    % --- Add vertical margin and limit x-axis ---
    % Find data range (excluding initial padding for min/max calculation)
    motion_att_data = trajectory_data(padding_samples+1:end, 4:6);
    min_att = min(motion_att_data, [], 'all');
    max_att = max(motion_att_data, [], 'all');
    att_range = max_att - min_att;
    att_margin = att_range * 0.2 + 1e-6; % Increased margin to 20%
    
    ylim([min_att - att_margin, max_att + att_margin]);
    xlim([0, 20]); % Limit x-axis to first 20 seconds
    % ---

    % Save Figure 3
    if enable_plot_saving
        fig3_filename = fullfile(plot_save_folder, 'attitude_states_vs_time.png');
        print(fig3, fig3_filename, '-dpng', '-r300'); % Save as 300 DPI PNG
        disp(['Saved Figure 3 to "' fig3_filename '"']);
    end

    % Reset global font sizes if needed
    % set(groot, 'defaultAxesFontSize', 'remove');
    % ... (reset others if you wish)
end










