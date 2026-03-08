function [ts, t, traj_data] = generate_6dof_step_trajectory(varargin)
% GENERATE_6DOF_STEP_TRAJECTORY Generates a sequential step-and-hold trajectory.
%
%   ts = GENERATE_6DOF_STEP_TRAJECTORY() returns a simulink.Timeseries object
%   with default parameters (5s steps, 0.5m / 5deg amplitudes).
%
%   [ts, t, data] = GENERATE_6DOF_STEP_TRAJECTORY(...) returns the raw time
%   vector and data matrix as well.
%
%   ... = GENERATE_6DOF_STEP_TRAJECTORY('Name', Value, ...) allows setting
%   specific parameters.
%
%   OPTIONAL PARAMETERS (Name-Value Pairs):
%       'StepDuration'   - Time to hold the step value [s] (Default: 5)
%       'RestDuration'   - Time to wait at zero between axes [s] (Default: 5)
%       'InitialPadding' - Zero-padding at start [s] (Default: 5)
%       'Dt'             - Sample time [s] (Default: 0.01)
%       'Amplitudes'     - Struct containing fields: x,y,z,roll,pitch,yaw
%                          (Default: 0.5m linear, 5-10 deg angular)
%       'EnablePlotting' - Boolean to show plots (Default: true)
%       'SaveToFile'     - Boolean to save .mat file (Default: false)
%       'FileName'       - Name of file if saving (Default: '6dof_step.mat')
%
%   EXAMPLE:
%       % Define custom amplitudes
%       my_amps.x = 1.0; my_amps.roll = deg2rad(10);
%       
%       % Generate 10s steps with plotting enabled
%       ts = generate_6dof_step_trajectory('StepDuration', 10, ...
%                                          'Amplitudes', my_amps, ...
%                                          'EnablePlotting', true);

    %% 1. Input Parsing
    p = inputParser;
    
    % Default Amplitudes
    default_amps.x = 0.5;
    default_amps.y = 0.5;
    default_amps.z = -0.5; % Negative is often 'Up' in NED
    default_amps.roll  = deg2rad(5);
    default_amps.pitch = deg2rad(5);
    default_amps.yaw   = deg2rad(10);

    addParameter(p, 'StepDuration', 5, @isnumeric);
    addParameter(p, 'RestDuration', 5, @isnumeric);
    addParameter(p, 'InitialPadding', 5, @isnumeric);
    addParameter(p, 'Dt', 0.01, @isnumeric);
    addParameter(p, 'Amplitudes', default_amps, @isstruct);
    addParameter(p, 'EnablePlotting', true, @islogical);
    addParameter(p, 'SaveToFile', false, @islogical);
    addParameter(p, 'FileName', '6dof_step_trajectory.mat', @ischar);
    
    parse(p, varargin{:});
    
    % Extract values for easier reading
    step_dur = p.Results.StepDuration;
    rest_dur = p.Results.RestDuration;
    pad_dur  = p.Results.InitialPadding;
    dt       = p.Results.Dt;
    amps     = p.Results.Amplitudes;
    
    % Merge user amplitudes with defaults (in case user only provided partial struct)
    f_names = fieldnames(default_amps);
    for k = 1:numel(f_names)
        if ~isfield(amps, f_names{k})
            amps.(f_names{k}) = default_amps.(f_names{k});
        end
    end

    %% 2. Time Vector Setup
    % Sequence: X -> Y -> Z -> Roll -> Pitch -> Yaw
    axis_names = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
    num_axes = 6;
    
    duration_per_axis = step_dur + rest_dur;
    total_duration = pad_dur + (num_axes * duration_per_axis);
    
    t = (0:dt:total_duration)';
    num_samples = length(t);
    
    % Initialize Data Matrix [x, y, z, roll, pitch, yaw]
    traj_data = zeros(num_samples, 6);
    
    %% 3. Generate Trajectory
    current_idx = round(pad_dur / dt) + 1;
    samples_step = round(step_dur / dt);
    samples_rest = round(rest_dur / dt);
    
    axis_start_times = zeros(1, num_axes); % For plotting lines later
    
    for i = 1:num_axes
        axis_start_times(i) = t(current_idx);
        
        % Select amplitude
        switch i
            case 1, val = amps.x;
            case 2, val = amps.y;
            case 3, val = amps.z;
            case 4, val = amps.roll;
            case 5, val = amps.pitch;
            case 6, val = amps.yaw;
        end
        
        % Apply Step (Hold value)
        end_step_idx = min(current_idx + samples_step - 1, num_samples);
        traj_data(current_idx:end_step_idx, i) = val;
        
        % Advance pointer (Step + Rest)
        current_idx = end_step_idx + 1 + samples_rest;
        if current_idx > num_samples, break; end
    end
    
    %% 4. Create Timeseries Object
    ts = timeseries(traj_data, t);
    ts.Name = '6DOF_Step_Trajectory';
    ts.DataInfo.Units = 'm_and_rad';
    
    %% 5. Save to File (Optional)
    if p.Results.SaveToFile
        save(p.Results.FileName, 'ts');
        fprintf('Trajectory saved to %s\n', p.Results.FileName);
    end
    
    %% 6. Plotting (Optional)
    if p.Results.EnablePlotting
        plot_trajectory(t, traj_data, axis_start_times, step_dur, axis_names);
    end
end

function plot_trajectory(t, data, start_times, step_dur, names)
    % plot_trajectory
    % Merges Linear and Rotational axes into a single figure with 3 subplots.
    % Uses yyaxis to handle different units (m vs deg).
    
    % Colors
    c_lin = [0, 0.4470, 0.7410];      % Blue
    c_rot = [0.8500, 0.3250, 0.0980]; % Orange
    
    fig = figure('Name', '6-DOF Trajectory (Combined)', 'Color', 'w', 'Position', [100 100 800 600]);
    
    % Titles for the 3 subplots
    row_titles = {'Longitudinal (X / Roll)', 'Lateral (Y / Pitch)', 'Vertical (Z / Yaw)'};
    
    for i = 1:3
        ax = subplot(3, 1, i);
        
        % --- Left Axis: Linear Position ---
        yyaxis left
        plot(t, data(:, i), '-', 'Color', c_lin, 'LineWidth', 1.5);
        ylabel(sprintf('%s [m]', names{i}));
        ax.YAxis(1).Color = c_lin;
        ylim_lin = max(abs(data(:,i))) * 1.5; 
        if ylim_lin==0, ylim_lin=1; end
        ylim([-ylim_lin, ylim_lin]);
        
        % --- Right Axis: Angular Position ---
        yyaxis right
        rot_idx = i + 3;
        plot(t, rad2deg(data(:, rot_idx)), '-', 'Color', c_rot, 'LineWidth', 1.5);
        ylabel(sprintf('%s [deg]', names{rot_idx}));
        ax.YAxis(2).Color = c_rot;
        ylim_rot = max(abs(rad2deg(data(:,rot_idx)))) * 1.5;
        if ylim_rot==0, ylim_rot=1; end
        ylim([-ylim_rot, ylim_rot]);
        
        % Formatting
        grid on;
        title(row_titles{i}, 'FontWeight', 'bold');
        if i == 3
            xlabel('Time [s]');
        end
        
        % Draw Phase Lines & Labels
        % Linear Start
        xline(start_times(i), '--', 'Color', c_lin, 'Alpha', 0.5);
        text(start_times(i)+step_dur/2, ylim_rot*0.8, names{i}, ...
            'Color', c_lin, 'HorizontalAlignment','center','FontSize',8,'BackgroundColor','w', 'EdgeColor', c_lin);
            
        % Rotational Start
        xline(start_times(rot_idx), '--', 'Color', c_rot, 'Alpha', 0.5);
        text(start_times(rot_idx)+step_dur/2, ylim_rot*0.8, names{rot_idx}, ...
            'Color', c_rot, 'HorizontalAlignment','center','FontSize',8,'BackgroundColor','w', 'EdgeColor', c_rot);
    end
end