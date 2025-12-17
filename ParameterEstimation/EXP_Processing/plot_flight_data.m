%% Comprehensive Plotting Script for RLS/UKF Analysis
% Visualizes all essential states with CLEAR TITLES.
% Restoration of Noise Check and Single Motor Analysis.

clear; clc; close all;

%% 1. FILE SELECTION
try; proj = currentProject; startPath = proj.RootFolder; catch; startPath = pwd; end
fprintf('Select a processed .mat file...\n');
[fileName, pathName] = uigetfile({'*.mat', 'Processed Data'}, 'Select Data', startPath);
if isequal(fileName, 0); disp('Cancelled.'); return; end
load(fullfile(pathName, fileName));

if ~exist('flightData', 'var'); error('Invalid .mat file.'); end

%% 2. CONFIGURATION & PRE-CALCULATION
set(0, 'DefaultAxesFontSize', 11); set(0, 'DefaultLineLineWidth', 1.5);
time = flightData.time;

% --- Colors ---
c_px4   = [0 0.4470 0.7410];      % Blue (Est)
c_mocap = [0.8500 0.3250 0.0980]; % Orange (Truth)

% --- Mocap Data Prep ---
hasMocap = isfield(flightData, 'mocap') && any(flightData.mocap.pos(:));

if hasMocap
    % 1. Derive Mocap Velocity (Finite Difference)
    dt_vals = [diff(time); 0.004]; dt_vals(dt_vals==0) = 0.004;
    mocap_vel = zeros(size(time,1), 3);
    for i=1:3
        mocap_vel(:,i) = smoothdata(gradient(flightData.mocap.pos(:,i))./dt_vals, 'gaussian', 20);
    end
    
    % 2. Ensure Euler Angles exist
    if ~isfield(flightData.mocap, 'rpy') || all(flightData.mocap.rpy(:)==0)
        [y, p, r] = quat2angle(flightData.mocap.q);
        flightData.mocap.rpy = [r, p, y];
    end
end

%% FIGURE 1: 3D TRAJECTORY COMPARISON
figure('Name', '3D Trajectory', 'Color', 'w');
plot3(flightData.px4_est.pos(:,1), flightData.px4_est.pos(:,2), flightData.px4_est.pos(:,3), ...
    'Color', c_px4, 'DisplayName', 'PX4 Est (NED)');
hold on; grid on; axis equal; 
zlabel('Down [m]'); xlabel('North [m]'); ylabel('East [m]');
set(gca, 'ZDir', 'reverse'); 

if hasMocap
    plot3(flightData.mocap.pos(:,1), flightData.mocap.pos(:,2), flightData.mocap.pos(:,3), ...
        'Color', c_mocap, 'LineStyle', '--', 'DisplayName', 'Mocap Truth (NED)');
end

title(['Flight Path: ' fileName], 'Interpreter', 'none');
legend('Location', 'best'); view(3);

%% FIGURE 2: STATE COMPARISON (Pos, Vel, Att)
figure('Name', 'State Comparison', 'Color', 'w', 'Position', [50 50 1200 800]);
t = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% Definitions for Loop
ax_labels  = {'X (North)', 'Y (East)', 'Z (Down)'};
att_labels = {'Roll', 'Pitch', 'Yaw'};

% --- ROW 1: POSITION ---
for i=1:3
    nexttile;
    plot(time, flightData.px4_est.pos(:,i), 'Color', c_px4, 'DisplayName', 'PX4'); hold on;
    if hasMocap
        plot(time, flightData.mocap.pos(:,i), 'Color', c_mocap, 'LineStyle', '--', 'DisplayName', 'Mocap');
    end
    grid on; 
    title(['Position ', ax_labels{i}]); 
    if i==1; ylabel('Position [m]'); legend; end
end

% --- ROW 2: VELOCITY (Inertial) ---
for i=1:3
    nexttile;
    if isfield(flightData.px4_est, 'vel')
        plot(time, flightData.px4_est.vel(:,i), 'Color', c_px4, 'DisplayName', 'PX4'); hold on;
    end
    if hasMocap
        plot(time, mocap_vel(:,i), 'Color', c_mocap, 'LineStyle', '--', 'DisplayName', 'Mocap (Deriv)');
    end
    grid on; 
    title(['Velocity ', ax_labels{i}]); 
    if i==1; ylabel('Velocity [m/s]'); end
end

% --- ROW 3: ATTITUDE (Euler) ---
px4_rpy_deg = rad2deg(flightData.px4_est.rpy);
if hasMocap; mocap_rpy_deg = rad2deg(flightData.mocap.rpy); end

for i=1:3
    nexttile;
    plot(time, px4_rpy_deg(:,i), 'Color', c_px4, 'DisplayName', 'PX4'); hold on;
    if hasMocap
        plot(time, mocap_rpy_deg(:,i), 'Color', c_mocap, 'LineStyle', '--', 'DisplayName', 'Mocap');
    end
    grid on; 
    title(['Attitude: ', att_labels{i}]); 
    if i==1; ylabel('Angle [deg]'); end
    xlabel('Time [s]');
end

%% FIGURE 3: ONBOARD ESTIMATOR DYNAMICS
figure('Name', 'PX4 Estimator Dynamics', 'Color', 'w', 'Position', [100 100 1000 800]);
t2 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% 1. Angular Velocity
nexttile;
plot(time, flightData.px4_est.ang_vel, 'LineWidth', 1.5); 
hold on; 
plot(time, flightData.imu.gyro, 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5, 'LineStyle', ':');
grid on; ylabel('Rad/s'); 
title('Angular Velocity (Colored=PX4 Est, Grey=Raw Gyro)'); 
legend('x','y','z');

% 2. Angular Acceleration
nexttile;
if isfield(flightData.px4_est, 'ang_accel')
    plot(time, flightData.px4_est.ang_accel);
    grid on; ylabel('Rad/s^2'); title('PX4 Angular Acceleration Est');
    legend('x','y','z');
else
    text(0.5,0.5,'No Ang Accel Data','HorizontalAlignment','center');
end

% 3. Linear Acceleration (Inertial)
nexttile;
if isfield(flightData.px4_est, 'acc')
    plot(time, flightData.px4_est.acc);
    grid on; ylabel('m/s^2'); 
    title('PX4 Linear Acceleration Est (Inertial NED)');
    legend('N','E','D');
else
    text(0.5,0.5,'No Lin Accel Data','HorizontalAlignment','center');
end

% 4. Body Frame Velocity (New Check)
nexttile;
if isfield(flightData.px4_est, 'vel_body')
    plot(time, flightData.px4_est.vel_body);
    grid on; ylabel('m/s'); xlabel('Time [s]');
    title('PX4 Body Frame Velocity (Derived)');
    legend('u (fwd)','v (right)','w (down)');
else
    text(0.5,0.5,'No Body Velocity Data','HorizontalAlignment','center');
end


%% FIGURE 4: MOTOR 1 ANALYSIS (Restored)
figure('Name', 'Motor 1 Analysis', 'Color', 'w');
if isfield(flightData.actuators, 'rpm')
    rpm = flightData.actuators.rpm;
    cmd = flightData.actuators.cmd;
    
    % --- SCALING LOGIC ---
    max_rpm = max(rpm(:)); if max_rpm < 1; max_rpm = 1; end
    rpm_norm = rpm / max_rpm;
    
    max_cmd = max(cmd(:)); min_cmd = min(cmd(:));
    if max_cmd > 1.1 && min_cmd > 800 % PWM case
        cmd_norm = (cmd - 1000) / 1000; cmd_norm(cmd_norm<0)=0;
    elseif max_cmd > 1.1 % Raw huge number case
        cmd_norm = cmd / max_cmd;
    else % Already 0-1
        cmd_norm = cmd;
    end
    
    % PLOT ONLY MOTOR 1
    plot(time, cmd_norm(:,1), 'Color', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Command (M1)');
    hold on;
    plot(time, rpm_norm(:,1), 'Color', c_px4, 'LineWidth', 1.5, 'DisplayName', 'RPM (M1)');
    
    ylabel('Norm. [0-1]'); 
    title(sprintf('Motor 1 Analysis (Max RPM: %.0f)', max_rpm));
    legend('Location', 'best'); ylim([-0.1 1.1]); grid on;
    xlabel('Time [s]');
else
    text(0.5, 0.5, 'No RPM Data', 'HorizontalAlignment', 'center');
end

%% FIGURE 5: NOISE CHECK (Restored)
figure('Name', 'Noise Check', 'Color', 'w');
% Calculate numerical derivative of gyro
dt = mean(diff(time));
if dt == 0; dt = 0.004; end
ang_accel_x = diff(flightData.imu.gyro(:,1)) ./ dt;
plot(time(1:end-1), ang_accel_x, 'Color', 'r');
title('Raw Angular Acceleration (Finite Difference)');
ylabel('rad/s^2'); xlabel('Time [s]');
grid on;
subtitle('If this looks like a solid block of color, you need a Low-Pass Filter.');