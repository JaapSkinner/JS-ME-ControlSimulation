%% Comprehensive Plotting Script for RLS/UKF Analysis
% Visualizes all essential states: Omega, Accel, Mocap, Motors, Pos/Vel.
clear; clc; close all;

%% 1. FILE SELECTION
try
    proj = currentProject;
    startPath = proj.RootFolder;
catch
    startPath = pwd;
end

fprintf('Select a processed .mat file...\n');
[fileName, pathName] = uigetfile({'*.mat', 'Processed Data'}, 'Select Data', startPath);
if isequal(fileName, 0); disp('Cancelled.'); return; end

load(fullfile(pathName, fileName));
if ~exist('flightData', 'var'); error('Invalid .mat file.'); end

%% 2. CONFIGURATION & PRE-CALCULATION
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.2);

time = flightData.time;
blue   = [0 0.4470 0.7410];
orange = [0.8500 0.3250 0.0980];
yellow = [0.9290 0.6940 0.1250];
purple = [0.4940 0.1840 0.5560];

% Calculate Velocity from Mocap (if available) for comparison
mocap_vel = zeros(size(time,1), 3);
if isfield(flightData, 'mocap') && any(flightData.mocap.pos(:))
    dt_vals = [diff(time); 0.004]; % Estimate dt
    % Simple finite difference for visualization
    mocap_vel(:,1) = gradient(flightData.mocap.pos(:,1)) ./ dt_vals;
    mocap_vel(:,2) = gradient(flightData.mocap.pos(:,2)) ./ dt_vals;
    mocap_vel(:,3) = gradient(flightData.mocap.pos(:,3)) ./ dt_vals;
end

%% FIGURE 1: 3D TRAJECTORY (Spatial Context)
figure('Name', '3D Trajectory', 'Color', 'w');
plot3(flightData.px4_est.pos(:,1), flightData.px4_est.pos(:,2), flightData.px4_est.pos(:,3), ...
    'Color', blue, 'DisplayName', 'PX4 Estimate');
hold on; grid on; axis equal;

if isfield(flightData, 'mocap') && any(flightData.mocap.pos(:))
    plot3(flightData.mocap.pos(:,1), flightData.mocap.pos(:,2), flightData.mocap.pos(:,3), ...
        'Color', orange, 'LineStyle', '--', 'DisplayName', 'Mocap Ground Truth');
end

xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
title(['Flight Path: ' fileName], 'Interpreter', 'none');
legend('Location', 'best');
view(3);

%% FIGURE 2: KINEMATIC STATES (Pos, Vel, Attitude)
figure('Name', 'Kinematics', 'Color', 'w', 'Position', [50 50 1000 800]);
t = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% 1. Position
nexttile;
plot(time, flightData.px4_est.pos(:,1), 'Color', blue, 'DisplayName', 'Est X'); hold on;
plot(time, flightData.px4_est.pos(:,2), 'Color', orange, 'DisplayName', 'Est Y');
plot(time, flightData.px4_est.pos(:,3), 'Color', yellow, 'DisplayName', 'Est Z');
% Optional: Overlay Mocap dashed
if isfield(flightData, 'mocap') && any(flightData.mocap.pos(:))
    plot(time, flightData.mocap.pos(:,1), 'Color', blue, 'LineStyle', ':', 'HandleVisibility', 'off');
    plot(time, flightData.mocap.pos(:,2), 'Color', orange, 'LineStyle', ':', 'HandleVisibility', 'off');
    plot(time, flightData.mocap.pos(:,3), 'Color', yellow, 'LineStyle', ':', 'HandleVisibility', 'off');
end
grid on; ylabel('Position [m]'); title('Position (Solid=Est, Dotted=Mocap)');
legend('Location', 'best');

% 2. Velocity
nexttile;
% Note: PX4 Estimate usually doesn't export velocity in local_position directly in all logs
% If you derived it or have it, plot it. Here we plot Mocap Velocity.
plot(time, mocap_vel(:,1), 'Color', blue, 'DisplayName', 'Vel X'); hold on;
plot(time, mocap_vel(:,2), 'Color', orange, 'DisplayName', 'Vel Y');
plot(time, mocap_vel(:,3), 'Color', yellow, 'DisplayName', 'Vel Z');
grid on; ylabel('Velocity [m/s]'); title('Velocity (Derived from Mocap)');
legend('Location', 'best');

% 3. Orientation (Quaternions - Raw)
% Note: RLS needs this for the Gravity Vector rotation
nexttile;
% Assuming attitude is available in PX4 data (often not in basic ulog unless added)
% If not, plot Mocap Quaternions
if isfield(flightData, 'mocap') && isfield(flightData.mocap, 'quat')
    q = flightData.mocap.quat;
    plot(time, q(:,1), 'k', 'DisplayName', 'q0'); hold on;
    plot(time, q(:,2), 'r', 'DisplayName', 'q1');
    plot(time, q(:,3), 'g', 'DisplayName', 'q2');
    plot(time, q(:,4), 'b', 'DisplayName', 'q3');
    ylabel('Quaternion'); title('Attitude (Mocap Quaternion)');
    legend('Location', 'best');
else
    text(0.5, 0.5, 'Attitude Data Not Available', 'HorizontalAlignment', 'center');
end
grid on; xlabel('Time [s]');

%% FIGURE 3: RLS INPUTS (The "Meat" for System ID)
figure('Name', 'System ID Inputs', 'Color', 'w', 'Position', [100 50 1000 900]);
t2 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% 1. Angular Velocity (Omega) - CRITICAL for RLS
nexttile;
plot(time, flightData.imu.gyro(:,1), 'Color', blue, 'DisplayName', '\omega_x'); hold on;
plot(time, flightData.imu.gyro(:,2), 'Color', orange, 'DisplayName', '\omega_y');
plot(time, flightData.imu.gyro(:,3), 'Color', yellow, 'DisplayName', '\omega_z');
grid on; ylabel('Rad/s'); title('Angular Velocity (Gyro)');
legend('Location', 'best');

% 2. Linear Acceleration (Accel) - CRITICAL for Observation/Update
nexttile;
plot(time, flightData.imu.accel(:,1), 'Color', blue, 'DisplayName', 'a_x'); hold on;
plot(time, flightData.imu.accel(:,2), 'Color', orange, 'DisplayName', 'a_y');
plot(time, flightData.imu.accel(:,3), 'Color', yellow, 'DisplayName', 'a_z');
grid on; ylabel('m/s^2'); title('Linear Acceleration (Raw IMU)');
legend('Location', 'best');

% 3. Motor Commands vs RPM - CRITICAL for Regressor Matrix (Thrust/Torque)
nexttile;
% Plot first 4 motors only to keep it readable
for i = 1:4
    % Normalize for visualization
    cmd = flightData.actuators.cmd(:,i);
    if mean(cmd) > 100; cmd = (cmd-1000)/1000; end % PWM Norm
    rpm = flightData.actuators.rpm(:,i);
    rpm_scale = max(rpm(:)); if rpm_scale==0; rpm_scale=1; end
    
    plot(time, cmd, 'Color', [0.5 0.5 0.5 0.5], 'DisplayName', 'Cmd (Norm)'); hold on;
    plot(time, rpm./rpm_scale, 'Color', blue, 'LineWidth', 1, 'DisplayName', 'RPM (Norm)');
end
grid on; ylabel('Norm. Input'); xlabel('Time [s]');
title('Actuation (Gray=Command, Blue=RPM)');
ylim([-0.1 1.1]);

%% FIGURE 4: VIBRATION / NOISE CHECK (Optional)
% Shows why you need filtering before RLS differentiation
figure('Name', 'Noise Check', 'Color', 'w');
plot(time(1:end-1), diff(flightData.imu.gyro(:,1))./diff(time), 'r');
title('Raw Angular Acceleration (Finite Difference)');
ylabel('rad/s^2'); xlabel('Time [s]');
grid on;
subtitle('If this looks like a solid block of color, you need a Low-Pass Filter.');