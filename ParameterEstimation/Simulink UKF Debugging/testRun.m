% test_UKF_setup_step2.m
% This script defines parameters for testing the REAL measurement function.
% It uses the full 17-state augmented vector.

clear UKF_full;
clc;

disp('Setting up parameters for the full measurement function test...');

% --- Define System Dimensions ---
num_dynamic_states = 13; % [pos, VBody, q, nuBody]
num_params = 4;          % [mass, Ixx, Iyy, Izz]
UKF_full.num_states = num_dynamic_states + num_params;
UKF_full.num_measurements = 6; % [accel_x,y,z, gyro_x,y,z]

% --- Initial State (x0) ---
UKF_full.InitialState = zeros(UKF_full.num_states, 1);
UKF_full.InitialState(7) = 1; % Set a valid quaternion [1,0,0,0] to avoid NaN

% --- Initial Covariance (P0) ---
UKF_full.InitialCovariance = eye(UKF_full.num_states) * 0.1;

% --- Process Noise (Q) ---
UKF_full.ProcessNoise = eye(UKF_full.num_states) * 0.01;

% --- Measurement Noise (R) ---
UKF_full.MeasurementNoise = eye(UKF_full.num_measurements) * 0.5;

% --- Unscented Transform Parameters ---
UKF_full.Alpha = 1e-3;
UKF_full.Beta = 2;
UKF_full.Kappa = 0;

disp('Full-size UKF test parameters created in the "UKF_full" struct.');
