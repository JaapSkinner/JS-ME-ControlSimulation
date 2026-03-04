%% Plot Validation Results: Nominal vs. RLS Comparison
% This script loads a "Validation_*.mat" file containing two simulation runs.
% It generates:
% 1. Step-Specific RMSE Metrics.
% 2. Overall Mission RMSE Metrics.
% 3. 6-DOF Comparison Plots (Ordered: Translation Left, Rotation Right).
% 4. X-Axis Analysis (X-Pos -> Pitch Error -> Pitch Torque).
% 5. Leakage Analysis (X-Maneuver -> Z-Response).

clearvars; clc; close all;

%% 1. Load Data
fprintf('Select a Validation Result file (Validation_*.mat)...\n');
baseDir = fullfile(pwd, 'Results', 'Mixer_Validation_Batch');
if ~isfolder(baseDir), baseDir = pwd; end
[fName, pName] = uigetfile(fullfile(baseDir, '*.mat'), 'Select Validation Data');
if isequal(fName, 0), return; end
load(fullfile(pName, fName));
fprintf('Loaded: %s\n', fName);

if ~exist('simOut_Nominal', 'var') || ~exist('simOut_RLS', 'var')
    error('File missing comparison data.');
end

%% 2. Data Extraction
names_ref    = {'trajectory_timeseries', 'Ref_Trajectory', 'Reference'};
names_states = {'UAV_State', 'States', 'x_state', 'Plant_States'};
names_torque = {'Moments_cmd', 'Torque_cmd', 'Moments'};

% --- Extract Reference ---
if exist('trajectory_timeseries', 'var')
    ts_ref = trajectory_timeseries;
else
    ts_ref = find_signal(simOut_Nominal, names_ref);
end
[t_ref, pos_ref] = sanitize_data(ts_ref, 1:3); 
[~,     att_ref] = sanitize_data(ts_ref, 4:6);

% --- Extract Nominal & RLS ---
[t_nom, pos_nom, att_nom, torq_nom] = extract_run(simOut_Nominal, names_states, names_torque);
[t_rls, pos_rls, att_rls, torq_rls] = extract_run(simOut_RLS, names_states, names_torque);

fprintf('Data Extraction Complete.\n');

%% 3. Metrics Configuration
T_pad  = 20; 
T_step = 10; 
T