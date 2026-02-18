function runSimulation_ExampleCase()
%% DEBUG CASE: Deterministic Perturbation for RLS vs UKF Validation
% This script generates a single file with KNOWN errors:
% 1. Motor.K_E increased by 15% (simulating standard deviation)
% 2. Uav.COM shifted by +3cm in Z (simulating absolute deviation)
%
% It guarantees the saved 'B_matrix_nominal' is the CLEAN, UNPERTURBED version.

clearvars; clc;

%% 1. Initialization (Load Nominal Parameters)
fprintf('Initializing Nominal Plant...\n');
run('ParameterEstimationBase.m');
run('InitUKF.m'); 
run('InitRLS.m');

% --- CRITICAL STEP: CAPTURE NOMINAL B MATRIX NOW ---
% We save this immediately to emagnetic screw t lab
%nsure it represents the "Perfect" plant.
if ~exist('B_matrix_nominal', 'var')
    error('B_matrix_nominal not found after InitRLS. Check your init scripts.');
end
BAxis   | RLS RMSE     | Nominal RMSE
-------------------------------------------------
Fx     | 0.0168       | 0.0304      
Fy     | 0.0193       | 0.0427      
Fz     | 0.2913       | 2.2231      
Mx     | 0.0028       | 0.0040      
My     | 0.0033       | 0.0147      
Mz     | 0.0008       | 0.0010      
ppr2exllldd130_matrix_nominal_TRUE = B_matrix_nominal; 
fprintf('Nominal B Matrix captured. First element: %f\n', B_matrix_nominal_TRUE(1,1));

%% 2. Trajectory Generation
fprintf('Generating Trajectories...\n');
[multisine, sineT, ~] = generate_orthogonal_multisine(8, 15, 0.1, 1.0, 100);
multisinesignal = [sineT, multisine];

run('generate_trajectory.m'); % Generates trajectory_timeseries

%% 3. Apply Deterministic Perturbations
fprintf('Applying Deterministic Perturbations...\n');

% Create copies
Uav_pert = Uav;
Motor_pert = Motor;

% --- PERTURBATION A: Motor K_E (+15% Variation) ---
% We apply a uniform multiplier to simulate a systematic offset or high std dev
ke_multiplier = 1.15; 
Motor_pert.K_E = Motor.K_E * ke_multiplier;
fprintf('  -> Motor.K_E scaled by %.2f\n', ke_multiplier);

% --- PERTURBATION B: COM Z-Shift (+3cm Absolute) ---
delta_Z = 0.03; % Meters
Uav_pert.COM = [0, 0, delta_Z]; 

% IMPORTANT: When COM changes, Motor Locations (Arm lengths) relative to COM change.
% We must update Uav.MotorLoc manually (Logic taken from sampleParameters)
delta_vec = [0, 0, delta_Z];
% Subtract delta from motor coordinates (COM shift means motors move "opposite" in body frame)
Uav_pert.MotorLoc(:, 1:3) = Uav.MotorLoc(:, 1:3) - repmat(delta_vec, Uav.N_ROTORS, 1);
% Recalculate arm lengths (4th column)
Uav_pert.MotorLoc(:, 4) = sqrt(sum(Uav_pert.MotorLoc(:, 1:3).^2, 2));

fprintf('  -> Uav.COM shifted by +%.3f m in Z. Motor positions updated.\n', delta_Z);

%% 4. Prepare Simulation Environment
% Run bus generation (Just in case it needs variables, but we protect B_nom)
run('mlebusgen.m'); 

% RESTORE THE TRUE NOMINAL MATRIX
% (In case mlebusgen recalculated it using the perturbed workspace vars)
B_matrix_nominal = B_matrix_nominal_TRUE; 

%% 5. Setup Simulation Input
simIn = Simulink.SimulationInput(modelName);
% Disable State Loading
simIn = simIn.setModelParameter('LoadInitialState', 'off');
simIn = simIn.setModelParameter('LoadExternalInput', 'off');

% Inject Variables
simIn = simIn.setVariable('windInput', windInput);
simIn = simIn.setVariable('uavType', uavType);
simIn = simIn.setVariable('windFile', windFile);
simIn = simIn.setVariable('Simulation', Simulation);
simIn = simIn.setVariable('Initial', Initial);
simIn = simIn.setVariable('InitialRLS', RLS_INITIAL_GUESS);
simIn = simIn.setVariable('Aero', Aero);
simIn = simIn.setVariable('MLEBUS', MLEBUS);
simIn = simIn.setVariable('UKF', UKF);
simIn = simIn.setVariable('trajectory_timeseries', trajectory_timeseries);
simIn = simIn.setVariable('multisinesignal', multisinesignal);

% *** THE IMPORTANT PART: Inject the PERTURBED Plant ***
simIn = simIn.setVariable('Uav', Uav_pert);
simIn = simIn.setVariable('Motor', Motor_pert);

%% 6. Run Simulation
fprintf('Running Simulation...\n');
simOut = sim(simIn);

%% 7. Save Results
outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation', 'DEBUG_DATA');
if ~isfolder(outputFolder), mkdir(outputFolder); end

outputFile = 'DEBUG_Fixed_COM_KE.mat';
savePath = fullfile(outputFolder, outputFile);

% Save Uav_pert as 'Uav' so the loading script sees the actual used params
Uav = Uav_pert; 
Motor = Motor_pert;

% Create a dummy features_i struct just so the RLS processor doesn't crash
features_i = struct();
features_i.DEBUG_NOTE = 'Manually generated perturbations';

save(savePath, 'simIn', 'simOut', 'Uav', 'Motor', 'features_i', 'B_matrix_nominal');

fprintf('SUCCESS. Debug file saved to:\n%s\n', savePath);
fprintf('Nominal B Matrix (0,0): %f\n', B_matrix_nominal(1,1));

end