function run_RLS_replay(sampleIndex)
%% RLS Replay Execution Function
% Runs the RLS Estimator against Real Flight Data.
% UPDATED: Supports new ulog_to_mat structure (Body Vel, RPM, RPY).
%
% Usage: run_RLS_replay(0) % Run with nominal params
%        run_RLS_replay(1) % Run with Parameter Set 1

    %% 1. Project & Path Setup
    try; proiwthoutj = currentProject; projectRoot = proj.RootFolder; catch; projectRoot = pwd; end
    
    % Define paths
    modelName = 'MultirotorSimRLSEXP'; 
    modelDataFile = fullfile(projectRoot, 'ParameterEstimation', 'Octo_CantedStacked_Flamingo.mat');
    
    % Add paths
    addpath(genpath(fullfile(projectRoot, 'Simulink_Flight_Simulation')));
    addpath(genpath(fullfile(projectRoot, 'ParameterEstimation')));

    %% 2. Initialize Vehicle Parameters (Nominal)
    if isfile(modelDataFile)
        load(modelDataFile); % Loads Uav, Motor, Aero, Initial
    else
        error('Octo_CantedStacked_Flamingo.mat not found. Cannot initialize vehicle parameters.');
    end
    
    % Run Standard Initialization
    [Uav, Motor, Aero, Initial] = InitializeParametersUAV(Uav, Motor, Aero);
    
    %% 3. (Optional) Load Perturbed Parameters
    if nargin > 0 && sampleIndex > 0
        paramFile = fullfile(projectRoot, 'ParameterEstimation', 'ParameterSet', ...
            sprintf('ParamSet_%03d.mat', sampleIndex));
        
        if isfile(paramFile)
            fprintf('Loading Perturbed Parameters: %s\n', paramFile);
            ptb = load(paramFile);
            Uav = ptb.Uav;
            Motor = ptb.Motor;
            [Uav, Motor, Aero, Initial] = InitializeParametersUAV(Uav, Motor, Aero);
        else
            warning('Parameter set %d not found. Using Nominal.', sampleIndex);
        end
    end

    %% 4. Initialize RLS (Calculate Guesses)
    fprintf('Initializing RLS Estimator...\n');
    run('InitRLSEXP.m'); 
    
    if ~exist('B_matrix_nominal', 'var')
        warning('B_matrix_nominal missing from workspace. Checking InitRLSEXP logic...');
    end

    %% 5. Load Replay Data
    fprintf('Loading Flight Data...\n');
    run('setup_replay_data.m'); 
    
    % --- DERIVED DATA (Truth Validation Only) ---
    % Create Truth Velocity (NED) if missing (Derived from Pos Truth)
    % (Useful for comparing against Estimator outputs in scopes)
    if exist('ts_pos_truth', 'var')
        pos_data = ts_pos_truth.Data;
        time_vec = ts_pos_truth.Time;
        vel_data = zeros(size(pos_data));
        
        % Simple finite difference
        dt_vals = [diff(time_vec); 0.004]; dt_vals(dt_vals==0) = 0.004;
        for i=1:3
            vel_data(:,i) = gradient(pos_data(:,i)) ./ dt_vals;
        end
        ts_vel_ned = timeseries(vel_data, time_vec, 'Name', 'Vel_Truth_NED');
    else
        ts_vel_ned = timeseries(zeros(length(t),3), t, 'Name', 'Vel_Truth_NED');
    end

    %% 6. Simulation Configuration
    outputFolder = fullfile(projectRoot, 'Results', 'ReplayRLS');
    if ~isfolder(outputFolder); mkdir(outputFolder); end
    
    tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
    outputFile = sprintf('RLS_Replay_Log_%s', tStr);
    
    simIn = Simulink.SimulationInput(modelName);
    
    % --- A. Vehicle Params ---
    simIn = simIn.setVariable('Uav', Uav);
    simIn = simIn.setVariable('Motor', Motor);
    
    % --- B. RLS Specifics ---
    simIn = simIn.setVariable('InitialRLS', RLS_INITIAL_GUESS);
    simIn = simIn.setVariable('B_matrix_nominal', B_matrix_nominal);
    
    % --- C. Filter Coefficients ---
    simIn = simIn.setVariable('diff_num', diff_num);
    simIn = simIn.setVariable('diff_den', diff_den);
    
    % --- D. Actuators & Sensors (MAPPING NEW -> OLD NAMES) ---
    simIn = simIn.setVariable('ts_u_cmd',       ts_u_cmd);      % Motor Commands [0-1]
    simIn = simIn.setVariable('ts_omega_rpm',   ts_rpm);        % Motor RPM (Mapped from ts_rpm)
    
    simIn = simIn.setVariable('ts_gyro_body',   ts_omega);      % IMU Gyro (Mapped from ts_omega)
    simIn = simIn.setVariable('ts_accel_body',  ts_accel);      % IMU Accel (Mapped from ts_accel)
    
    % --- E. Ground Truth (NED) ---
    simIn = simIn.setVariable('ts_pos_ned',     ts_pos_truth);  % Mocap Position
    simIn = simIn.setVariable('ts_vel_ned',     ts_vel_ned);    % Mocap Velocity (Derived above)
    simIn = simIn.setVariable('ts_eta',         ts_rpy);        % Mocap Euler Angles (Mapped from ts_rpy)
    simIn = simIn.setVariable('ts_quat',        ts_quat);       % Mocap Quaternion
    
    % --- F. PX4 Estimates (BODY DATA) ---
    simIn = simIn.setVariable('ts_est_vel_body',  ts_est_vel_body);
    simIn = simIn.setVariable('ts_est_acc_body',  ts_est_acc_body);
    simIn = simIn.setVariable('ts_est_ang_accel', ts_est_ang_accel);
    simIn = simIn.setVariable('ts_est_ang_vel',   ts_est_ang_vel); 
    simIn = simIn.setVariable('ts_est_ang', ts_est_ang);

    % --- G. Solver Config ---
    simIn = simIn.setModelParameter('StopTime', string(sim_duration));
    simIn = simIn.setModelParameter('SolverType', 'Fixed-step');
    simIn = simIn.setModelParameter('FixedStep', string(replay_dt));
    
    %% 7. Run & Save
    fprintf('Running Replay Simulation (Duration: %.1fs)...\n', sim_duration);
    simOut = sim(simIn);
    
    save(fullfile(outputFolder, outputFile), 'simOut', 'Uav', 'Motor', 'RLS_INITIAL_GUESS');
    fprintf('Replay Complete. Results saved to: %s\n', outputFile);
end