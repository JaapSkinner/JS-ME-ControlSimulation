%% GENERATE PARAMETER SETS (Monte Carlo Pre-Generation)
% This script generates N distinct sets of perturbed Uav/Motor parameters
% and saves them to a folder. This ensures every simulation run is
% reproducible.

clear; clc; close all;

%% --- 1. USER CONFIGURATION ----------------------------------------------

% Output Settings
NUM_SAMPLES = 1;                % How many datasets to generate
run('RLSBaseHEX.m'); %must be here because it gets projectRoot
Uav.COM = [0 0 0];
OUTPUT_DIR  = fullfile(projectRoot, 'ParameterEstimation', 'ParameterSetPHEX');
FILE_PREFIX = 'ParamSet_';       % Filename format (e.g., ParamSet_001.mat)

% Distribution Configuration
% Format: {Struct, Field, Distribution, Value}
%   - Value: For 'normal'/'uniform', this is Percentage (e.g., 3.0 = 3%).
%            For COM, this is Absolute Meters (e.g., [0.001, 0.001, 0.03]).
VARIATION_CONFIG = {
    % Name     Field          Dist       Value (%, or Abs for COM)
    'Motor',   'K_V',         'normal',  3.0;
    'Motor',   'K_E',         'normal',  3.0;
    'Motor',   'C_TAU',       'normal',  3.0;
    'Motor',   'B',           'normal',  1.0;
    'Motor',   'Volt_offset', 'uniform', 2.0;
    'Motor',   'volt_slope',  'uniform', 2.0;
    'Motor',   'R',           'normal',  5.0;
    'Motor',   'I_0',         'normal',  10.0;
    'Uav',     'D_UAV',       'normal',  0.5;
    'Uav',     'D_PROP',      'normal',  1.0;
    'Uav',     'M',           'normal',  2.0;
    'Uav',     'I',           'normal',  5.0;
    'Uav',     'RHO_AIR',     'normal',  10.0;
    'Uav',     'R_PROP',      'normal',  1.0;
    'Uav',     'A_UAV',       'normal',  0.5;
    'Uav',     'A_PROP',      'normal',  1.0;
    'Uav',     'ZETA',        'normal',  5.0;
    'Uav',     'COM',         'normal',  [0.001, 0.001, 0.03]; % [X Y Z] Absolute Sigma (m)
};

%% --- 2. INITIALIZE BASELINE (NOMINAL) -----------------------------------
fprintf('Initializing Nominal Plant and B Matrix...\n');

% Run standard initialization to get Uav, Motor, and B_matrix_nominal
try
    run('InitUKFHEX.m'); 
    run('InitRLSHEX.m');
    % run('mlebusgen.m'); % Run if needed to finalize structures
catch
    error('Could not run initialization scripts. Check path.');
end

% Capture the "Gold Standard" variables
Motor_nom = Motor;
Uav_nom = Uav;

if ~exist('B_matrix_nominal', 'var')
    warning('B_matrix_nominal not found. Calculating defaults...');
    % Insert manual calculation here if needed, or ensure InitRLS provides it
end

B_matrix_nominal_TRUE = B_matrix_nominal; % Save this to inject into every file

fprintf('Nominal Initialization Complete.\n');
fprintf('Nominal B Matrix (1,1): %f\n', B_matrix_nominal_TRUE(1,1));

%% --- 3. PREPARE OUTPUT --------------------------------------------------
if ~isfolder(OUTPUT_DIR)
    mkdir(OUTPUT_DIR);
    fprintf('Created directory: %s\n', OUTPUT_DIR);
else
    fprintf('Output directory: %s\n', OUTPUT_DIR);
end

%% --- 4. GENERATION LOOP -------------------------------------------------
fprintf('Generating %d parameter sets...\n', NUM_SAMPLES);

rng('shuffle'); % Seed the generator

for k = 1:NUM_SAMPLES
    
    % A. Call the Sampling Function
    [Motor_i, Uav_i, features_i] = localSampleParameters(Motor_nom, Uav_nom, VARIATION_CONFIG, Uav_nom.N_ROTORS);
    
    % B. Prepare Structure for Saving
    % We save Uav/Motor as the PERTURBED versions
    Uav = Uav_i;
    Motor = Motor_i;
    
    % We save B_matrix_nominal as the CLEAN version (Crucial for your RLS check)
    B_matrix_nominal = B_matrix_nominal_TRUE;
    
    % C. Save to File
    fName = sprintf('%s%03d.mat', FILE_PREFIX, k);
    fPath = fullfile(OUTPUT_DIR, fName);
    
    % Save minimal necessary data to reconstruct the plant
    save(fPath, 'Uav', 'Motor', 'features_i', 'B_matrix_nominal');
    
    if mod(k, 10) == 0
        fprintf('  Generated set %d / %d\n', k, NUM_SAMPLES);
    end
end

fprintf('Done. All parameter sets saved.\n');


%% --- LOCAL FUNCTIONS ----------------------------------------------------

function [Motor, Uav, variationFeatures] = localSampleParameters(Motor_nom, Uav_nom, configCell, N_motors)
    % Local adaptation of your sampleParameters function that takes the
    % configuration cell array instead of parallel arrays.
    
    Motor = Motor_nom;
    Uav = Uav_nom;
    variationFeatures = struct();
    
    perMotorParams = ["K_V", "K_E", "C_TAU", "B", "Volt_offset", "volt_slope", "R", "I_0", "D_UAV", "D_PROP"];
    
    numParams = size(configCell, 1);
    
    for i = 1:numParams
        structName = configCell{i, 1}; % 'Motor' or 'Uav'
        fieldName  = configCell{i, 2}; % 'K_V', etc.
        distType   = configCell{i, 3}; % 'normal' or 'uniform'
        valueSpec  = configCell{i, 4}; % Percent or Absolute Array
        
        % Get Nominal Value
        nominal = eval([structName '_nom.' fieldName]);
        
        % Handle COM (Vector) vs Scalar
        if strcmp(fieldName, 'COM')
            % valueSpec is [sigmaX, sigmaY, sigmaZ] in meters
            sigma = valueSpec; 
            if length(sigma) ~= 3, sigma = [sigma(1) sigma(1) sigma(1)]; end
            
            % Force row vector shape
            if size(nominal,1) > size(nominal,2), nominal = nominal'; end 
            if size(sigma,1) > size(sigma,2), sigma = sigma'; end 
            
            switch distType
                case 'normal'
                    sampled = nominal + sigma .* randn(size(nominal));
                case 'uniform'
                    % Uniform +/- sigma
                    sampled = nominal + (2 * rand(size(nominal)) - 1) .* sigma;
            end
            
            % Update Struct
            Uav.COM = sampled;
            
            % --- SPECIAL HANDLING: Update Motor Locations based on COM shift ---
            delta = sampled - nominal;
            
            % Shift motor positions (If COM moves +X, Motors move -X relative to COM)
            Uav.MotorLoc(:,1:3) = Uav.MotorLoc(:,1:3) - repmat(delta, N_motors, 1);
            % Recalculate radial distance (4th column)
            Uav.MotorLoc(:,4) = sqrt(sum(Uav.MotorLoc(:,1:3).^2, 2));
            
            % Log Features
            v = struct();
            v.per_axis_deviation = delta;
            variationFeatures.(fieldName) = v;
            
        else
            % STANDARD SCALAR PARAMETERS (Percentage based)
            percent = valueSpec / 100;
            sigma = abs(nominal) * percent;
            
            switch distType
                case 'normal'
                    sampled = nominal + sigma .* randn(size(nominal));
                case 'uniform'
                    sampled = nominal + (2 * rand(size(nominal)) - 1) .* sigma;
            end
            
            % Assign back to struct
            eval([structName '.' fieldName ' = sampled;']);
            
            % Log Features
            delta = sampled - nominal;
            v = struct();
            v.mean_deviation = mean(abs(delta(:)));
            v.max_deviation = max(abs(delta(:)));
            
            if any(perMotorParams == fieldName)
                v.std_across_motors = std(sampled(:));
            elseif strcmp(fieldName, 'I')
                v.per_axis_deviation = diag(delta)';
            end
            variationFeatures.(fieldName) = v;
        end
    end
end