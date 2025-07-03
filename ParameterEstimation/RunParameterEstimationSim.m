% RunParameterEstimationSim.m
clear; clc;

% Run the base script to load and initialize everything
run('ParameterEstimationBase.m');
run('mlebusgen.m');
addpath(submodulePath);

%% Parameter Variation
variationPercent = 0;  % percent variation
vary = @(x) x .* (1 + (2*rand(size(x)) - 1) * (variationPercent / 100));
% MOTOR VARIATION
Motor.K           = vary(Motor.K);
Motor.K_T         = vary(Motor.K_T);
Motor.K_V         = vary(Motor.K_V);
Motor.K_E         = vary(Motor.K_E);
Motor.C_TAU       = vary(Motor.C_TAU);
Motor.I_R         = vary(Motor.I_R);
Motor.I_R_ZZ      = vary(Motor.I_R_ZZ);
Motor.I_Rv        = vary(Motor.I_Rv);
Motor.B           = vary(Motor.B);
Motor.Volt_offset = vary(Motor.Volt_offset);
Motor.volt_slope  = vary(Motor.volt_slope);
Motor.pwm_min     = vary(Motor.pwm_min);
Motor.pwm_max     = vary(Motor.pwm_max);
Motor.R           = vary(Motor.R);
Motor.V_0         = vary(Motor.V_0);
Motor.I_0         = vary(Motor.I_0);

% UAV VARIATION

Uav.D_UAV               = vary(Uav.D_UAV);           % Drag coefficient UAV body
Uav.D_PROP              = vary(Uav.D_PROP);          % Drag coefficient propeller
Uav.RotorTiltDeg        = vary(Uav.RotorTiltDeg);    % Rotor tilt angle, affects thrust direction
Uav.MotorLoc            = vary(Uav.MotorLoc);        % Rotor positions relative to body
Uav.M                   = vary(Uav.M);               % Mass
Uav.I                   = vary(Uav.I);               % Inertia matrix entries (if stored as vector/array)
Uav.NOMINAL_BATTERY_VOLTAGE = vary(Uav.NOMINAL_BATTERY_VOLTAGE);
Uav.ROTOR_DIRECTION     = Uav.ROTOR_DIRECTION;       % Usually ±1, no variation unless fault sim
Uav.RHO_AIR             = vary(Uav.RHO_AIR);         % Air density

Uav.R_PROP              = vary(Uav.R_PROP);          % Prop radius
Uav.A_UAV               = vary(Uav.A_UAV);           % UAV frontal area
Uav.A_PROP              = vary(Uav.A_PROP);          % Prop area
Uav.ZETA                = vary(Uav.ZETA);            % Damping factor (if used)

% AERO VARIATION

aeroFields = {'Cz2P', 'Cz3P', 'Cx2P', 'CM2P', 'Cz1P', 'Cz1B', 'Cx1B', 'Cx1P', 'CM1P', 'CM1B'};

for i = 1:numel(aeroFields)
    fld = aeroFields{i};
    % Vary only the coefficients, keep inputs and equation intact
    Aero.(fld).coefs = vary(Aero.(fld).coefs);
end




%% Create test tag and file name
testCase = 'estimation';
tStr = datestr(now,'yyyy-mm-dd_HH-MM-SS');
outputFolder = fullfile(projectRoot, 'Results', 'ParameterEstimation');
outputFile = sprintf('%s_%s_%s', testCase, uavType, tStr);




%% Deactivate initial states
set_param(modelName, 'LoadInitialState', 'off' );
set_param( modelName, 'LoadExternalInput', 'off' );

% Mean wind for log
UMean = mean(windInput.Data(:,1));





%% 4) CUSTOM CODE
% pUT ANY SPECIAL CONDITIONS FOR YOUR SIMULATION, SUCH AS POSITION OR
% ATTITUDE SETPOINTS, HERE...








%% RUN SIMULATION
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setVariable('windInput', windInput);
simIn = simIn.setVariable('uavType', uavType);
simIn = simIn.setVariable('windFile', windFile);
simIn = simIn.setVariable('Simulation', Simulation);
simIn = simIn.setVariable('Uav', Uav);
simIn = simIn.setVariable('Motor', Motor);
simIn = simIn.setVariable('Initial', Initial);

disp('Simulation runs:')
fprintf('\t[%d/%d] wind file: % 18s, Umean = %5.2f m/s\n', 1, 1, windFile, UMean);

simOut = sim(simIn);

if ~isfolder(outputFolder)
    mkdir(outputFolder)
end
save(fullfile(outputFolder, outputFile), 'simIn', 'simOut', 'Uav', 'Motor', 'windInput', 'windFile');
