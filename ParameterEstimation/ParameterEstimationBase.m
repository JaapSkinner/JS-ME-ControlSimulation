% Base file for all parameter estimation simulations
clear; clc;

% Get project root
proj = matlab.project.currentProject;
projectRoot = proj.RootFolder;
submodulePath = fullfile(projectRoot, 'Simulink_Flight_Simulation', 'Multirotor with PX4 2023');
currentPath = fullfile(projectRoot, 'ParameterEstimation');
modelDataFile = fullfile(projectRoot, 'ParameterEstimation', "Octo_CantedStacked_Flamingo.mat");
windFile = fullfile(projectRoot, 'ParameterEstimation', 'windData.mat');
%% Load Parameters
paramsPath = fullfile(projectRoot, 'ParameterEstimation', 'parameters.m');
run(paramsPath);
%% Init UAV
cd(submodulePath);

% Airframe
load(modelDataFile);
[Uav, Motor, Aero, Initial] = InitializeParametersUAV(Uav, Motor, Aero);
uavType = Uav.uavType;

% Wind
load(windFile);
windInput = timeseries(windData(:,2:4), windData(:,1));
dt = mean(gradient(windInput.Time));
t_init = 0:dt:30;
vel_init = [linspace(0,windInput.Data(1,1),length(t_init))', ...
            linspace(0,windInput.Data(1,2),length(t_init))', ...
            linspace(0,windInput.Data(1,3),length(t_init))'];
initInput = timeseries(vel_init, t_init);
windInput.Time = windInput.Time + t_init(end) + dt;
windInput = append(initInput, windInput);

% Load Simulink model
simModelPath = fullfile(projectRoot, 'Simulink_Flight_Simulation');
addpath(simModelPath);
modelName = 'MultirotorSimPx4.slx';
load_system(modelName);

% Set submodules
set_param([modelName(1:end-4) '/Drag model'], 'ModelName', 'DragModelAIAAv3');
set_param([modelName(1:end-4) '/Motor model'], 'ModelName', 'MotorModelZJChen');

% Setup mixer
Motor.CommandMixing = [ Uav.N_ROTORS 10000 10000 10000 0 ];
set_param([modelName(1:end-4) '/Mixer'], 'rotor_count', "Uav.N_ROTORS");
set_param([modelName(1:end-4) '/Mixer'], 'MotorMixer', "Motor.mixingMatrix");
set_param([modelName(1:end-4) '/Mixer'], 'command_mixing', "Motor.CommandMixing");

% Mask parameters
UseWindProfile(modelName(1:end-4), true);
UseEstimators(modelName(1:end-4), true);
UsePositionController(modelName(1:end-4), true);

% Initialize simulation
tEnd = 300;projectRoot
Simulation = InitializeModel(modelName(1:end-4), Initial, tEnd);

cd(currenPath);