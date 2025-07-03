%% Base file for all parameter estimation simulations
%% Get project root
proj = matlab.project.currentProject;
projectRoot = proj.RootFolder;
submodulePath = fullfile(projectRoot, 'Simulink_Flight_Simulation', 'Multirotor with PX4 2023');
currentPath = fullfile(projectRoot, 'ParameterEstimation');
modelDataFile = fullfile(projectRoot, 'ParameterEstimation', "Octo_CantedStacked_Flamingo.mat");
windFile = fullfile(projectRoot, 'ParameterEstimation', 'windData.mat');

addpath(genpath(submodulePath))


%% Load Parameters
paramsPath = fullfile(projectRoot, 'ParameterEstimation', 'parameters.m');
run(paramsPath);
%% Init UAV

% Airframe
load(modelDataFile);
[Uav, Motor, Aero, Initial] = InitializeParametersUAV(Uav, Motor, Aero);
uavType = Uav.uavType;

% Wind
load(windFile);

if exist('windData','var')
windInput = timeseries(windData(:,2:4),windData(:,1));
end
tEnd = 300;
%Add initialisation period (30s) for the simulation
dt = mean(gradient(windInput.time));        %Mean sim timestep  
t_init = 0:dt:10;                           %time array for the new data
%Create linear init ramp
vel_init = [linspace(0,windInput.Data(1,1),length(t_init))',...
    linspace(0,windInput.Data(1,2),length(t_init))',...
    linspace(0,windInput.Data(1,3),length(t_init))'];
initInput = timeseries(vel_init,t_init);    %Init timeseries
% Add to the time for the main input
windInput.Time = windInput.Time + t_init(end) + dt;
%Concatenate wind files
windInput = append(initInput,windInput);

%% Load Simulink model
simModelPath = fullfile(projectRoot, 'Simulink_Flight_Simulation');
addpath(simModelPath);
modelName = 'MultirotorSimPx4';
load_system([modelName '.slx']);

%% Set submodules
set_param([modelName '/Drag model'], 'ModelName', 'DragModelAIAAv3');
set_param([modelName '/Motor model'], 'ModelName', 'MotorModelZJChen');

%% Setup mixer
Motor.CommandMixing = [ Uav.N_ROTORS 10000 10000 10000 0 ];
set_param([modelName '/Mixer'], 'rotor_count', "Uav.N_ROTORS");
set_param([modelName '/Mixer'], 'MotorMixer', "Motor.mixingMatrix");
set_param([modelName '/Mixer'], 'command_mixing', "Motor.CommandMixing");

%% Mask parameters
UseWindProfile(modelName, true);
UseEstimators(modelName, true);
UsePositionController(modelName, true);

%% Initialize simulation
tEnd = 300;
Simulation = InitializeModel(modelName, Initial, tEnd);