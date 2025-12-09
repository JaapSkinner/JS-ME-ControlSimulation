%% Deep Inspection of ULog Structure
clear; clc;

% 1. Load File
[fileName, pathName] = uigetfile({'*.ulg', 'PX4 ULog'}, 'Select Log');
if isequal(fileName, 0); return; end
ulog = ulogreader(fullfile(pathName, fileName));

% 2. Define the Topics we care about
targetTopics = { ...
    'sensor_combined', ...
    'actuator_outputs', ...
    'actuator_motors', ...
    'esc_status', ...
    'vehicle_visual_odometry', ...
    'vehicle_local_position'};

fprintf('\n=== INSPECTING LOG CONTENTS ===\n');

% 3. Iterate and Print Details
availableList = ulog.AvailableTopics.TopicNames;

for i = 1:length(targetTopics)
    name = targetTopics{i};
    fprintf('\n------------------------------------------------\n');
    fprintf('SEARCHING FOR: %s ... ', name);
    
    if any(strcmp(availableList, name))
        fprintf('FOUND.\n');
        
        % Read the data
        try
            dataStruct = readTopicMsgs(ulog, 'TopicNames', {name});
            tt = dataStruct.TopicMessages{1};
            
            % Print Dimensions
            fprintf('  > Rows: %d\n', height(tt));
            
            % Print Variable Names
            fprintf('  > Column Names: ');
            disp(tt.Properties.VariableNames);
            
            % Check for Zeros / Content in first numeric column
            vars = tt.Properties.VariableNames;
            if length(vars) > 1
                firstVar = vars{1}; % Usually 'timestamp'
                secondVar = vars{2}; % First data column
                
                sampleData = tt.(secondVar);
                if isnumeric(sampleData)
                    fprintf('  > Data Check (%s): Mean = %f, Max = %f\n', ...
                        secondVar, mean(mean(sampleData)), max(max(sampleData)));
                    
                    if max(max(abs(sampleData))) == 0
                        fprintf('  > WARNING: THIS TOPIC CONTAINS ALL ZEROS.\n');
                    end
                end
            end
            
        catch ME
            fprintf('  > ERROR READING: %s\n', ME.message);
        end
    else
        fprintf('NOT FOUND.\n');
    end
end
fprintf('\n------------------------------------------------\n');