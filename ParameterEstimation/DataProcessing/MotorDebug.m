%% UKF X-RAY INSPECTOR
% Run this on your UKF file. It prints the raw state vector structure.
clear; clc;

% 1. Load
try
    proj = matlab.project.rootProject();
    startPath = fullfile(proj.RootFolder, 'Results', 'ParameterEstimation');
    if ~isfolder(startPath), startPath = pwd; end
catch
    startPath = pwd;
end
[ukfFile, ukfPath] = uigetfile(fullfile(startPath, 'estimation_*.mat'), 'Select UKF File');
S_ukf = load(fullfile(ukfPath, ukfFile));

% 2. Get Final State Vector
ukfData = S_ukf.simOut.UKFData.UKF_DATA;
final_vec = ukfData.Data(end, :); % The very last recorded instant
N = S_ukf.Uav.N_ROTORS;
Len = length(final_vec);

fprintf('--- FILE X-RAY ---\n');
fprintf('Total State Length: %d\n', Len);
fprintf('N Rotors: %d\n', N);

% 3. Print the "Tail" of the vector (Last 4*N + 5 items)
% We expect params to be here.
start_print = Len - (4*N) - 2; 

fprintf('\n--- RAW VALUES (Index %d to End) ---\n', start_print);
fprintf('Expected Pattern: [Gain... | Cw... | Cw2... | Bias...]\n\n');

for i = start_print : Len
    val = final_vec(i);
    
    % Try to guess what it is based on magnitude
    tag = '';
    if abs(val) > 1000, tag = '(Likely GAIN)';
    elseif abs(val) > 10, tag = '(Likely BIAS?)';
    elseif abs(val) > 0.0001, tag = '(Likely Coeff)';
    elseif val == 0, tag = '(ZERO)';
    end
    
    fprintf('Idx %d: %12.4f  %s\n', i, val, tag);
    
    % Add separator lines between blocks of N
    dist_from_end = Len - i;
    if mod(dist_from_end, N) == 0 && dist_from_end > 0
        fprintf('----------------------------------\n');
    end
end