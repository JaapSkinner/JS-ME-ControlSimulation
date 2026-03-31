%% UAV Mixer Generation & Validation Script
%  1. Loads Simulation Data (UKF and/or RLS).
%  2. Generates Normalized Mixers (PX4 Logic) for both.
%  3. VALIDATION: Plots predicted (B*u) vs Real flight data.
%  4. EXPORT: Saves the generated mixers to a .mat file.
clear; clc; close all;

%% 1. Configuration
% --- PROCESS TOGGLES ---
PROCESS_UKF  = false;   % Generate Mixer from UKF Data?
PROCESS_RLS  = true;   % Generate Mixer from RLS Data?

% --- SAMPLING CONFIGURATION ---
SAMPLE_TIME  = 17.0;   % RLS: Snapshot time (s)
UKF_AVG_WIN  = 3.0;    % UKF: Time window (s) from end to average

% --- VISUALIZATION TOGGLES ---
SHOW_REAL    = true;   
SHOW_RLS     = true;   
SHOW_UKF     = false;
SHOW_NOMINAL = true;  

startPath  = fullfile(pwd, 'Results');

%% 2. Load File (UI)
fprintf('=================================================\n');
fprintf('           DATA SELECTION\n');
fprintf('=================================================\n');

if ~isfolder(startPath), startPath = pwd; end

% 2.1 Load UKF File (Primary Source)
fprintf('Select UKF/Simulation file (Master Data)...\n');
[ukfFile, ukfPath] = uigetfile(fullfile(startPath, '*.mat'), 'Select UKF/Sim File');
if isequal(ukfFile, 0), return; end
ukfFullFile = fullfile(ukfPath, ukfFile);

% 2.2 Load RLS File (If enabled)
rlsFullFile = '';
if PROCESS_RLS
    % Try to find matching file automatically
    sampleIdMatch = regexp(ukfFile, 'Sample\d+', 'match');
    if ~isempty(sampleIdMatch)
        recPattern = ['*' sampleIdMatch{1} '*.mat'];
        fprintf('Select Matching RLS File (%s)...\n', recPattern);
        [rlsFile, rlsPath] = uigetfile(fullfile(ukfPath, recPattern), 'Select RLS File');
    else
        fprintf('Select Matching RLS File...\n');
        [rlsFile, rlsPath] = uigetfile(fullfile(ukfPath, '*.mat'), 'Select RLS File');
    end
    
    if isequal(rlsFile, 0)
        warning('RLS enabled but no file selected. Skipping RLS.');
        PROCESS_RLS = false;
    else
        rlsFullFile = fullfile(rlsPath, rlsFile);
    end
end

%% 3. Process Data & Generate Matrices
fprintf('\nLoading Data...\n');
S_ukf = load(ukfFullFile);

% Validate simOut
if ~isfield(S_ukf, 'simOut') || ~isprop(S_ukf.simOut, 'UKFData')
    error('Variable "simOut.UKFData" not found in UKF file.');
end

ukfData = S_ukf.simOut.UKFData;
N_ROTORS = S_ukf.Uav.N_ROTORS;
B_Matrices = struct(); 

% --- A. Extract UKF Matrix ---
if PROCESS_UKF
    fprintf('Processing UKF Estimation...\n');
    endTime = ukfData.UKF_DATA.Time(end);
    startTime = max(ukfData.UKF_DATA.Time(1), endTime - UKF_AVG_WIN);
    
    % Get subset
    ts_subset = getsampleusingtime(ukfData.UKF_DATA, startTime, endTime);
    
    % 1. Extract ALL parameters (Columns 15 onwards)
    all_params = ts_subset.Data(:, 14 + N_ROTORS : end);
    
    % 2. Average over window
    params_avg = mean(all_params, 1);
    
    % 3. Slice only the B-matrix elements (6 * N_ROTORS)
    idx_B_end = 6 * N_ROTORS;
    if length(params_avg) < idx_B_end
        error('UKF Parameter vector is too short. Expected at least %d elements, got %d.', idx_B_end, length(params_avg));
    end
    B_vec = params_avg(1 : idx_B_end);
    
    % 4. Reshape
    B_Matrices.UKF = reshape(B_vec, [6, N_ROTORS]);
end

% --- B. Extract RLS Matrix ---
if PROCESS_RLS
    fprintf('Processing RLS Estimation...\n');
    S_rls = load(rlsFullFile);
    if ~isprop(S_rls.simOut, 'RLSData'), error('RLSData not found.'); end
    
    rlsData = S_rls.simOut.RLSData;
    ts_f = rlsData.ForceEffectiveness;
    ts_t = rlsData.TorqueEffectiveness;
    
    % Find sample index
    [~, idx] = min(abs(ts_f.Time - SAMPLE_TIME));
    
    F_est = ts_f.Data(:, :, idx);
    T_est = ts_t.Data(:, :, idx);
    
    % Handle dimensions
    if size(F_est, 1) ~= 3, F_est = F_est'; end
    if size(T_est, 1) ~= 3, T_est = T_est'; end
    
    B_Matrices.RLS = [F_est; T_est];
end

% --- C. Nominal Matrix ---
if isfield(S_ukf, 'B_matrix_nominal')
    B_Matrices.Nominal = S_ukf.B_matrix_nominal;
else
    warning('Nominal B Matrix not found in file.');
end

%% 4. Calculate Mixers (PX4 Logic)
normalize_mixer = @(M) perform_normalization(M);
Mixers = struct();
names = fieldnames(B_Matrices);
axis_names = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};

fprintf('\n=================================================\n');
fprintf('       GENERATED MIXERS (Inverse B)\n');
fprintf('=================================================\n');

for i = 1:length(names)
    type = names{i};
    B_curr = B_Matrices.(type);
    
    % 1. Pseudo Inverse
    M_raw = pinv(B_curr);
    
    % 2. Normalize
    [M_norm, ~] = normalize_mixer(M_raw);
    Mixers.(type) = M_norm;
    
    % 3. Print
    fprintf('---> %s Mixer (Normalized) <--- \n', type);
    disp(M_norm);
end

%% 5. Validation: Predict vs Real
fprintf('\nRunning Model Validation...\n');

% Prepare Data
ts_real = ukfData.Real_Wrench;
t_target = ts_real.Time;
Y_real = ts_real.Data; % [Time x 6] usually

ts_omega = ukfData.Omega;
ts_omega_res = resample(ts_omega, t_target);
U_sq = ts_omega_res.Data .^ 2; 

% Ensure dimensions [6 x N] and [N x Time]
if size(Y_real, 2) == 6, Y_real = Y_real'; end
if size(U_sq, 2) == N_ROTORS, U_sq = U_sq'; end

% Predictions & RMSE
Y_preds = struct();
rmse = struct();

for i = 1:length(names)
    type = names{i};
    Y_preds.(type) = B_Matrices.(type) * U_sq;
    err = Y_real - Y_preds.(type);
    rmse.(type) = sqrt(mean(err.^2, 2)); % RMSE per axis
end

% Print RMSE Table
fprintf('\n%-6s', 'Axis');
for i=1:length(names), fprintf(' | %-12s', [names{i} ' RMSE']); end
fprintf('\n-------------------------------------------------\n');
for k = 1:6
    fprintf('%-6s', axis_names{k});
    for i=1:length(names)
        type = names{i};
        fprintf(' | %-12.4f', rmse.(type)(k));
    end
    fprintf('\n');
end

%% 6. Visualization
% A. Validation Plot (3x2 Grid)
figure('Name', 'Model Validation: Prediction vs Reality', 'Color', 'w');
y_labels = {'N', 'N', 'N', 'Nm', 'Nm', 'Nm'};
colors = {'r--', 'b--', 'm--', 'g--'}; 

for k = 1:6
    subplot(2, 3, k);
    hold on;
    if SHOW_REAL, plot(t_target, Y_real(k, :), 'k', 'LineWidth', 1.0, 'DisplayName', 'Real'); end
    
    c_idx = 1;
    for i = 1:length(names)
        type = names{i};
        if strcmp(type, 'Nominal') && ~SHOW_NOMINAL, continue; end
        if strcmp(type, 'UKF') && ~SHOW_UKF, continue; end
        if strcmp(type, 'RLS') && ~SHOW_RLS, continue; end
        
        plot(t_target, Y_preds.(type)(k, :), colors{c_idx}, 'LineWidth', 1.2, ...
             'DisplayName', sprintf('%s (RMSE: %.2f)', type, rmse.(type)(k)));
        c_idx = c_idx + 1;
    end
    title(axis_names{k}); ylabel(y_labels{k}); xlabel('Time (s)'); grid on;
    if k == 1, legend('Location', 'best', 'Interpreter', 'none'); end
end
sgtitle('Validation: Static B vs Full Flight');

% B. Mixer Heatmaps
num_plots = length(fieldnames(Mixers));
if num_plots > 0
    figure('Name', 'Generated Mixers', 'Color', 'w');
    f_names = fieldnames(Mixers);
    for i = 1:num_plots
        type = f_names{i};
        subplot(1, num_plots, i);
        
        M_curr = Mixers.(type);
        imagesc(M_curr); 
        colormap(jet); clim([-1 1]); 
        if i == num_plots, colorbar; end
        
        title([type ' Mixer']); 
        xlabel('Inputs'); ylabel('Motors');
        xticks(1:6); xticklabels(axis_names); 
        yticks(1:N_ROTORS);
        
        % --- Overlay Values (Restored from Original Script) ---
        textStrings = num2str(M_curr(:), '%0.2f');
        textStrings = strtrim(cellstr(textStrings)); 
        [x, y] = meshgrid(1:6, 1:N_ROTORS);
        
        text(x(:), y(:), textStrings(:), 'HorizontalAlignment', 'center', ...
            'Color', 'w', 'FontWeight', 'bold', 'FontSize', 8);
        % ------------------------------------------------------
    end
end

%% 7. Export Results
fprintf('\n=================================================\n');
fprintf('           EXPORTING RESULTS\n');
fprintf('=================================================\n');

% Create Export Directory
saveDir = fullfile(pwd, 'Results','Generated_Mixers');
if ~isfolder(saveDir)
    mkdir(saveDir);
    fprintf('Created directory: %s\n', saveDir);
end

% Construct Filename
[~, fName, ~] = fileparts(ukfFile);
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM');
saveName = sprintf('Mixer_%s_%s.mat', fName, timestamp);
fullSavePath = fullfile(saveDir, saveName);

% Save Data
fprintf('Saving data to: %s ...\n', saveName);
save(fullSavePath, 'Mixers', 'B_Matrices', 'rmse', 'ukfFullFile', 'rlsFullFile');
fprintf('Export Complete.\n');

%% Helper Function
function [M_norm, Scales] = perform_normalization(M_raw)
    M_norm = zeros(size(M_raw));
    Scales = zeros(1, 6);
    for i = 1:6
        col = M_raw(:, i);
        max_val = max(abs(col));
        if max_val > 1e-9
            M_norm(:, i) = col / max_val;
            Scales(i) = max_val;
        else
            M_norm(:, i) = col;
        end
    end
end