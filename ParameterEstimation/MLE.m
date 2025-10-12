% Maximum Likelyhood Estimation of physical parameters from flight data
%% Initialize Environment
clear; clc;
global rotors; % Define rotors as a global variable

% Define the physical layout of the rotors.
% YOU MUST MODIFY THIS FUNCTION with the correct geometry for your UAV.
defineRotorGeometry(); 

%% Load data and form structures
data = load('MLE-Simout.mat');  % replace with actual file
fields = fieldnames(data.data);
for i = 1:numel(fields)
    assignin('base', fields{i}, data.data.(fields{i}));
end

% Structure the measurement set Z (pose + optional rotor speeds)
Z = struct();
Z.t = timevec.Data;                      % timestamps
dt_seq = diff(Z.t);
Z.p = xi.Data;                       % position
Z.q = q.Data;                      % quaternion orientation
Z.n = rotorSpeeds.Data;

% Structure input set U (control + inertial)
U = struct();
U.t = timevec.Data;
pwm_squeezed = squeeze(pwm.Data);
U.uM = pwm_squeezed';                   % motor commands
U.uI = [nuBodyMeasured.Data, VBodyDot.Data];           % IMU readings: [ω a]

%% Set initial guesses for state trajectory and parameters

% Number of time steps
K = length(Z.t);
numMotors = size(U.uM, 2);

% Preallocate state trajectory
Xbar = repmat(createEmptyState(numMotors), K, 1);
for k = 1:K
    Xbar(k).p = Z.p(k, :)';         % position from Vicon
    Xbar(k).q = Z.q(k, :)';         % orientation quaternion
    % v, omega, biases, n are initialized to zeros by createEmptyState
end

% Initial parameter guesses
thetaBar = createEmptyParams();
cT_guess = 1e-6; % An initial guess for the thrust magnitude
cM_guess = 1e-7; % An initial guess for the moment magnitude (z-axis)

thetaBar.J = [5e-3; 5e-3; 1e-2];   % guess: diagonal inertia
thetaBar.rBC = [0; 0; 0];          % IMU at CoG initially

for i = 1:numMotors
    % Guess: All thrust is purely along the rotor's +z axis
    thetaBar.gamma_T(:, i) = [0; 0; cT_guess];
    
    % Guess: All moment is purely around the rotor's +z axis
    % Note: The sign (+/-) depends on rotor spin direction.
    % You might need to make some negative.
    thetaBar.gamma_M(:, i) = [0; 0; cM_guess];
end
% Example: If odd motors are CCW (+) and even are CW (-)
% for i = 1:numMotors
%     if mod(i, 2) == 0 % Even motor (e.g., CW)
%         thetaBar.gamma_M(3, i) = -cM_guess;
%     else % Odd motor (e.g., CCW)
%         thetaBar.gamma_M(3, i) = cM_guess;
%     end
% end

%% Measuement Covariances
R_pos = 1e-4 * eye(3);   % Position noise: small variance
R_quat = 1e-4 * eye(3);  % Quaternion (rotation vector) noise
nx = length(getStateVector(Xbar(1)));  % state vector dimension
nx_minimal = 20; % The update vector size (3+3+3+3+8) since we use dtheta not quaterion (transformed later)
num_mav_residuals = 3 + 3 + 3 + 3 + numMotors;
Qm = 1e-3 * eye(num_mav_residuals);   % MAV dynamics process noise
Qi = 1e-3 * eye(nx);   % IMU dynamics process noise


%% Run MLE until convergance
max_iters = 10;
tol = 1e-6;
cost_prev = Inf;

fprintf("Starting MLE Optimization...\n");
for iter = 1:max_iters
    fprintf("Iteration %d...\n", iter);
    
    % 1. Assemble residuals + Jacobians
    [residuals, A, state_idx, param_idx] = buildBatchSystem(Xbar, thetaBar, Z, U, dt_seq, R_pos, R_quat, Qm, Qi);
    
    % 2. Solve least squares: A * delta = residuals
    delta = A \ residuals;
    
    % 3. Update states
    for k = 1:K
        idx = state_idx(k,:);
        % check if the index is valid before updating
        if all(idx > 0)
            dx = delta(idx);
            Xbar(k) = applyStateDelta(Xbar(k), dx);
        end
    end
    
    % 4. Update parameters
    d_theta = delta(param_idx);
    thetaBar = setParamVector(getParamVector(thetaBar) + d_theta);

    % Sanitize the parameters to ensure they are physically valid.
    thetaBar = sanitizeParameters(thetaBar);
    
    % 5. Check convergence
    cost = norm(residuals);
    fprintf("Cost: %.6f\n", cost);
    if abs(cost_prev - cost) < tol
        fprintf("Converged at iteration %d with cost %.6f\n", iter, cost);
        break;
    end
    cost_prev = cost;
end

disp("Estimated Parameters:");
disp(thetaBar);

%% Helper Functions

% --- Rotor Geometry Definition ---
function defineRotorGeometry()
    global rotors;

    % --- X8 Octorotor Configuration Example ---
    % YOU MUST MODIFY THESE VALUES FOR YOUR SPECIFIC UAV

    % Distance from center to a motor arm
    d = 0.35; % example: 350mm from center
    
    % Vertical distance between top and bottom motors
    h = 0.05; % example: 5cm separation
    
    pos = d / sqrt(2);

    % Position vectors [x; y; z] from CoG to each rotor hub
    % Top Rotors (e.g., z is negative if CoG is frame origin and top is -z)
    rotors(1).r = [ pos;  pos; -h/2]; % Top-Front-Right
    rotors(2).r = [ pos; -pos; -h/2]; % Top-Back-Right
    rotors(3).r = [-pos; -pos; -h/2]; % Top-Back-Left
    rotors(4).r = [-pos;  pos; -h/2]; % Top-Front-Left
    
    % Bottom Rotors
    rotors(5).r = [ pos;  pos; h/2]; % Bottom-Front-Right
    rotors(6).r = [ pos; -pos; h/2]; % Bottom-Back-Right
    rotors(7).r = [-pos; -pos; h/2]; % Bottom-Back-Left
    rotors(8).r = [-pos;  pos; h/2]; % Bottom-Front-Left

    % Thrust direction (assuming no tilt, all thrust along body z-axis)
    % For fixed-tilt, you would modify the rotation matrix R accordingly.
    for i = 1:8
        rotors(i).R = eye(3); 
    end
end


% --- State and Parameter Struct Definitions ---
function x = createEmptyState(numMotors)
    x = struct();
    x.p = zeros(3,1);       % position
    x.v = zeros(3,1);       % velocity
    x.q = [1; 0; 0; 0];     % quaternion
    x.omega = zeros(3,1);   % angular velocity
    x.n = zeros(numMotors,1); % rotor speeds
end


function theta = createEmptyParams()
    numMotors = 8;
    theta = struct();
    theta.J = zeros(3,1);
    theta.rBC = zeros(3,1);
    theta.gamma_T = zeros(3, numMotors);
    theta.gamma_M = zeros(3, numMotors);
    % ADDED: IMU bias parameters
    theta.bw = zeros(3,1);
    theta.ba = zeros(3,1);
end

% --- State and Parameter Vector/Struct Conversion ---
function x_vec = getStateVector(x)
    x_vec = [x.p; x.v; x.q; x.omega; x.n];
end

function x = setStateVector(x_template, x_vec)
    x = x_template;
    idx = 1;
    x.p = x_vec(idx:idx+2); idx = idx+3;
    x.v = x_vec(idx:idx+2); idx = idx+3;
    x.q = x_vec(idx:idx+3); idx = idx+4;
    x.omega = x_vec(idx:idx+2); idx = idx+3;
    x.n = x_vec(idx:end);
end

function theta_vec = getParamVector(theta)
    % Flatten vector 
    theta_vec = [theta.J;
                 theta.rBC;
                 theta.gamma_T(:);
                 theta.gamma_M(:);
                 theta.bw; % ADDED
                 theta.ba]; % ADDED
end

function theta = setParamVector(theta_vec)
    numMotors = 8;
    theta = createEmptyParams();

    idx = 1;
    theta.J = theta_vec(idx:idx+2); idx = idx+3;
    theta.rBC = theta_vec(idx:idx+2); idx = idx+3;
    len_gamma = 3 * numMotors;
    theta.gamma_T = reshape(theta_vec(idx:idx+len_gamma-1), 3, numMotors);
    idx = idx + len_gamma;
    theta.gamma_M = reshape(theta_vec(idx:idx+len_gamma-1), 3, numMotors);
    idx = idx + len_gamma;
    theta.bw = theta_vec(idx:idx+2); idx = idx+3; % ADDED
    theta.ba = theta_vec(idx:idx+2); idx = idx+3; % ADDED
end

% --- State Update Functions ---
function x_new = applyStateDelta(x, dx)
    x_new = x;
    x_new.p = x.p + dx(1:3);
    x_new.v = x.v + dx(4:6);
    dtheta = dx(7:9);
    dq = [1; 0.5*dtheta];
    x_new.q = quatnormalize(quatmultiply(x.q', dq'))';
    x_new.omega = x.omega + dx(10:12);
    x_new.n = x.n + dx(13:end);
end


% ** NEWLY IMPLEMENTED FUNCTION **
function x_out = addState(x_in, dx_struct)
    % Adds a state derivative struct (dx_struct) to a state struct (x_in)
    x_out = x_in;
    x_out.p = x_in.p + dx_struct.p;
    x_out.v = x_in.v + dx_struct.v;
    % For quaternion, we add the derivative and then re-normalize
    x_out.q = x_in.q + dx_struct.q;
    x_out.q = x_out.q / norm(x_out.q); % Normalize
    x_out.omega = x_in.omega + dx_struct.omega;
    x_out.n = x_in.n + dx_struct.n;
end

% --- Residual Calculation Functions ---
function dq = quatError(q_est, q_meas)
    q_est_row = q_est(:)';   
    q_meas_row = q_meas(:)'; 
    q_err = quatmultiply(quatconj(q_est_row), q_meas_row);
    dq = 2 * q_err(2:4)';
end

function [res, R] = measurementResidual(xk, zk, R_pos, R_quat)
    dp = zk.p' - xk.p;
    dq = quatError(xk.q, zk.q);
    res = [dp; dq];
    R = blkdiag(R_pos, R_quat);
end

function dchi = stateResidual(xk, xk_pred)
    dp = xk.p - xk_pred.p;
    dv = xk.v - xk_pred.v;
    dq = quatError(xk_pred.q, xk.q);
    domega = xk.omega - xk_pred.omega;
    dn = xk.n - xk_pred.n;
    dchi = [dp; dv; dq; domega; dn];
end

% --- Dynamics and Integration ---
function dx = mavDynamics(x, uM, theta)
    global rotors; % Access the global rotor geometry
    m = 1;  
    J = diag(theta.J);
    rBC = theta.rBC;
    
    C_IB = quat2rotm(x.q');
    
    % Angular acceleration (calculated earlier to be used in dv)
    [F_tot, M_tot] = rotorForcesAndMoments(uM, x.n, x, theta);
    domega = J \ (M_tot - cross(x.omega, J * x.omega)); % Euler's equation
    
    % Position derivative
    dp = C_IB * x.v;
    
    % Velocity derivative -- CORRECTED
    g_W = [0; 0; -9.81]; % Gravity in world frame
    
    % The following line is the complete equation for linear acceleration
    % of the CoG, accounting for the IMU offset rBC.
    dv = (1/m) * F_tot + C_IB' * g_W - cross(x.omega, x.v) ...
         - cross(domega, rBC) - cross(x.omega, cross(x.omega, rBC));
    
    % Quaternion derivative
    Omega = [0 -x.omega'; x.omega -skew(x.omega)];
    dq = 0.5 * Omega * x.q;
    
    % Rotor speed dynamics (remains the same)
    tau = 0.02; 
    dn = (1/tau) * (uM(:) - x.n);
    
    % ... (assemble derivative struct dx)
    dx.p = dp;
    dx.v = dv;
    dx.q = dq;
    dx.omega = domega; % Already calculated
    dx.n = dn;
end

function [F_tot, M_tot] = rotorForcesAndMoments(uM, n, x, theta)
    global rotors;
    % REMOVED: cT, cM, cd
    rBC = theta.rBC; % CoG to IMU offset
    
    F_tot = zeros(3,1);
    M_tot = zeros(3,1);
    
    numMotors = length(n);
    for i = 1:numMotors
        ni = n(i);
        
        % --- NEW MODEL ---
        % Fetch the effectiveness vectors for this specific motor
        gamma_T_i = theta.gamma_T(:, i);
        gamma_M_i = theta.gamma_M(:, i);
        
        % Calculate force and moment directly in the rotor's own frame (Ai)
        % This elegantly combines magnitude and direction.
        F_Ai = gamma_T_i * ni^2;
        Mi_A = gamma_M_i * ni^2;
        % --- END NEW MODEL ---

        rBAi = rotors(i).r;  % Body to Rotor_i position
        C_BAi = rotors(i).R; % Body to Rotor_i orientation (usually eye(3))
        
        % Transform forces and moments to body frame
        F_B = C_BAi * F_Ai;
        rCAi = rBAi - rBC; % Vector from CoG to rotor
        M_B = C_BAi * Mi_A + cross(rCAi, F_B);
        
        F_tot = F_tot + F_B;
        M_tot = M_tot + M_B;
    end
end

function x_next = integrateMAV(xk, ukM, dt, theta)
    f = @(x) mavDynamics(x, ukM, theta);
    k1 = f(xk);
    k2 = f(addState(xk, scaleState(k1, dt/2)));
    k3 = f(addState(xk, scaleState(k2, dt/2)));
    k4 = f(addState(xk, scaleState(k3, dt)));
    
    dx_final = scaleState(k1, 1/6);
    dx_final = addState(dx_final, scaleState(k2, 2/6));
    dx_final = addState(dx_final, scaleState(k3, 2/6));
    dx_final = addState(dx_final, scaleState(k4, 1/6));
    
    x_next = addState(xk, scaleState(dx_final, dt));
end

function dx = imuDynamics(x, uI, theta)
    gyro = uI(1:3);
    accel = uI(4:6);

    % Get biases from the theta parameter struct
    omega_corr = gyro - theta.bw;
    accel_corr = accel - theta.ba;

    g_W = [0; 0; 9.81]; % Assuming NED
    C_IB = quat2rotm(x.q');

    dv = accel_corr + C_IB' * g_W - cross(omega_corr, x.v);

    Omega = [0 -omega_corr'; omega_corr -skew(omega_corr)];
    dq = 0.5 * Omega * x.q;

    dx = createEmptyState(length(x.n));
    dx.v = dv;
    dx.q = dq;
end


function x_next = integrateIMU(xk, uIk, dt, theta)
    f = @(x) imuDynamics(x, uIk, theta);
    k1 = f(xk);
    k2 = f(addState(xk, scaleState(k1, dt/2)));
    k3 = f(addState(xk, scaleState(k2, dt/2)));
    k4 = f(addState(xk, scaleState(k3, dt)));
    
    dx_final = scaleState(k1, 1/6);
    dx_final = addState(dx_final, scaleState(k2, 2/6));
    dx_final = addState(dx_final, scaleState(k3, 2/6));
    dx_final = addState(dx_final, scaleState(k4, 1/6));
    
    x_next = addState(xk, scaleState(dx_final, dt));
end

% A helper to scale the state derivative struct for RK4
function dx_scaled = scaleState(dx, factor)
    dx_scaled = dx;
    dx_scaled.p = dx.p * factor;
    dx_scaled.v = dx.v * factor;
    dx_scaled.q = dx.q * factor;
    dx_scaled.omega = dx.omega * factor;
    dx_scaled.n = dx.n * factor;
end

% --- Finite Difference for Jacobian Calculation ---
function J = finiteDifference(f, x0, dx)
    n = length(x0);
    f0 = f(x0);
    m = length(f0);
    J = zeros(m, n);
    for i = 1:n
        x_pert = x0;
        x_pert(i) = x_pert(i) + dx;
        f1 = f(x_pert);
        J(:, i) = (f1 - f0) / dx;
    end
end

% --- NEW RESIDUAL FUNCTION ---
function dchi_imu = imuStateResidual(xk, xk_pred)
    % This residual only compares the states predicted by the IMU model
    dv = xk.v - xk_pred.v;
    dq = quatError(xk_pred.q, xk.q);
    dchi_imu = [dv; dq];
end

% --- Batch System Assembly ---
function [residuals, A, state_idx, param_idx] = buildBatchSystem(Xbar, thetaBar, Z, U, dt_seq, R_pos, R_quat, Qm, Qi)
    K = length(Xbar);
    nx = length(getStateVector(Xbar(1)))-1; %-1 because using dtheta instead of quaternion. transforms later
    np = length(getParamVector(thetaBar));
    
    % Preallocate for speed
    num_meas_residuals = 6; % pos(3) + quat_err(3)
    num_dyn_residuals = nx;
    total_rows = (K-1) * (num_meas_residuals + 2 * num_dyn_residuals);
    residuals = zeros(total_rows, 1);
    A = sparse(total_rows, K*nx + np);
    
    state_idx = zeros(K, nx);
    
    current_row = 1;
    
    Qi_imu = 1e-3 * eye(6);

    for k = 2:K
        dt = dt_seq(k-1);
    
        Zk.p = Z.p(k, :);
        Zk.q = Z.q(k, :);
    
        % --- Measurement Residual ---
        [res_meas, R_meas] = measurementResidual(Xbar(k), Zk, R_pos, R_quat);
        L_meas = chol(inv(R_meas), 'lower');
        res_meas_norm = L_meas * res_meas;
    
        % Create a function that takes a minimal (20-element) dx as input
        f_meas_minimal = @(dx) measurementResidual(applyStateDelta(Xbar(k), dx), Zk, R_pos, R_quat);
        
        % Differentiate with respect to a zero dx vector of the correct size
        dx0 = zeros(nx, 1); % nx is already correctly set to 20
        H_meas_dx = finiteDifference(f_meas_minimal, dx0, 1e-7);
        H_meas = L_meas * H_meas_dx;

    
        % --- MAV Model Residual ---
        x_pred_mav = integrateMAV(Xbar(k-1), U.uM(k-1, :)', dt, thetaBar);
        res_dyn_mav = stateResidual(Xbar(k), x_pred_mav);
        L_mav = chol(inv(Qm), 'lower');
        res_dyn_mav_norm = L_mav * res_dyn_mav;
    
        f_mav_xkm1_minimal = @(dx) stateResidual(Xbar(k), integrateMAV(applyStateDelta(Xbar(k-1), dx), U.uM(k-1,:)', dt, thetaBar));
        dx0 = zeros(nx, 1); % nx is already correctly set to 20
        H_dynM_xkm1 = L_mav * finiteDifference(f_mav_xkm1_minimal, dx0, 1e-7);

        
        f_mav_xk_minimal = @(dx) stateResidual(applyStateDelta(Xbar(k), dx), x_pred_mav);
        dx0 = zeros(nx, 1);
        H_dynM_xk = L_mav * finiteDifference(f_mav_xk_minimal, dx0, 1e-7);
    
        f_mav_theta = @(theta_vec) stateResidual(Xbar(k), integrateMAV(Xbar(k-1), U.uM(k-1,:)', dt, setParamVector(theta_vec)));
        H_dynM_theta = L_mav * finiteDifference(f_mav_theta, getParamVector(thetaBar), 1e-7);
    
        % --- IMU Model Residual ---
        x_pred_imu = integrateIMU(Xbar(k-1), U.uI(k-1, :)', dt, thetaBar);
        
        res_dyn_imu = imuStateResidual(Xbar(k), x_pred_imu); % Use new function
        L_imu = chol(inv(Qi_imu), 'lower'); % Use new 6x6 covariance
        res_dyn_imu_norm = L_imu * res_dyn_imu;
    
        % We need a function handle that outputs the 6x1 imu residual.      
        % Jacobian w.r.t x_{k-1} 
        f_imu_xkm1_minimal = @(dx) imuStateResidual(Xbar(k), integrateIMU(applyStateDelta(Xbar(k-1), dx), U.uI(k-1,:)', dt, thetaBar));
        dx0 = zeros(nx, 1);
        H_dynI_xkm1 = L_imu * finiteDifference(f_imu_xkm1_minimal, dx0, 1e-7);

        
        % Jacobian w.r.t x_k
        % Here the prediction x_pred_imu is constant w.r.t x_k
        f_imu_xk_minimal = @(dx) imuStateResidual(applyStateDelta(Xbar(k), dx), x_pred_imu);
        dx0 = zeros(nx, 1);
        H_dynI_xk = L_imu * finiteDifference(f_imu_xk_minimal, dx0, 1e-7);

        % --- ADDED: Jacobian w.r.t. theta ---
        f_imu_theta_handle = @(theta_vec) imuStateResidual(Xbar(k), integrateIMU(Xbar(k-1), U.uI(k-1,:)', dt, setParamVector(theta_vec)));
        H_dynI_theta = L_imu * finiteDifference(f_imu_theta_handle, getParamVector(thetaBar), 1e-7);

    
        % --- Stack Residuals and Jacobians for this time step ---
        idx_km1 = (k-2)*nx + (1:nx);
        idx_k   = (k-1)*nx + (1:nx);
        state_idx(k-1,:) = idx_km1;
        state_idx(k,:) = idx_k;
        idx_theta = K*nx + (1:np);
        
        % Add measurement residual and Jacobian
        rows = current_row : current_row + size(res_meas_norm,1) - 1;
        residuals(rows) = res_meas_norm;
        A(rows, idx_k) = H_meas;
        current_row = rows(end) + 1;
        
        % Add MAV dynamics residual and Jacobians
        rows = current_row : current_row + size(res_dyn_mav_norm,1) - 1;
        residuals(rows) = res_dyn_mav_norm;
        A(rows, idx_km1) = H_dynM_xkm1;
        A(rows, idx_k) = H_dynM_xk;
        A(rows, idx_theta) = H_dynM_theta;

        rows = current_row : current_row + size(res_dyn_imu_norm, 1) - 1;
        A(rows, idx_theta) = H_dynI_theta; % ADDED THIS PART
        current_row = rows(end) + 1;
        
        % Add IMU dynamics residual and Jacobians
        rows = current_row : current_row + size(res_dyn_imu_norm,1) - 1;
        residuals(rows) = res_dyn_imu_norm;
        A(rows, idx_km1) = H_dynI_xkm1;
        A(rows, idx_k) = H_dynI_xk;
        current_row = rows(end) + 1;
    end
    
    param_idx = K*nx + (1:np);
end

% --- Utility Functions ---
function S = skew(v)
    % Converts a 3x1 vector to a 3x3 skew-symmetric matrix
    S = [  0  -v(3)  v(2);
          v(3)   0  -v(1);
         -v(2)  v(1)   0  ];
end

% --- NEW HELPER FUNCTION ---
function theta = sanitizeParameters(theta)
    % This function enforces physical constraints on the parameters.

    % Inertia values must be positive.
    % We enforce a small minimum value to prevent division by zero.
    min_inertia = 1e-6;
    theta.J(theta.J < min_inertia) = min_inertia;
end