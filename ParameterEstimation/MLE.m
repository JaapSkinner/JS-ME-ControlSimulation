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
U.uM = pwm.Data;                   % motor commands
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
thetaBar.cT = 1e-6;                % guess: thrust coefficient
thetaBar.cM = 1e-7;                % guess: moment coefficient
thetaBar.cd = 0.1;                 % guess: drag coefficient
thetaBar.J = [5e-3; 5e-3; 1e-2];   % guess: diagonal inertia
thetaBar.rBC = [0; 0; 0];          % IMU at CoG initially

%% Measuement Covariances
R_pos = 1e-4 * eye(3);   % Position noise: small variance
R_quat = 1e-4 * eye(3);  % Quaternion (rotation vector) noise
nx = length(getStateVector(Xbar(1)));  % state vector dimension
Qm = 1e-3 * eye(nx);   % MAV dynamics process noise
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
    % This function defines the physical layout of the UAV's rotors.
    % --- YOU MUST MODIFY THESE VALUES FOR YOUR SPECIFIC UAV ---
    global rotors;
    
    % Example for a 250mm quadcopter in 'X' configuration
    d = 0.25 / sqrt(2); % distance from center to a motor
    
    rotors(1).r = [ d;  d; 0]; % Position of Rotor 1 (Front-Right)
    rotors(2).r = [ d; -d; 0]; % Position of Rotor 2 (Back-Right)
    rotors(3).r = [-d; -d; 0]; % Position of Rotor 3 (Back-Left)
    rotors(4).r = [-d;  d; 0]; % Position of Rotor 4 (Front-Left)
    
    % All rotors are assumed to be parallel to the body XY plane (no tilt)
    for i = 1:4
        rotors(i).R = eye(3);
    end
end

% --- State and Parameter Struct Definitions ---
function x = createEmptyState(numMotors)
    x = struct();
    x.p = zeros(3,1);              % position in world frame
    x.v = zeros(3,1);              % velocity in body frame
    x.q = [1; 0; 0; 0];            % orientation quaternion (wxyz)
    x.omega = zeros(3,1);          % angular velocity in body frame
    x.bw = zeros(3,1);             % gyro bias
    x.ba = zeros(3,1);             % accel bias
    x.n = zeros(numMotors,1);      % rotor speeds
end

function theta = createEmptyParams()
    theta = struct();
    theta.cT = 0;                 % thrust coefficient
    theta.cM = 0;                 % moment coefficient
    theta.cd = 0;                 % drag coefficient
    theta.J = zeros(3,1);         % diagonal inertia [Jx Jy Jz]
    theta.rBC = zeros(3,1);       % IMU offset from CoG
end

% --- State and Parameter Vector/Struct Conversion ---
function x_vec = getStateVector(x)
    x_vec = [x.p; x.v; x.q; x.omega; x.bw; x.ba; x.n];
end

function x = setStateVector(x_template, x_vec)
    x = x_template;
    idx = 1;
    x.p = x_vec(idx:idx+2); idx = idx+3;
    x.v = x_vec(idx:idx+2); idx = idx+3;
    x.q = x_vec(idx:idx+3); idx = idx+4;
    x.omega = x_vec(idx:idx+2); idx = idx+3;
    x.bw = x_vec(idx:idx+2); idx = idx+3;
    x.ba = x_vec(idx:idx+2); idx = idx+3;
    x.n = x_vec(idx:end);
end

function theta_vec = getParamVector(theta)
    theta_vec = [theta.cT; theta.cM; theta.cd; theta.J; theta.rBC];
end

function theta = setParamVector(theta_vec)
    theta = createEmptyParams(); % Ensures all fields are present
    theta.cT = theta_vec(1);
    theta.cM = theta_vec(2);
    theta.cd = theta_vec(3);
    theta.J = theta_vec(4:6);
    theta.rBC = theta_vec(7:9);
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
    x_new.bw = x.bw + dx(13:15);
    x_new.ba = x.ba + dx(16:18);
    x_new.n = x.n + dx(19:end);
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
    x_out.bw = x_in.bw + dx_struct.bw;
    x_out.ba = x_in.ba + dx_struct.ba;
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
    dbw = xk.bw - xk_pred.bw;
    dba = xk.ba - xk_pred.ba;
    dn = xk.n - xk_pred.n;
    dchi = [dp; dv; dq; domega; dbw; dba; dn];
end

% --- Dynamics and Integration ---
function dx = mavDynamics(x, uM, theta)
    global rotors; % Access the global rotor geometry
    m = 1;  
    J = diag(theta.J);
    rBC = theta.rBC;
    
    C_IB = quat2rotm(x.q');
    
    % Position derivative
    dp = C_IB * x.v;
    
    % Velocity derivative
    [F_tot, M_tot] = rotorForcesAndMoments(uM, x.n, x, theta);
    g_W = [0; 0; -9.81]; % Gravity in world frame
    dv = (1/m) * F_tot + C_IB' * g_W - cross(x.omega, x.v);
    % Note: Simplified model, assumes IMU at CoG (rBC=0), so omega_dot term is omitted
    
    % Quaternion derivative
    Omega = [0 -x.omega'; x.omega -skew(x.omega)];
    dq = 0.5 * Omega * x.q;
    
    % Angular acceleration
    domega = J \ (M_tot - cross(x.omega, J * x.omega));
    
    % Rotor speed dynamics
    tau = 0.02; % motor time constant (guess)
    dn = (1/tau) * (uM(:) - x.n);
    
    % Assemble derivative struct
    dx = createEmptyState(length(uM));
    dx.p = dp;
    dx.v = dv;
    dx.q = dq;
    dx.omega = domega;
    dx.n = dn;
end

function [F_tot, M_tot] = rotorForcesAndMoments(uM, n, x, theta)
    global rotors;
    cT = theta.cT;
    cM = theta.cM;
    cd = theta.cd;
    rBC = theta.rBC; % CoG to IMU offset
    
    F_tot = zeros(3,1);
    M_tot = zeros(3,1);
    
    for i = 1:length(n)
        ni = n(i);
        Ti = cT * ni^2;
        Mi_A = cM * ni^2 * [0; 0; 1]; % Moment in rotor's own frame (Ai)
        
        rBAi = rotors(i).r; % Body to Rotor_i position
        C_BAi = rotors(i).R; % Body to Rotor_i orientation
        
        v_hub_B = x.v + cross(x.omega, rBAi);
        v_hub_Ai = C_BAi' * v_hub_B;
        
        % Drag force in rotor frame
        D = diag([cd, cd, 0]);
        F_drag_Ai = -Ti * D * v_hub_Ai;
        
        % Total force from this rotor in rotor frame
        F_Ai = [0; 0; Ti] + F_drag_Ai;
        
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

function dx = imuDynamics(x, uI)
    gyro = uI(1:3);
    accel = uI(4:6);
    
    omega_corr = gyro - x.bw;
    accel_corr = accel - x.ba;
    
    g_W = [0; 0; -9.81];
    C_IB = quat2rotm(x.q');
    
    dv = C_IB * accel_corr + g_W - cross(omega_corr, x.v);
    
    Omega = [0 -omega_corr'; omega_corr -skew(omega_corr)];
    dq = 0.5 * Omega * x.q;
    
    dx = createEmptyState(length(x.n));
    dx.v = dv;
    dx.q = dq;
end

function x_next = integrateIMU(xk, uIk, dt)
    f = @(x) imuDynamics(x, uIk);
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
    dx_scaled.bw = dx.bw * factor;
    dx_scaled.ba = dx.ba * factor;
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

% --- Batch System Assembly ---
function [residuals, A, state_idx, param_idx] = buildBatchSystem(Xbar, thetaBar, Z, U, dt_seq, R_pos, R_quat, Qm, Qi)
    K = length(Xbar);
    nx = length(getStateVector(Xbar(1)));
    np = length(getParamVector(thetaBar));
    
    % Preallocate for speed
    num_meas_residuals = 6; % pos(3) + quat_err(3)
    num_dyn_residuals = nx;
    total_rows = (K-1) * (num_meas_residuals + 2 * num_dyn_residuals);
    residuals = zeros(total_rows, 1);
    A = sparse(total_rows, K*nx + np);
    
    state_idx = zeros(K, nx);
    
    current_row = 1;
    
    for k = 2:K
        dt = dt_seq(k-1);
    
        Zk.p = Z.p(k, :);
        Zk.q = Z.q(k, :);
    
        % --- Measurement Residual ---
        [res_meas, R_meas] = measurementResidual(Xbar(k), Zk, R_pos, R_quat);
        L_meas = chol(inv(R_meas), 'lower');
        res_meas_norm = L_meas * res_meas;
    
        f_meas = @(x_vec) measurementResidual(setStateVector(Xbar(k), x_vec), Zk, R_pos, R_quat);
        H_meas_dx = finiteDifference(f_meas, getStateVector(Xbar(k)), 1e-7);
        H_meas = L_meas * H_meas_dx;
    
        % --- MAV Model Residual ---
        x_pred_mav = integrateMAV(Xbar(k-1), U.uM(k-1, :)', dt, thetaBar);
        res_dyn_mav = stateResidual(Xbar(k), x_pred_mav);
        L_mav = chol(inv(Qm), 'lower');
        res_dyn_mav_norm = L_mav * res_dyn_mav;
    
        f_mav_xkm1 = @(x_vec) stateResidual(Xbar(k), integrateMAV(setStateVector(Xbar(k-1), x_vec), U.uM(k-1,:)', dt, thetaBar));
        H_dynM_xkm1 = L_mav * finiteDifference(f_mav_xkm1, getStateVector(Xbar(k-1)), 1e-7);
        
        f_mav_xk = @(x_vec) stateResidual(setStateVector(Xbar(k), x_vec), x_pred_mav);
        H_dynM_xk = L_mav * finiteDifference(f_mav_xk, getStateVector(Xbar(k)), 1e-7);
    
        f_mav_theta = @(theta_vec) stateResidual(Xbar(k), integrateMAV(Xbar(k-1), U.uM(k-1,:)', dt, setParamVector(theta_vec)));
        H_dynM_theta = L_mav * finiteDifference(f_mav_theta, getParamVector(thetaBar), 1e-7);
    
        % --- IMU Model Residual ---
        x_pred_imu = integrateIMU(Xbar(k-1), U.uI(k-1, :)', dt);
        res_dyn_imu = stateResidual(Xbar(k), x_pred_imu);
        L_imu = chol(inv(Qi), 'lower');
        res_dyn_imu_norm = L_imu * res_dyn_imu;
    
        f_imu_xkm1 = @(x_vec) stateResidual(Xbar(k), integrateIMU(setStateVector(Xbar(k-1), x_vec), U.uI(k-1,:)', dt));
        H_dynI_xkm1 = L_imu * finiteDifference(f_imu_xkm1, getStateVector(Xbar(k-1)), 1e-7);
        
        f_imu_xk = @(x_vec) stateResidual(setStateVector(Xbar(k), x_vec), x_pred_imu);
        H_dynI_xk = L_imu * finiteDifference(f_imu_xk, getStateVector(Xbar(k)), 1e-7);
    
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