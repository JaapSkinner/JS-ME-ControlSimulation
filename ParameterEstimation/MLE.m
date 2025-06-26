% Maximum Likelyhood Estimation of physical parameters from flight data


%% Load data and form structures

load('simData.mat');  % replace with actual file

% Structure the measurement set Z (pose + optional rotor speeds)
Z = struct();
Z.t = timeVec;                      % timestamps
Z.p = pos_WB;                       % position
Z.q = quat_WB;                      % quaternion orientation
Z.hasRotorSpeed = exist('rotorSpeeds', 'var') == 1;
if Z.hasRotorSpeed
    Z.n = rotorSpeeds;
end

% Structure input set U (control + inertial)
U = struct();
U.t = timeVec;
U.uM = motorCmds;                   % motor commands
U.uI = [gyro_B, accel_B];           % IMU readings: [ω a]

%% Set initial guesses for state trajectory and parameters

% Define state template
% Note: use this template to initialize each time step state x_k
function x = createEmptyState(numMotors)
    x = struct();
    x.p = zeros(3,1);              % position in world frame
    x.v = zeros(3,1);              % velocity in body frame
    x.q = [1; 0; 0; 0];            % orientation quaternion (wxyz)
    x.omega = zeros(3,1);          % angular velocity in body frame
    x.bw = zeros(3,1);             % gyro bias
    x.ba = zeros(3,1);             % accel bias
    x.n = zeros(numMotors,1);     % rotor speeds
end

% Define parameter vector template
function theta = createEmptyParams()
    theta = struct();
    theta.cT = 0;                 % thrust coefficient
    theta.cM = 0;                 % moment coefficient
    theta.cd = 0;                 % drag coefficient
    theta.J = zeros(3,1);         % diagonal inertia [Jx Jy Jz]
    theta.rBC = zeros(3,1);       % IMU offset from CoG
end

% Number of time steps
K = length(Z.t);
numMotors = size(U.uM, 2);

% Preallocate state trajectory
Xbar = repmat(createEmptyState(numMotors), K, 1);

for k = 1:K
    Xbar(k).p = Z.p(k, :)';         % position from Vicon
    Xbar(k).q = Z.q(k, :)';         % orientation quaternion
    % v, omega, biases, n are initialized to zeros
end

thetaBar = createEmptyParams();
thetaBar.cT = 1e-6;                % guess: thrust coefficient
thetaBar.cM = 1e-7;                % guess: moment coefficient
thetaBar.cd = 0.1;                 % guess: drag coefficient
thetaBar.J = [5e-3; 5e-3; 1e-2];   % guess: diagonal inertia
thetaBar.rBC = [0; 0; 0];          % IMU at CoG initially

%% State and Param update helpers
function x_new = applyStateDelta(x, dx)
    % dx is a vector containing [dp; dv; dtheta; domega; dbw; dba; dn]
    x_new = x;
    x_new.p = x.p + dx(1:3);
    x_new.v = x.v + dx(4:6);
    
    % For quaternion update, convert small angle delta to quaternion
    dtheta = dx(7:9);
    dq = [1; 0.5*dtheta]; % approx small angle quaternion
    x_new.q = quatnormalize(quatmultiply(x.q', dq'))';
    
    x_new.omega = x.omega + dx(10:12);
    x_new.bw = x.bw + dx(13:15);
    x_new.ba = x.ba + dx(16:18);
    x_new.n = x.n + dx(19:end);
end

function theta = setParamVector(theta_vec)
    % theta_vec is vector of parameters matching order in your theta struct
    theta = struct();
    theta.cT = theta_vec(1);
    theta.cM = theta_vec(2);
    theta.cd = theta_vec(3);
    theta.J = theta_vec(4:6);
    theta.rBC = theta_vec(7:9);
end

function theta_vec = getParamVector(theta)
    % Converts theta struct back to vector form
    theta_vec = [theta.cT; theta.cM; theta.cd; theta.J; theta.rBC];
end


%% Measurement Residuals

% Compute minimal quaternion residual (3D vector)
function dq = quatError(q_est, q_meas)
    % Both quaternions are column vectors [w; x; y; z]
    q_err = quatmultiply(quatconj(q_est'), q_meas');  % MATLAB needs row inputs
    q_err = q_err';  % convert back to column
    dq = 2 * q_err(2:4);  % small angle approx: vector part scaled
end

% Return pose residual (position + orientation)
function [res, R] = measurementResidual(xk, zk, R_pos, R_quat)
    dp = zk.p' - xk.p;
    dq = quatError(xk.q, zk.q);
    res = [dp; dq];
    R = blkdiag(R_pos, R_quat);  % full measurement covariance
end

function [res, Rn] = rotorSpeedResidual(xk, nk_meas, Rn_scalar)
    res = nk_meas' - xk.n;
    Rn = Rn_scalar * eye(length(xk.n));
end

%% Model Residuals
function x_next = integrateMAV(xk, ukM, dt, theta)
    % Runge-Kutta 4 integration of MAV dynamics
    f = @(x) mavDynamics(x, ukM, theta);
    x1 = f(xk);
    x2 = f(addState(xk, x1 * dt/2));
    x3 = f(addState(xk, x2 * dt/2));
    x4 = f(addState(xk, x3 * dt));
    dx = (x1 + 2*x2 + 2*x3 + x4) * dt / 6;
    x_next = addState(xk, dx);
end

function dx = mavDynamics(x, uM, theta)
    % Unpack parameters
    m = 1;  % assume known for now (mass)
    J = diag(theta.J);        % 3x3 inertia
    rBC = theta.rBC;          % IMU offset

    cT = theta.cT;
    cM = theta.cM;
    cd = theta.cd;

    % Convert quaternion to rotation matrix
    C_IB = quat2rotm(x.q');  % MATLAB needs row input

    % === 15a === Position derivative in world frame
    dp = C_IB * x.v;

    % === 15b === Velocity derivative in body frame
    [F_tot, M_tot] = rotorForcesAndMoments(uM, x.n, x, theta);  % see below
    g = [0; 0; 9.81];

    term1 = (1/m) * F_tot;
    term2 = -C_IB' * g;
    term3 = -cross(x.omega, x.v);
    term4 = -cross(x.omega_dot, rBC);  % needs ω̇ — set to 0 for now
    term5 = -cross(x.omega, cross(x.omega, rBC));

    dv = term1 + term2 + term3 + term4 + term5;

    % === 15c === Quaternion derivative
    Omega = [0       -x.omega';
             x.omega -skew(x.omega)];
    dq = 0.5 * Omega * x.q;

    % === 15d === Angular acceleration
    domega = J \ (M_tot - cross(x.omega, J * x.omega));

    % === 15e === Rotor speeds
    tau = 0.02;  % guess
    dn = (1/tau) * (uM(:) - x.n);

    % === Output as struct ===
    dx = struct();
    dx.p = dp;
    dx.v = dv;
    dx.q = dq;
    dx.omega = domega;
    dx.bw = zeros(3,1);  % biases constant
    dx.ba = zeros(3,1);
    dx.n = dn;
end
%% Rotor forces and moments
function [F_tot, M_tot] = rotorForcesAndMoments(uM, n, x, theta)
    % rotor geometry assumed global
    global rotors

    cT = theta.cT;
    cM = theta.cM;
    cd = theta.cd;
    rBC = theta.rBC;

    F_tot = zeros(3,1);
    M_tot = zeros(3,1);

    for i = 1:length(n)
        ni = n(i);
        Ti = cT * ni^2;
        Mi = cM * ni^2 * [0; 0; 1];  % in Ai frame

        % v_hub_i in Ai frame
        rBAi = rotors(i).r;
        C_BAi = rotors(i).R;
        v_hub_B = x.v + cross(x.omega, rBAi);
        v_hub_Ai = C_BAi' * v_hub_B;

        % drag term
        D = diag([cd, cd, 0]);
        F_Ai = (Ti * [0; 0; 1]) - Ti * D * v_hub_Ai;

        % Transform to body frame
        F_B = C_BAi * F_Ai;

        rCAi = rBAi - rBC;
        M_B = C_BAi * Mi + cross(F_B, rCAi);

        F_tot = F_tot + F_B;
        M_tot = M_tot + M_B;
    end
end
%% IMU Driven Dynamics
function dx = imuDynamics(x, uI)
    % uI = [gyro_meas(3x1); accel_meas(3x1)]
    gyro = uI(1:3);
    accel = uI(4:6);

    % Corrected IMU signals
    omega_corr = gyro - x.bw;
    accel_corr = accel - x.ba;

    % Gravity
    g = [0; 0; 9.81];

    % Rotation
    C_IB = quat2rotm(x.q');

    % === 16a === velocity derivative
    term1 = accel_corr;
    term2 = -C_IB' * g;
    term3 = -cross(omega_corr, x.v);
    dv = term1 + term2 + term3;

    % === 16b === quaternion derivative
    Omega = [0 -omega_corr';
             omega_corr -skew(omega_corr)];
    dq = 0.5 * Omega * x.q;

    % === 16c/d === biases integrate white noise (ignored here)
    dbw = zeros(3,1);
    dba = zeros(3,1);

    % === Output as struct ===
    dx = struct();
    dx.p = zeros(3,1);     % no position update
    dx.v = dv;
    dx.q = dq;
    dx.omega = zeros(3,1); % no angular model
    dx.bw = dbw;
    dx.ba = dba;
    dx.n = zeros(length(x.n),1);  % rotor speeds not used here
end

function x_next = integrateIMU(xk, uIk, dt)
    f = @(x) imuDynamics(x, uIk);
    x1 = f(xk);
    x2 = f(addState(xk, x1 * dt/2));
    x3 = f(addState(xk, x2 * dt/2));
    x4 = f(addState(xk, x3 * dt));
    dx = (x1 + 2*x2 + 2*x3 + x4) * dt / 6;
    x_next = addState(xk, dx);
end

function dchi = stateResidual(xk, xk_pred)
    % Compute minimal residual between two states
    dp = xk.p - xk_pred.p;
    dv = xk.v - xk_pred.v;
    dq = quatError(xk_pred.q, xk.q);  % minimal rotation vector
    domega = xk.omega - xk_pred.omega;
    dbw = xk.bw - xk_pred.bw;
    dba = xk.ba - xk_pred.ba;
    dn = xk.n - xk_pred.n;

    dchi = [dp; dv; dq; domega; dbw; dba; dn];
end

%% difference helper

function J = finiteDifference(f, x0, dx)
    % f: function handle returning residual vector
    % x0: nominal point (column vector)
    % dx: small perturbation (scalar)
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

%% Build Batch system
function [residuals, A, state_idx, param_idx] = buildBatchSystem(Xbar, thetaBar, Z, U, dt_seq, R_pos, R_quat, Qm, Qi)
    K = length(Xbar);
    nx = length(getStateVector(Xbar(1)));
    np = length(getParamVector(thetaBar));

    residuals = [];
    A = [];
    state_idx = zeros(K, nx);
    total_idx = 0;

    for k = 2:K
        dt = dt_seq(k-1);
        
        % === Measurement residual ===
        [res_meas, R_meas] = measurementResidual(Xbar(k), Z(k), R_pos, R_quat);
        L_meas = chol(R_meas, 'lower') \ eye(size(R_meas));
        res_meas_norm = L_meas * res_meas;

        f_meas = @(x_vec) measurementResidual(setStateVector(Xbar(k), x_vec), Z(k), R_pos, R_quat);
        H_meas = L_meas * finiteDifference(f_meas, getStateVector(Xbar(k)), 1e-6);

        % === MAV model residual ===
        x_pred_mav = integrateMAV(Xbar(k-1), U.uM(k-1, :)', dt, thetaBar);
        res_dyn_mav = stateResidual(Xbar(k), x_pred_mav);
        L_mav = chol(Qm, 'lower') \ eye(size(Qm));
        res_dyn_mav_norm = L_mav * res_dyn_mav;

        f_mav_x = @(x_vec) stateResidual(Xbar(k), ...
            integrateMAV(setStateVector(Xbar(k-1), x_vec), U.uM(k-1,:)', dt, thetaBar));
        H_dynM_x = L_mav * finiteDifference(f_mav_x, getStateVector(Xbar(k-1)), 1e-6);

        f_mav_theta = @(theta_vec) stateResidual(Xbar(k), ...
            integrateMAV(Xbar(k-1), U.uM(k-1,:)', dt, setParamVector(theta_vec)));
        H_dynM_theta = L_mav * finiteDifference(f_mav_theta, getParamVector(thetaBar), 1e-6);

        % === IMU model residual ===
        x_pred_imu = integrateIMU(Xbar(k-1), U.uI(k-1, :)', dt);
        res_dyn_imu = stateResidual(Xbar(k), x_pred_imu);
        L_imu = chol(Qi, 'lower') \ eye(size(Qi));
        res_dyn_imu_norm = L_imu * res_dyn_imu;

        f_imu_x = @(x_vec) stateResidual(Xbar(k), ...
            integrateIMU(setStateVector(Xbar(k-1), x_vec), U.uI(k-1,:)', dt));
        H_dynI_x = L_imu * finiteDifference(f_imu_x, getStateVector(Xbar(k-1)), 1e-6);

        % === Stack residuals ===
        residuals = [residuals; res_meas_norm; res_dyn_mav_norm; res_dyn_imu_norm];

        % === Stack Jacobians ===
        A_k = zeros(size(res_meas_norm,1) + size(res_dyn_mav_norm,1) + size(res_dyn_imu_norm,1), ...
                   K*nx + np);

        idx_km1 = (k-2)*nx + (1:nx);
        idx_k   = (k-1)*nx + (1:nx);
        state_idx(k,:) = idx_k;

        % H_meas applies to x_k
        A_k(1:size(H_meas,1), idx_k) = H_meas;

        % H_dynM_x applies to x_{k-1}, H_dynM_theta to θ
        r1 = size(H_meas,1) + 1;
        r2 = r1 + size(H_dynM_x,1) - 1;
        A_k(r1:r2, idx_km1) = H_dynM_x;
        A_k(r1:r2, end-np+1:end) = H_dynM_theta;

        % H_dynI_x applies to x_{k-1}
        r3 = r2 + 1;
        r4 = r3 + size(H_dynI_x,1) - 1;
        A_k(r3:r4, idx_km1) = H_dynI_x;

        % Stack into total system
        A = [A; A_k];
    end

    % Final param index range
    param_idx = (K-1)*nx + (1:np);
end

%% Run MLE until convergance

max_iters = 10;
tol = 1e-6;
cost_prev = Inf;

for iter = 1:max_iters
    fprintf("Iteration %d...\n", iter);

    % 1. Assemble residuals + Jacobians
    [residuals, A, state_idx, param_idx] = buildBatchSystem(Xbar, thetaBar, Z, U, dt_seq, R_pos, R_quat, Qm, Qi);

    % 2. Solve least squares: A * delta = residuals
    delta = A \ residuals;

    % 3. Update states
    for k = 1:K
        idx = state_idx(k,:);
        dx = delta(idx);
        Xbar(k) = applyStateDelta(Xbar(k), dx);
    end

    % 4. Update parameters
    thetaBar = setParamVector(getParamVector(thetaBar) + delta(param_idx));

    % 5. Check convergence
    cost = norm(residuals);
    if abs(cost_prev - cost) < tol
        fprintf("Converged at iteration %d with cost %.6f\n", iter, cost);
        break;
    end
    cost_prev = cost;
end

disp("Estimated Parameters:");
disp(thetaBar);