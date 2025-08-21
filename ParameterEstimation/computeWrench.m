function W_ss = computeWrench(u_sp, params)
% computeWrench - Computes the steady-state UAV wrench from a control setpoint.
%
% This function implements the full motor and mixer dynamics for an 
% overactuated UAV to find the steady-state wrench in the body frame.
% It is the analytical steady-state simplification of the entire dynamics
% chain, including control allocation, motor physics, and wrench combination.
%
% Syntax:  W_ss = computeWrench(u_sp, params)
%
% Inputs:
%    u_sp - The raw control setpoint vector (e.g., from joystick/autopilot).
%           A 6x1 column vector [Fx_d; Fy_d; Fz_d; Tau_x_d; Tau_y_d; Tau_z_d].
%
%    params - A structure containing all necessary matrices and constants.
%             It must contain the following fields:
%             .N         : Total number of motors (scalar)
%             .A         : Input scaling matrix (6x6 diagonal)
%             .u_min     : Lower clip limits for scaled inputs (6x1 vector)
%             .u_max     : Upper clip limits for scaled inputs (6x1 vector)
%             .M         : The complete 6-DOF mixer matrix (Nx6)
%             .B_matrix  : Overall wrench effectiveness matrix (6xN). This
%                          matrix encapsulates the logic from your
%                          'motorCombined.m' and 'calculate_ss_forces.m' scripts.
%             -- Motor Dynamics Coefficients --
%             .C_tau     : Motor constant A for the quadratic equation
%             .Kt        : Motor torque constant
%             .Ke        : Motor back-EMF constant
%             .R         : Motor terminal resistance
%             .mv        : Voltage mapping slope
%             .V_off     : Voltage mapping offset
%             .I0        : Motor no-load current
%
% Outputs:
%    W_ss - The total steady-state wrench vector [T_ss; tau_ss] in the 
%           body frame. A 6x1 column vector.
%

% --- 1. Input Validation ---
if ~isvector(u_sp) || length(u_sp) ~= 6
    error('u_sp must be a 6-element vector.');
end
% Ensure u_sp is a column vector for matrix operations
u_sp = u_sp(:); 

% --- 2. Unpack Parameters from Struct for Readability ---
A_scale   = params.A;
u_min     = params.u_min;
u_max     = params.u_max;
M_mixer   = params.M;
B_wrench  = params.B_matrix;
N         = params.N;

% --- 3. Implement the Main Expression Step-by-Step ---

% Step 1 & 2: Scale and clip the raw setpoint vector u_sp
% Corresponds to: [ A * u_sp ]_u_min^u_max
u_scaled = A_scale * u_sp;
u_clipped = max(u_min, min(u_max, u_scaled));

% Step 3, 4, 5: Apply mixer, clip, and normalize to get motor commands [0, 1]
% This is the control allocation portion.
motor_mix_commands = M_mixer * u_clipped;
motor_mix_clipped = max(-1, min(1, motor_mix_commands));
u_normalized = 0.5 * (motor_mix_clipped + ones(N, 1));

% Step 6: Apply element-wise function f_omega^2 to get squared speeds.
% This vectorized step performs the same logic as your 'fcn.m' for all N motors.
omega_ss_sq = calculate_omega_sq(u_normalized, params);

% Step 7: Apply the wrench effectiveness matrix to get the final wrench.
% This single multiplication combines all motor forces and torques,
% as shown in your 'motorCombined.m' script, for the steady-state case.
W_ss = B_wrench * omega_ss_sq;

end


function omega_sq = calculate_omega_sq(u, motor_params)
% Helper function to solve for steady-state omega^2 for each motor.
% This is a vectorized version of the logic in your 'fcn.m' script.

    % --- Unpack motor dynamics coefficients ---
    A_quad = motor_params.C_tau; % Corresponds to your 'Ctau'
    Kt     = motor_params.Kt;
    Ke     = motor_params.Ke;
    R      = motor_params.R;
    mv     = motor_params.mv;
    V_off  = motor_params.V_off;
    I0     = motor_params.I0;

    % --- Calculate quadratic equation coefficients (matches your fcn.m) ---
    B_quad = (Kt * Ke) / R;
    C_quad_vec = -(Kt / R) * (mv * u + V_off - I0 * R);

    % --- Solve for omega_ss for each motor (vectorized) ---
    % Calculate the discriminant (the part inside the square root)
    discriminant = B_quad^2 - 4 * A_quad * C_quad_vec;
    
    % Handle cases where the discriminant is negative (matches your fcn.m logic)
    discriminant(discriminant < 0) = 0;
    
    % Solve the quadratic equation for omega (we take the positive root)
    omega_ss = (-B_quad + sqrt(discriminant)) / (2 * A_quad);
    
    % The final result is the square of the angular velocities
    omega_sq = omega_ss.^2;
end