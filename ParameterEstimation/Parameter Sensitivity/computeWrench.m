function W_ss = computeWrench(W_des, params)
% computeWrench - Computes the steady-state UAV wrench from a 6-DOF wrench setpoint.
% MATRIX VERSION: Aligned with the MultirotorMixer.m source code.
%
% Inputs:
%    W_des - The desired 6-DOF wrench setpoint vector [Fx; Fy; Fz; Tx; Ty; Tz].
%
%    params - A structure containing all necessary matrices and constants.
%             .A_eff     : The effective scaling & reordering matrix (6x6).
%             .u_min     : Clip limits for the scaled inputs (6x1 vector [Fx,Fy,Fz,Tx,Ty,Tz]).
%             .u_max     : Clip limits for the scaled inputs (6x1 vector [Fx,Fy,Fz,Tx,Ty,Tz]).
%             .M         : The complete Mixer Matrix (Nx6).
%             ... plus all per-motor dynamics parameters (Kt, Ke, etc. as Nx1 vectors)
%
% Outputs:
%    W_ss - The resulting steady-state wrench vector (6x1).
%

% --- 1. Apply Effective Scaling & Reordering Matrix ---
u_scaled_reordered = params.A_eff * W_des(:);

% --- 2. Clip the result ---
% The clip limits u_min/max need to be reordered to match tilde_u's structure.
% [Tx Ty Tz Fz Fx Fy]
u_min_reordered = [params.u_min(4:6); params.u_min(3); params.u_min(1:2)];
u_max_reordered = [params.u_max(4:6); params.u_max(3); params.u_max(1:2)];
tilde_u = max(u_min_reordered, min(u_max_reordered, u_scaled_reordered));

% --- 3. Apply Mixer and Normalize ---
o_mixer = params.M * tilde_u;
o_mixer_clipped = max(-1, min(1, o_mixer));
u_dyn = 0.5 * (o_mixer_clipped + ones(params.N, 1));

% --- 4. Calculate Squared Rotor Speeds ---
omega_ss_sq = calculate_omega_sq(u_dyn, params);

% --- 5. Calculate Final Wrench ---
W_ss = params.B_matrix * omega_ss_sq;

end

function omega_sq = calculate_omega_sq(u, motor_params)
% Helper function to solve for omega^2 for each motor using per-motor parameters.
    A_quad_vec = motor_params.C_tau(:); Kt_vec = motor_params.Kt(:);
    Ke_vec = motor_params.Ke(:); R_vec = motor_params.R(:);
    mv_vec = motor_params.mv(:); V_off_vec = motor_params.V_off(:);
    I0_vec = motor_params.I0(:);
    B_quad_vec = (Kt_vec .* Ke_vec) ./ R_vec;
    C_quad_vec = -(Kt_vec ./ R_vec) .* (mv_vec .* u + V_off_vec - I0_vec .* R_vec);
    discriminant = B_quad_vec.^2 - 4 * A_quad_vec .* C_quad_vec;
    discriminant(discriminant < 0) = 0;
    omega_ss = (-B_quad_vec + sqrt(discriminant)) ./ (2 * A_quad_vec);
    omega_sq = omega_ss.^2;
end

