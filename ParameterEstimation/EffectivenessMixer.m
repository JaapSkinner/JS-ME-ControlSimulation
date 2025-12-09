%% 1. Calculate Standard Effectiveness Matrix (Physics Order)
% Standard Order: [Fx, Fy, Fz, Mx, My, Mz]
N_MOTORS = Uav.N_ROTORS;
kf_vec = 0.5 * Uav.RHO_AIR * Aero.Cz3P.coefs(1) * (Uav.D_PROP.^2) .* Uav.A_PROP;
km_vec = Motor.C_TAU;

B_physics = zeros(6, N_MOTORS);

for m = 1:N_MOTORS
    % Local Thrust (-Z) and Torque vectors
    t_local   = [0; 0; -kf_vec(m)];
    tau_local = [0; 0; -Uav.ROTOR_DIRECTION(m) * km_vec(m)];
    
    % Rotate to Body Frame
    R = Uav.R_MOTOR_TO_BODY(:,:,m);
    f_body = R * t_local;
    
    % Moments = Torque + (Position x Force)
    m_body = (R * tau_local) + cross(Uav.MotorLoc(m, 1:3)', f_body);
    
    % Fill Standard B Matrix
    B_physics(:, m) = [f_body; m_body];
end

%% 2. Calculate Raw Mixer (Pseudoinverse)
% Solves F = B * u
Mixer_Physics = pinv(B_physics);

% Clean up numerical noise (e.g., 1e-17 becomes 0)
Mixer_Physics(abs(Mixer_Physics) < 1e-9) = 0;

%% 3. Re-Order Columns to: [Roll, Pitch, Yaw, Z, X, Y]
% Current Physics Indices: 
% 1:Fx, 2:Fy, 3:Fz, 4:Mx(Roll), 5:My(Pitch), 6:Mz(Yaw)

% Desired Map:
% Col 1 (Roll)  <-- Old Col 4
% Col 2 (Pitch) <-- Old Col 5
% Col 3 (Yaw)   <-- Old Col 6
% Col 4 (Z)     <-- Old Col 3
% Col 5 (X)     <-- Old Col 1
% Col 6 (Y)     <-- Old Col 2

perm_order = [4, 5, 6, 3, 1, 2];
Mixer_Reordered = Mixer_Physics(:, perm_order);

%% 4. Normalize (Make Max Value 1.0)
% This removes the tiny units (1e-5) and scales the inputs 
% so a command of 1.0 hits the motor limit.

Mixer_Final = zeros(size(Mixer_Reordered));

for i = 1:6
    col = Mixer_Reordered(:, i);
    max_val = max(abs(col));
    
    if max_val > 1e-12
        Mixer_Final(:, i) = col / max_val;
    else
        Mixer_Final(:, i) = 0; % Axis not controllable
    end
end

%% 5. Output
disp('===================================================');
disp('      6-DOF MIXER (Normalized & Re-Ordered)');
disp('===================================================');
disp('Columns: [ Roll | Pitch | Yaw | Thrust Z | Force X | Force Y ]');
disp('Rows:    [ Motor 1 ... Motor N ]');
disp(' ');

% Use formatted printing for cleaner look
fprintf('%8s %8s %8s %8s %8s %8s\n', 'Roll', 'Pitch', 'Yaw', 'ThrustZ', 'ForceX', 'ForceY');
for m = 1:N_MOTORS
    fprintf('M%d: %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n', ...
        m, Mixer_Final(m,1), Mixer_Final(m,2), Mixer_Final(m,3), ...
        Mixer_Final(m,4), Mixer_Final(m,5), Mixer_Final(m,6));
end

disp(' ');
disp('Note: Values are normalized. 1.0000 = This motor works hardest for this axis.');