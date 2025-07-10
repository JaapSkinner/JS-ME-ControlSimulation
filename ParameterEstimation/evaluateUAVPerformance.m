function metrics = evaluateUAVPerformance(simOut)
% Extracts performance metrics from simulation output

    % Get logs
    logsout = simOut.get('logsout');

    % Cross-track & rotation error
    pos_ref = logsout.get('xref').Values;      % Desired position [x y z]
    pos_act = logsout.get('xi').Values;        % Actual position

    att_ref = logsout.get('etaref').Values;    % Desired [roll pitch yaw]
    att_act = logsout.get('eta').Values;       % Actual attitude

    % Torque & thrust
    tau_cmd = logsout.get('tau_des').Values;   % Commanded torque
    tau_act = logsout.get('tau').Values;       % Actual torque

    thrust_cmd = logsout.get('T_des').Values;  % Commanded total thrust
    thrust_act = logsout.get('T').Values;      % Actual total thrust

    % Time range
    t = pos_ref.Time;
    t_start = t(1);
    t_end = t(end);

    % Cross-track error (Euclidean norm)
    pos_error = pos_ref.Data - pos_act.Data;
    att_error = att_ref.Data - att_act.Data;

    crossTrackRMS = sqrt(mean(sum(pos_error.^2, 2)));
    rotErrorRMS   = sqrt(mean(sum(att_error.^2, 2)));

    % Thrust error (absolute mean error)
    thrustError = abs(thrust_cmd.Data - thrust_act.Data);
    thrustErrorMean = mean(thrustError);

    % Torque error (Euclidean)
    tau_error = tau_cmd.Data - tau_act.Data;
    torqueErrorRMS = sqrt(mean(sum(tau_error.^2, 2)));

    % Output struct
    metrics.CrossTrackRMS     = crossTrackRMS;
    metrics.RotationErrorRMS  = rotErrorRMS;
    metrics.ThrustErrorMean   = thrustErrorMean;
    metrics.TorqueErrorRMS    = torqueErrorRMS;
end
