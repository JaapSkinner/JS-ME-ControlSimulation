function metrics = evaluateUAVPerformance(simOut)
% Extracts performance metrics from simulation output (only between t=15s and t=50s)

    % Trimmed timeseries between 15s and 50s
    xref    = simOut.xref.getsampleusingtime(15, 50);
    xi      = simOut.xi.getsampleusingtime(15, 50);
    etaref  = simOut.etaref.getsampleusingtime(15, 50);
    eta     = simOut.eta.getsampleusingtime(15, 50);
    tau_des = simOut.tau_des.getsampleusingtime(15, 50);
    tau     = simOut.tau.getsampleusingtime(15, 50);
    T_des   = simOut.T_des.getsampleusingtime(15, 50);
    T       = simOut.T.getsampleusingtime(15, 50);

    % Data
    pos_act   = xi.Data;
    att_ref   = etaref.Data;
    att_act   = eta.Data;
    tau_cmd   = squeeze(tau_des.Data)';   % (N x 3)
    tau_act   = tau.Data;
    thrust_cmd = T_des.Data(:,3);
    thrust_act = T.Data(:,3);

    % Y-axis deviation RMS
    y_error = pos_act(:,2);
    yRMS = sqrt(mean(y_error.^2));

    % Attitude error RMS
    att_error = att_ref - att_act;
    rotErrorRMS = sqrt(mean(sum(att_error.^2, 2)));

    % Thrust error mean
    thrustError = abs(thrust_cmd - thrust_act);
    thrustErrorRMS = sqrt(mean(thrustError.^2));

    % Torque RMS error
    tau_error = tau_cmd - tau_act;
    torqueErrorRMS = sqrt(mean(sum(tau_error.^2, 2)));

    % Output
    metrics.YDeviationRMS      = yRMS;
    metrics.RotationErrorRMS   = rotErrorRMS;
    metrics.ThrustErrorMean    = thrustErrorMean;
    metrics.TorqueErrorRMS     = torqueErrorRMS;
end
