NUMLOOPS = 100;
clc;

for i = 1:NUMLOOPS
    % Disable warnings and Simulink output
    warningState = warning('off', 'all');
    evalc('[~] = runSimulationVariedFcn();');  % captures all printed output silently
    warning(warningState);                     % restore warning state

    fprintf('... Progress: %.0f%% (%d / %d Simulations Run)\n', ...
        (i/NUMLOOPS)*100, i, NUMLOOPS)
end
