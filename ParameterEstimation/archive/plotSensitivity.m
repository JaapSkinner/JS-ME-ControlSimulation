load('ParameterEstimation/sweep_combined_results.mat');  % loads combinedResults

metrics = {'YDeviationRMS', 'RotationErrorRMS', 'ThrustErrorMean', 'TorqueErrorRMS'};
metricLabels = {
    'Y Deviation RMS [m]', ...
    'Rotation Error RMS [rad]', ...
    'Thrust Error Mean [N]', ...
    'Torque Error RMS [Nm]'
};
paramNames = {
    'Motor.K_V';
    'Motor.R';
    'Motor.volt_slope';
    'Motor.Volt_offset';
    'Motor.B';
};

% Parameters to include in plots
activeParams = {'Motor.K_V', 'Motor.volt_slope', 'Motor.Volt_offset', 'Motor.B'};
activeIndices = find(ismember(paramNames, activeParams));
colors = lines(numel(activeParams));

figure;
t = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(t, 'Parameter Sensitivity Sweep');

for m = 1:numel(metrics)
    nexttile;
    hold on;
    grid on;

    for idx = 1:numel(activeIndices)
        i = activeIndices(idx);

        match = arrayfun(@(r) ...
            combinedResults.VarianceList{r}(i) > 0 && ...
            all(combinedResults.VarianceList{r}([1:i-1, i+1:end]) == 0), ...
            1:height(combinedResults));

        if ~any(match), continue; end

        values = arrayfun(@(r) combinedResults.VarianceList{r}(i), find(match));
        metricVals = combinedResults{match, metrics{m}};
        valid = ~isnan(metricVals);

        plot(values(valid), metricVals(valid), '-o', ...
            'DisplayName', paramNames{i}, ...
            'Color', colors(idx,:));
    end

    title(metricLabels{m}, 'Interpreter', 'none');
    xlabel('Percent Deviation');
    ylabel(metricLabels{m});
end

% Shared legend with only activeParams
lgd = legend(activeParams, 'Location', 'southoutside', 'Orientation', 'horizontal');
lgd.Layout.Tile = 'south';
