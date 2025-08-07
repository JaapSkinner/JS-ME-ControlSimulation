paramList = {
    'K_V', true;        % perMotor (std_across_motors + mean_deviation)
    'K_E', true;
    'B', true;
    'Volt_offset', true;
    'volt_slope', true;
    'R', true;
    'I_0', true;
    'D_UAV', true;
    'D_PROP', true;
    'M', false;         % scalar only
    'I', false;         % vector per_axis_deviation
    'RHO_AIR', false;
    'R_PROP', false;
    'A_UAV', false;
    'A_PROP', false;
    'ZETA', false;
};

figure('Position', [100 100 1200 900]);

t = tiledlayout(10, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

plotIdx = 1;

for k = 1:size(paramList,1)
    field = paramList{k,1};
    isPerMotor = paramList{k,2};
    
    if isPerMotor
        % Plot std_across_motors
        nexttile(plotIdx);
        vals = arrayfun(@(f) f.(field).std_across_motors, F);
        histogram(vals);
        title([field ' std across motors']);
        xlabel('Std Dev'); ylabel('Count');
        plotIdx = plotIdx + 1;

        % Plot mean deviation
        nexttile(plotIdx);
        vals = arrayfun(@(f) f.(field).mean_deviation, F);
        histogram(vals);
        title([field ' mean deviation']);
        xlabel('\Delta'); ylabel('Count');
        plotIdx = plotIdx + 1;
    else
        % Check if vector (per_axis_deviation)
        sample = F(1).(field);
        if isfield(sample, 'per_axis_deviation')
            vals_mat = reshape([vertcat(F.(field)).per_axis_deviation], 3, [])';
            for ax = 1:3
                nexttile(plotIdx);
                histogram(vals_mat(:,ax));
                title([field ' axis ' num2str(ax) ' deviation']);
                xlabel('\Delta'); ylabel('Count');
                plotIdx = plotIdx + 1;
            end
        else
            % Scalar only
            vals = arrayfun(@(f) f.(field).mean_deviation, F);
            nexttile(plotIdx);
            histogram(vals);
            title([field ' mean deviation']);
            xlabel('\Delta'); ylabel('Count');
            plotIdx = plotIdx + 1;
        end
    end
end

sgtitle('Monte Carlo Parameter Variation Distributions');
