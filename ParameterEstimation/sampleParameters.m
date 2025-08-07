function [Motor, Uav, variationFeatures] = sampleParameters(Motor_nom, Uav_nom, variationPercent, distTypes, N_motors)
    if isempty(distTypes)
        distTypes = repmat({'normal'}, 16, 1);
    end

    Motor = Motor_nom;
    Uav = Uav_nom;

    variationFeatures = struct();

    paramList = {
        'Motor', 'K_V';
        'Motor', 'K_E';
        'Motor', 'B';
        'Motor', 'Volt_offset';
        'Motor', 'volt_slope';
        'Motor', 'R';
        'Motor', 'I_0';
        'Uav',   'D_UAV';
        'Uav',   'D_PROP';
        'Uav',   'M';
        'Uav',   'I';
        'Uav',   'RHO_AIR';
        'Uav',   'R_PROP';
        'Uav',   'A_UAV';
        'Uav',   'A_PROP';
        'Uav',   'ZETA';
    };

    % Parameters that have N_motors elements
    perMotorParams = ["K_V", "K_E", "B", "Volt_offset", "volt_slope", "R", "I_0", "D_UAV", "D_PROP"];

    % 3-element vector params
    vector3Params = ["I", "COM"];

    for i = 1:length(paramList)
        structName = paramList{i,1};
        fieldName  = paramList{i,2};
        distType   = distTypes{i};
        percent    = variationPercent(i)/100;

        nominal = eval([structName '_nom.' fieldName]);
        sigma   = abs(nominal) * percent;

        % Sample
        switch distType
            case 'normal'
                sampled = nominal + sigma .* randn(size(nominal));
            case 'uniform'
                sampled = nominal + (2 * rand(size(nominal)) - 1) .* sigma;
            otherwise
                error(['Unknown distribution: ' distType]);
        end

        % Assign sampled
        eval([structName '.' fieldName ' = sampled;']);

        % Deviation metrics
        delta = sampled - nominal;
        v = struct();
        v.mean_deviation = mean(abs(delta(:)));
        v.max_deviation = max(abs(delta(:)));

        if any(perMotorParams == fieldName)
            v.std_across_motors = std(sampled(:));
            v.range = max(sampled(:)) - min(sampled(:));
        elseif strcmp(fieldName, 'I')
            % Extract diagonal elements deviation only for inertia matrix
            deltaDiag = diag(delta)';
            v.per_axis_deviation = deltaDiag;  % 1x3 vector
        elseif strcmp(fieldName, 'COM')
            % For COM, just difference vector
            v.per_axis_deviation = delta(:)'; % ensure 1x3 vector
        end



        variationFeatures.(fieldName) = v;
    end
end
