function [Motor, Uav, variationFeatures] = sampleParameters(Motor_nom, Uav_nom, variationPercent, distTypes, N_motors)
    % --- MODIFIED: Now 18 parameters ---
    if isempty(distTypes)
        distTypes = repmat({'normal'}, 18, 1);
    end
    Motor = Motor_nom;
    Uav = Uav_nom;
    variationFeatures = struct();
    
    % --- MODIFIED: Added C_TAU to the list ---
    paramList = {
        'Motor', 'K_V';
        'Motor', 'K_E';
        'Motor', 'C_TAU'; % <-- ADDED HERE
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
        'Uav',   'COM';
    };
    
    % --- MODIFIED: Added C_TAU to per-motor list ---
    perMotorParams = ["K_V", "K_E", "C_TAU", "B", "Volt_offset", "volt_slope", "R", "I_0", "D_UAV", "D_PROP"];

    % 3-element vector params
    vector3Params = ["I", "COM"];
    
    % --- First pass: COM shift ---
    comIdx = find(strcmp(paramList(:,2), 'COM'));
    if ~isempty(comIdx)
        structName = paramList{comIdx,1};
        fieldName  = paramList{comIdx,2};
        distType   = distTypes{comIdx};
        percent    = variationPercent(comIdx)/100;
        nominal = eval([structName '_nom.' fieldName]);
        absSigma = [0.001, 0.001, 0.03]; % 1mm, 1mm, 30mm in meters
        if (percent == 0)
            absSigma = [0 0 0];
        end
        sigma = max(abs(nominal) * percent, absSigma);
        switch distType
            case 'normal'
                sampled = nominal + sigma .* randn(size(nominal));
            case 'uniform'
                sampled = nominal + (2 * rand(size(nominal)) - 1) .* sigma;
            otherwise
                error(['Unknown distribution: ' distType]);
        end
        eval([structName '.' fieldName ' = sampled;']);
        delta = sampled - nominal;
        v = struct();
        v.per_axis_deviation = delta(:)'; % 1x3 [dx dy dz]
        variationFeatures.(fieldName) = v;
        
        % Shift motor positions
        Uav.MotorLoc(:,1:3) = Uav.MotorLoc(:,1:3) - repmat(delta(:)', size(Uav.MotorLoc,1), 1);
        
        % Recalculate distance to centroid (4th column)
        Uav.MotorLoc(:,4) = sqrt(sum(Uav.MotorLoc(:,1:3).^2, 2));
    end

    % --- Second pass: all other params ---
    for i = 1:length(paramList)
        fieldName  = paramList{i,2};
        if strcmp(fieldName, 'COM')
            continue; % already done above
        end
        structName = paramList{i,1};
        distType   = distTypes{i};
        percent    = variationPercent(i)/100;
        nominal = eval([structName '_nom.' fieldName]);
        sigma   = abs(nominal) * percent;
        switch distType
            case 'normal'
                sampled = nominal + sigma .* randn(size(nominal));
            case 'uniform'
                sampled = nominal + (2 * rand(size(nominal)) - 1) .* sigma;
            otherwise
                error(['Unknown distribution: ' distType]);
        end
        eval([structName '.' fieldName ' = sampled;']);
        delta = sampled - nominal;
        v = struct();
        v.mean_deviation = mean(abs(delta(:)));
        v.max_deviation = max(abs(delta(:)));
        if any(perMotorParams == fieldName)
            v.std_across_motors = std(sampled(:));
            v.range = max(sampled(:)) - min(sampled(:));
        elseif strcmp(fieldName, 'I')
            deltaDiag = diag(delta)';
            v.per_axis_deviation = deltaDiag;
        end
        variationFeatures.(fieldName) = v;
    end
end