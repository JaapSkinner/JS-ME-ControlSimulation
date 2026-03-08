function z = test_MeasurementFcn(x)
% test_MeasurementFcn - A simple function that "measures" the 2D position.
%
% State vector 'x': [position_x; velocity_x; position_y; velocity_y]
% Measurement vector 'z': [measured_x; measured_y]
% We are measuring the first and third elements of the state vector.
pos_x = x(1);
pos_y = x(3);

% Assemble the measurement vector
z = [pos_x; pos_y];
    
end
