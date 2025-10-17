% --- Copy and Paste these commands into your MATLAB Console ---

disp('--- Verifying UKF Variable Dimensions ---');

% Check the core size calculation
fprintf('n_total (Expected: 61 for 8 motors): ');
disp(n_total);

% Check the size of the Initial State vector
fprintf('Size of UKF.InitialState (Expected: 61 x 1): ');
disp(size(UKF.InitialState));

% Check the size of the Initial Covariance matrix
fprintf('Size of UKF.InitialCovariance (Expected: 61 x 61): ');
disp(size(UKF.InitialCovariance));

% Check the size of the Process Noise matrix
fprintf('Size of UKF.ProcessNoise (Expected: 61 x 61): ');
disp(size(UKF.ProcessNoise));

% Check the size of the Measurement Noise matrix
fprintf('Size of UKF.MeasurementNoise (Expected: 13 x 13): ');
disp(size(UKF.MeasurementNoise));

disp('--- Verification Complete ---');