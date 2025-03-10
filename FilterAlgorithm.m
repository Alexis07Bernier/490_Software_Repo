clc; clear; close all;

%Load .mat file
data = load('snap01.mat'); %this is one of the files from ALI
timeVar = data.TimeVar; % Assuming timeVar is the variable name in .mat

% Convert to seconds
time = seconds(timeVar); % Convert duration to seconds

%% Process Raw Data
rawData = data.RawData;
numRowsRawData = size(rawData, 1); % Get total number of rows in rawData

% Initialize empty matrices for storing results
AccelerometerValues = [];
GyroscopeValues = [];
MagnetometerValues = [];
HighGyroscopeValues = [];

for i = 1:numRowsRawData
    % Extract the first 210 columns of the current row and reshape
    fullData = reshape(rawData(i, 1:210), [21, 10])';

    % Process each row's data
    Accel_processed = processAccel(fullData);
    Gyro_processed = processGyro(fullData);
    Mag_processed = processMag(fullData);
    HighGyro_processed = processHighGyro(fullData);

    % Append results to the respective matrices
    AccelerometerValues = [AccelerometerValues; Accel_processed];
    GyroscopeValues = [GyroscopeValues; Gyro_processed];
    MagnetometerValues = [MagnetometerValues; Mag_processed];
    HighGyroscopeValues = [HighGyroscopeValues; HighGyro_processed];
end

FinalAccel = AccelerometerValues - mean(AccelerometerValues(1:100, :), "omitnan") + [0 0 9.81];
FinalGyro = (GyroscopeValues - mean(GyroscopeValues(1:100, :), "omitnan")) .* [1 -1 -1];
FinalMag = MagnetometerValues .* [1 -1 1];
FinalHighGyro = (HighGyroscopeValues - mean(HighGyroscopeValues(1:100, :), "omitnan")) * -1;

% Fill missing data using linear interpolation
AccelerometerValues = fillmissing(AccelerometerValues, 'linear');
GyroscopeValues = fillmissing(GyroscopeValues, 'linear');
MagnetometerValues = fillmissing(MagnetometerValues, 'linear');
HighGyroscopeValues = fillmissing(HighGyroscopeValues, 'linear');

%% Initialize Madgwick Filter
madgwick = MadgwickAHRS('SamplePeriod', 1/400, 'Beta', 0.1);

% Initialize quaternions
quaternions = repmat([1, 0, 0, 0], length(numRowsRawData), 1); %Allocate for quaternions

%% Main Loop - Apply Filters
numRowsProcessed = size(AccelerometerValues,1);
velocityArray = zeros(numRowsProcessed, 1); % Initialize the array for the velocities
highZCounter = 0;
threshold = 50; % Define Z-axis gyroscope threshold

for i = 1:numRowsProcessed
    % Ensure sensor data is a row vector (1x3)
    gyro_sample = FinalGyro(i, :);
    accel_sample = FinalAccel(i, :);
    mag_sample = FinalMag(i, :);
    
    % Check if Z-axis exceeds threshold
    if abs(gyro_sample(3)) > threshold
        highZCounter = highZCounter + 1; % Increase counter
    else
        highZCounter = 0; % Reset counter if condition is not met
    end

    % If threshold exceeded for 5+ consecutive samples, replace Z-axis
    if highZCounter >= 5
        fprintf("High Z-axis gyro detected at index %d. Replacing with high-rate gyro.\n", i);
        gyro_sample(3) = FinalHighGyro(i); % Replace Z-axis with high-rate gyro
    end

    % Update filter and store quaternion
    madgwick.Update(gyro_sample, accel_sample, mag_sample);
    quaternions(i, :) = madgwick.Quaternion; % Ensure it's a row vector

    velocityArray(i) = estimateVelocity(quaternions(i, :), accel_sample, gyro_sample, mag_sample);

end

peakVelocity = max(velocityArray);
fprintf('Peak Velocity: %.2f m/s\n', peakVelocity);

%% Functions
function a = processAccel(data)
    a = swapbytes(typecast(reshape(uint8(data(:, 1:6)'), 1, []), "uint16"'));
    a = reshape(a, [3, 10])';
    a = double(a) * 4096 / 65535;
    a = a ./ [12.279 12.037 13.999];
    a = a * 9.81;  
end
        
function g = processGyro(data)
    g = swapbytes(typecast(reshape(uint8(data(:, 7:12)'), 1, []), "int16"));
    g = reshape(g, [3, 10])';
    g = double(g) * 4000 / 32768;
    g = deg2rad(g);
end

function hg = processHighGyro(data)
    hg = swapbytes(typecast(reshape(uint8(data(:, 13:14)'), 1, []), "uint16"));
    hg = reshape(hg, [1, 10])';
    hg = double(hg) * 4096 / 65535;
    hg = hg / 0.0955;
    hg = deg2rad(hg);
end

function m = processMag(data) 
    m = swapbytes(typecast(reshape(uint8(data(:, 15:20)'), 1, []), "uint16"));
    m = reshape(m, [3, 10])';
    m = uint32(m);
    m(:, 1) = bitor(bitsll(m(:,1),2), uint32(bitsrl(bitand(uint8(data(:, 21)), 0xC0), 6)));
    m(:, 2) = bitor(bitsll(m(:,2),2), uint32(bitsrl(bitand(uint8(data(:, 21)), 0x30), 4)));
    m(:, 3) = bitor(bitsll(m(:,3),2), uint32(bitsrl(bitand(uint8(data(:, 21)), 0x0C), 2)));
    m = double(m) - 131072;
    m = m / 16384;
    m = m * 100;
end

function velocityValue = estimateVelocity(madgwick_quat, accel, gyro, mag)
    persistent fuse;
    
    if isempty(fuse)
        fuse = insfilterMARG;
        fuse.IMUSampleRate = 400;
        fuse.StateCovariance = 1e-12 * ones(22);
    end

    % Set Kalman filter's orientation from Madgwick quaternion
    fuse.State(1:4) = madgwick_quat'; % Apply Madgwick’s orientation

    % Predict state based on accelerometer and gyroscope
    fuse.predict(accel, gyro);

    % Magnetometer fusion     
    fuse.fusemag(mag, eye(3)*1e-12);

    % Get velocity estimate
    [~, ~, velocity] = pose(fuse);

    % Compute speed magnitude
    velocityValue = norm(velocity(1:3));

    % Optionally store velocity for debugging
    fprintf('Velocity: %.2f m/s\n', velocityValue);
end


%% Plot Results
% figure;
% subplot(3,1,1);
% plot(time, velocity(:,1), 'r'); title('Velocity X'); xlabel('Time (s)'); ylabel('m/s');
% subplot(3,1,2);
% plot(time, velocity(:,2), 'g'); title('Velocity Y'); xlabel('Time (s)'); ylabel('m/s');
% subplot(3,1,3);
% plot(time, velocity(:,3), 'b'); title('Velocity Z'); xlabel('Time (s)'); ylabel('m/s');

% Plot velocityArray over time
figure; % Create a new figure window
t = 0:2.5e-3:(size(velocityArray, 1)-1)/400;
plot(t, velocityArray);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity over Time');
grid on;

disp('Kalman filter processing complete.');
