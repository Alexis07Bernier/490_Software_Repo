clc; clear; close all;

%Load .mat file
data = load('slap01.mat'); %this is one of the files from ALI
timeVar = data.TimeVar; % Assuming timeVar is the variable name in .mat

% Convert to seconds
time = seconds(timeVar); % Convert duration to seconds

%% Process Raw Data
rawData = data.RawData;

% Accelerometer
accel = data.RawData(:, 1:6); 
% Gyroscope
gyro = data.RawData(:, 7:12);
% Magnetometer
mag = data.RawData(:, 15:20);

Accel_processed = processAccel(rawData);
Gyro_processed = processGyro(rawData);
Mag_processed = processMag(rawData);

% Fill missing data using linear interpolation
Accel_processed = fillmissing(Accel_processed, 'linear');
Gyro_processed = fillmissing(Gyro_processed, 'linear');
Mag_processed = fillmissing(Mag_processed, 'linear');

FinalAccel = Accel_processed - mean(Accel_processed(1:100, :), "omitnan") + [0 0 9.81];
FinalGyro = (Gyro_processed - mean(Gyro_processed(1:100, :), "omitnan")) .* [1 -1 -1];
FinalMag = Mag_processed .* [1 -1 1];

%% Initialize Madgwick Filter
madgwick = MadgwickAHRS('SamplePeriod', 1/40, 'Beta', 0.1);

% Initialize quaternions
num_samples = length(rawData);
quaternions = repmat([1, 0, 0, 0], length(gyro), 1); %Allocate for quaternions

%% Main Loop - Apply Filters
velocityArray = zeros(num_samples, 1); % Initialize the array for the velocities

for i = 1:num_samples
    % Ensure sensor data is a row vector (1x3)
    gyro_sample = FinalGyro(i, :);
    accel_sample = FinalAccel(i, :);
    mag_sample = FinalMag(i, :);
    
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
    a = reshape(a, [], 3); 
    a = double(a) * 4096 / 65535;
    a = a * 9.81;  
end
        
function g = processGyro(data)
    g = swapbytes(typecast(reshape(uint8(data(:, 7:12)'), 1, []), "int16"));
    g = reshape(g, [], 3);
    g = double(g) * 4000 / 32768;
    g = deg2rad(g);
end

function m = processMag(data) 
    m = swapbytes(typecast(reshape(uint8(data(:, 15:20)'), 1, []), "uint16"));
    m = reshape(m, [], 3);
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
        fuse.IMUSampleRate = 40;
        fuse.StateCovariance = 1e-9 * ones(22);
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
figure;
subplot(3,1,1);
plot(time, velocity(:,1), 'r'); title('Velocity X'); xlabel('Time (s)'); ylabel('m/s');
subplot(3,1,2);
plot(time, velocity(:,2), 'g'); title('Velocity Y'); xlabel('Time (s)'); ylabel('m/s');
subplot(3,1,3);
plot(time, velocity(:,3), 'b'); title('Velocity Z'); xlabel('Time (s)'); ylabel('m/s');

disp('Kalman filter processing complete.');
