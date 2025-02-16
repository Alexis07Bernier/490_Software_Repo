clc; clear; close all;
%from ahrs.filters import Madgwick

%Load .mat file
data = load('slap01.mat'); %this is one of the files from ALI
timeVar = data.TimeVar; % Assuming timeVar is the variable name in .mat

% Convert string times to duration (if stored as strings)
if iscell(timeVar) % Check if stored as cell array (e.g., due to undefined values)
    timeVar = timeVar(~strcmp(timeVar, 'undefined')); % Remove undefined
    timeVar = duration(timeVar, 'InputFormat', 'hh:mm:ss.SSS'); % Convert to duration
elseif isstring(timeVar) || ischar(timeVar) % If stored as string array
    timeVar = duration(timeVar, 'InputFormat', 'hh:mm:ss.SSS');
end

% Convert to seconds
time = seconds(timeVar); % Convert duration to seconds

% Compute time step (dt)
dt = mean(diff(time)); % Mean time difference between samples

% Display dt
disp(['Computed dt: ', num2str(dt), ' seconds']);

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

%% Initialize Madgwick Filter
madgwick = MadgwickTEST();

% Initialize quaternions
num_samples = length(time);
quaternions = repmat([1, 0, 0, 0], length(gyro), 1); %Allocate for quaternions

% FinalAccel = Accel_processed - mean(Accel_processed(1:100, :), "omitnan") + [0 0 9.81];
% FinalGyro = (Gyro_processed - mean(Gyro_processed(1:100, :), "omitnan")) .* [1 -1 -1];
% FinalMag = Mag_processed .* [1 -1 1];

%% Main Loop - Apply Filters
for i = 1:num_samples
    % Ensure sensor data is a row vector (1x3)
    gyro_sample = Gyro_processed(i, :);
    accel_sample = Accel_processed(i, :);
    mag_sample = Mag_processed(i, :);
    
    % Update filter and store quaternion
    madgwick.update(gyro_sample, accel_sample, mag_sample);
    quaternions(i, :) = madgwick.qEst; % Ensure it's a row vector
    disp(size(quaternions))

    estimateVelocity(quaternions(i, :), accel_sample, gyro_sample, mag_sample);

end

%% Functions
function a = processAccel(data)
    a = swapbytes(typecast(reshape(uint8(data(:, 1:6)'), 1, []), "uint16"'));
    %a = reshape(a, [3, 10])';
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

function estimateVelocity(madgwick_quat, accel, gyro, mag)
    persistent fuse;
    disp(size(madgwick_quat))
    
    if isempty(fuse)
        fuse = insfilterMARG;
        fuse.IMUSampleRate = 400;
        fuse.StateCovariance = 1e-12 * eye(22);
    end

    % Set Kalman filter's orientation from Madgwick quaternion
    fuse.State(1:4) = madgwick_quat'; % Apply Madgwick’s orientation

    % Predict state based on accelerometer and gyroscope
    fuse.predict(accel, gyro);

    % Magnetometer fusion every 200 samples
    if mod(size(fuse.State, 2), 200) == 0
        fuse.fusemag(mag, cov(mag));
    end

    % Get velocity estimate
    [~, ~, velocity] = pose(fuse);

    % Compute speed magnitude
    velocity(4) = norm(velocity(1:3));

    % Optionally store velocity for debugging
    fprintf('Velocity: %.2f m/s\n', velocity(4));
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
