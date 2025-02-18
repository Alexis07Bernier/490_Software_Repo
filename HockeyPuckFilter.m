clc; clear; close all;

%Load .mat file
data = load('iceimpact01.mat'); %this is one of the files from ALI
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


% Accelerometer (m/s^2)
accel_local = data.Accelerometer; %[data.AccelX, data.AccelY, data.AccelZ];

% Gyroscope (rad/s)
gyro = data.Gyroscope; %[data.GyroX, data.GyroY, data.GyroZ];

%% Initialize Madgwick Filter
fs = 1/dt; % Sampling frequency
madgwick = MadgwickAHRS('SamplePeriod', dt, 'Beta', 0.1);

% Initialize quaternions
num_samples = length(time);
quaternions = zeros(num_samples, 4);
velocity = zeros(num_samples, 3);

%% Kalman Filter Initialization
A = eye(3); % State transition matrix
B = dt * eye(3); % Control matrix (for acceleration input)
H = eye(3); % Measurement matrix (assume direct observation of velocity)

Q = 0.01 * eye(3); % Process noise covariance
R = 0.1 * eye(3); % Measurement noise covariance
P = eye(3); % Initial covariance matrix

x_est = zeros(3,1); % Initial state (velocity estimate)
g = [0; 0; -9.81]; % Gravity vector in global frame

%% Main Loop - Apply Filters
for i = 1:num_samples
    % Update Madgwick filter with gyro + accel
    madgwick.UpdateIMU(gyro(i, :), accel_local(i, :));
    q = madgwick.Quaternion;
    quaternions(i, :) = q;
    
    % Convert acceleration to global frame
    %R = quat2rotm(q); 
    %Need the quat2rotm function but not available on MATLAB online
    %Implement it manually
    R = [1 - 2*(q(3)^2 + q(4)^2),  2*(q(2)*q(3) - q(1)*q(4)),  2*(q(2)*q(4) + q(1)*q(3));
     2*(q(2)*q(3) + q(1)*q(4)),  1 - 2*(q(2)^2 + q(4)^2),  2*(q(3)*q(4) - q(1)*q(2));
     2*(q(2)*q(4) - q(1)*q(3)),  2*(q(3)*q(4) + q(1)*q(2)),  1 - 2*(q(2)^2 + q(3)^2)];
    
     % Convert quaternion to rotation matrix
    accel_global = (R * accel_local(i, :)')'; 
    
    % Remove gravity
    accel_motion = accel_global - g';

    % Kalman Prediction Step
    x_pred = A * x_est + B * accel_motion'; % Predicted state
    P_pred = A * P * A' + Q; % Predicted covariance
    
    % Measurement (Velocity from Acceleration Integration)
    velocity_meas = x_est' + accel_motion * dt;
    
    % Kalman Update Step
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    x_est = x_pred + K * (velocity_meas' - H * x_pred); % Update estimate
    P = (eye(3) - K * H) * P_pred; % Update covariance
    
    % Store velocity estimate
    velocity(i, :) = x_est';
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
