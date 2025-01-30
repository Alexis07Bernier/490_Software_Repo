clc; clear; close all;

%% Load Data from Excel
filename = 'sensor_data.xlsx'; % Replace with actual file name
data = readtable(filename);

% Extract columns
time = data.Time; % Time in seconds
dt = mean(diff(time)); % Compute time step

% Accelerometer (m/s^2)
accel_local = [data.AccelX, data.AccelY, data.AccelZ];

% Gyroscope (rad/s)
gyro = [data.GyroX, data.GyroY, data.GyroZ];

%% Initialize Madgwick Filter
fs = 1/dt; % Sampling frequency
madgwick = madgwickAHRS('SamplePeriod', dt, 'Beta', 0.1);

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
    R = quat2rotm(q); % Convert quaternion to rotation matrix
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
