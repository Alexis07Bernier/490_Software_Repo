%This was for plotting the raw data to see if there's an issue there
data = load('slap02.mat'); %this is one of the files from ALI
timeVar = data.TimeVar; % Assuming timeVar is the variable name in .mat

% Convert to seconds
time = seconds(timeVar); % Convert duration to seconds
time = time(1:end-1);
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

%% Plot raw data
%% Plot Accelerometer Data
figure;
subplot(3,1,1);
plot(time, Accel_processed(:,1), 'r'); title('Accelerometer X'); xlabel('Time (s)'); ylabel('m/s²');
subplot(3,1,2);
plot(time, Accel_processed(:,2), 'g'); title('Accelerometer Y'); xlabel('Time (s)'); ylabel('m/s²');
subplot(3,1,3);
plot(time, Accel_processed(:,3), 'b'); title('Accelerometer Z'); xlabel('Time (s)'); ylabel('m/s²');

%% Plot Gyroscope Data
figure;
subplot(3,1,1);
plot(time, Gyro_processed(:,1), 'r'); title('Gyroscope X'); xlabel('Time (s)'); ylabel('deg/s');
subplot(3,1,2);
plot(time, Gyro_processed(:,2), 'g'); title('Gyroscope Y'); xlabel('Time (s)'); ylabel('deg/s');
subplot(3,1,3);
plot(time, Gyro_processed(:,3), 'b'); title('Gyroscope Z'); xlabel('Time (s)'); ylabel('deg/s');

%% Plot Magnetometer Data
figure;
subplot(3,1,1);
plot(time, Mag_processed(:,1), 'r'); title('Magnetometer X'); xlabel('Time (s)'); ylabel('μT');
subplot(3,1,2);
plot(time, Mag_processed(:,2), 'g'); title('Magnetometer Y'); xlabel('Time (s)'); ylabel('μT');
subplot(3,1,3);
plot(time, Mag_processed(:,3), 'b'); title('Magnetometer Z'); xlabel('Time (s)'); ylabel('μT');