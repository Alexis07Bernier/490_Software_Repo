classdef PuckSensorApp < matlab.apps.AppBase

% Properties that correspond to app components
properties (Access = public) ...

properties (Access = private) ...

methods (Access = private)

    function readCharacteristicData(app, src, ~)
        d = read(src, 'oldest');
        app.Data = [app.Data; d];
        processCounter(app, d);
        d = reshape(d(1:210), [21, 10])';
        processAccel(app, d);
        processGyro(app, d);
        processHighGyro(app, d);
        processMag(app, d);
        app.Time = [app.Time; datetime('now', ...

    'Format', 'dd-MMM-uuuu HH:mm:ss.SSS')];
    end

    function processCounter(app, data)
        c = double(swapbytes(typecast(uint8(data(211:212)), 'uint16')));
        app.Counter = [app.Counter; c];
    end

    function processAccel(app, data)
        a = swapbytes(typecast(reshape( ...
        uint8(data(:, 1:6)'), 1, []), 'uint16''));
        a = reshape(a, [3, 10])';
        a = double(a) * 4096 / 65535;
        switch app.DeviceName
            case 'Puck Sensor 1'
                a = a ./ [12.279 12.037 13.999];
            case 'Puck Sensor 2'
                a = a ./ [12.130 12.555 14.329];
        end

        a = a * 9.81;
        app.Accel = [app.Accel; ...
            NaN([(app.Counter(end)-app.Counter(end-1)-1)*10 3]); a];
    end


    function processGyro(app, data)
        g = swapbytes(typecast(reshape( ...
        uint8(data(:, 7:12)'), 1, []), 'int16'));
        g = reshape(g, [3, 10])';
        g = double(g) * 4000 / 32768;
        g = deg2rad(g);
        app.Gyro = [app.Gyro; ...
            NaN([(app.Counter(end)-app.Counter(end-1)-1)*10 3]); g];
    end

    function processHighGyro(app, data)
        hg = swapbytes(typecast(reshape( ...
        uint8(data(:, 13:14)'), 1, []), 'uint16'));
        hg = reshape(hg, [1, 10])';
        hg = double(hg) * 4096 / 65535;
        hg = hg / 0.0796;
        hg = deg2rad(hg);
        app.HighGyro = [app.HighGyro; ...
            NaN([(app.Counter(end)-app.Counter(end-1)-1)*10 1]); hg];
    end

    function processMag(app, data)
        m = swapbytes(typecast(reshape( ...
        uint8(data(:, 15:20)'), 1, []), 'uint16');
        m = reshape(m, [3, 10])';
        m = uint32(m);
        m(:, 1) = bitor(bitsll(m(:,1),2), ...
        uint32(bitsrl(bitand(uint8(data(:, 21)), 0xC0), 6)));
        m(:, 2) = bitor(bitsll(m(:,2),2), ...
        uint32(bitsrl(bitand(uint8(data(:, 21)), 0x30), 4)));
        m(:, 3) = bitor(bitsll(m(:,3),2), ...
        uint32(bitsrl(bitand(uint8(data(:, 21)), 0x0C), 2)));
        m = double(m) - 131072;
        m = m / 16384;
        m = m * 100;
        app.Mag = [app.Mag; ...
            NaN([(app.Counter(end)-app.Counter(end-1)-1)*10 3]); m];
    end

    function estimateVelocity(app)
        app.Vel = [];
        app.Ornt = quaternion();
        fuse = insfilterMARG;
        fuse.IMUSampleRate = 400;
        fuse.StateCovariance = 1e-12 * eye(22);

        idx = find(abs(app.FinalGyro(:, 3)) > 69);
        if (length(idx) >= 5)
            temp = app.FinalGyro(:, 3);
            app.FinalGyro(:, 3) = app.FinalHighGyro;
        end

        for ii = 1:size(app.FinalAccel, 1)
            fuse.predict(app.FinalAccel(ii, :), app.FinalGyro(ii, :));
            if ~mod(ii, 200)
                fuse.fusemag(app.FinalMag(ii, :), ...
                cov(app.FinalMag(1:100, :)));
            end
            [~, app.Ornt(ii), app.Vel(ii, :)] = pose(fuse);
        end
        app.FinalGyro(:, 3) = temp;

        for ii = 1:size(app.Vel, 1)
            app.Vel(ii, 4) = norm(app.Vel(ii, 1:3));
        end
    end

    function plotData(app)
        t = 0:2.5e-3:(size(app.Vel, 1)-1)/400;
        plot(app.UIAxes, t, app.Vel(:, 1:3))
        legend(app.UIAxes, {'X Axis', 'Y Axis', 'Z Axis'})
        plot(app.UIAxes2, t, app.Vel(:, 4))
        text(app.UIAxes2, 'String', sprintf(...
            'Maximum Estimated Velocity: %.3f m/s', max(app.Vel(:,4))), ...
            'Units', 'normalized', 'Position', [0.025 0.9], ...
            'VerticalAlignment','top', 'EdgeColor','k', ...
            'BackgroundColor','w');
        plot(app.UIAxes3, t, rad2deg(quat2eul(app.Ornt)))
        legend(app.UIAxes3, {'X Axis', 'Y Axis', 'Z Axis'})
    end
end


% Callbacks that handle component events
methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.Label.Text = 'Waiting for Search...';
        end

        % Button pushed function: SearchButton
        function SearchButtonPushed(app, event)
            device = [];
            app.Label.Text = 'Searching for' + newline + 'the Puck Sensor...';
            while (isempty(device))
                device = blelist('Name', 'Puck Sensor');
            end
            app.DeviceName = device.Name;
            app.Label.Text = app.DeviceName + newline + 'detected';
            app.ConnectButton.Enable = 'on';
        end

        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            app.BLE = ble(app.DeviceName);
            app.BLEChar = characteristic(app.BLE, ...
                '0A361000-115B-4E3F-80E6-05D7F5F2730E', ...
                '0A361001-115B-4E3F-80E6-05D7F5F2730E');
            app.BLEChar.DataAvailableFcn = @(src, event) ...
                app.readCharacteristicData(src, event);
            app.Label.Text = 'Connected to' + newline + app.DeviceName;
            app.SearchButton.Enable = 'off';
            app.ConnectButton.Enable = 'off';
            app.DisconnectButton.Enable = 'on';
            app.StartButton.Enable = 'on';
        end


        % Button pushed function: DisconnectButton
        function DisconnectButtonPushed(app, event)
            unsubscribe(app.BLEChar);
            disconnect(app.BLE);
            app.BLEChar = [];
            app.BLE = [];
            app.Time = [];
            app.Label.Text = 'Disconnected from' + newline + app.DeviceName;
            app.SearchButton.Enable = 'on';
            app.ConnectButton.Enable = 'on';
            app.DisconnectButton.Enable = 'off';
            app.StartButton.Enable = 'off';
        end

        % Button pushed function: StartButton
        function StartButtonPushed(app, event)
            app.Time = datetime('now', 'Format', 'dd-MMM-uuuu HH:mm:ss.SSS');
            app.Data = [];
            app.Counter = app.Counter(end);
            app.Accel = [];
            app.Gyro = [];
            app.HighGyro = [];
            app.Mag = [];
            app.Label.Text = 'Getting data from' + newline + app.DeviceName;
            app.StartButton.Enable = 'off';
            app.FinishButton.Enable = 'on';
        end

        % Button pushed function: FinishButton
        function FinishButtonPushed(app, event)
            app.FinalTime = app.Time;
            app.FinalData = app.Data;
            app.FinalCounter = app.Counter;
            app.FinalAccel = app.Accel - ...

                mean(app.Accel(1:100, :), 'omitnan') + [0 0 9.81];
            app.FinalGyro = (app.Gyro - ...
                mean(app.Gyro(1:100, :), 'omitnan')) .* [1 -1 -1];
            app.FinalHighGyro = (app.HighGyro - ...
                mean(app.HighGyro(1:100, :), 'omitnan')) * -1;
            app.FinalMag = app.Mag .* [1 -1 1];

            app.FinalAccel = fillmissing(app.FinalAccel, 'linear');
            app.FinalGyro = fillmissing(app.FinalGyro, 'linear');
            app.FinalHighGyro = fillmissing(app.FinalHighGyro, 'linear');
            app.FinalMag = fillmissing(app.FinalMag, 'linear');

            estimateVelocity(app);
            plotData(app);

            app.Label.Text = 'Elapsed time:' + newline + ...
                string(app.FinalTime(end) - app.FinalTime(1), 'mm:ss.SSS');
            app.StartButton.Enable = 'on';
            app.FinishButton.Enable = 'off';
            app.SaveButton.Enable = 'on';
        end

        % Button pushed function: SaveButton
        function SaveButtonPushed(app, event)
            TimeVar = duration(app.FinalTime - app.FinalTime(1), ...
            'Format', 'mm:ss.SSS'); %#ok
            CounterVar = app.FinalCounter - app.Counter(1); %#ok
            RawData = app.FinalData; %#ok
            Accelerometer = app.FinalAccel; %#ok
            Gyroscope = [app.FinalGyro app.FinalHighGyro]; %#ok
            Magnetometer = app.FinalMag; %#ok
            Velocity = app.Vel; %#ok
            Orientation = app.Ornt'; %#ok
            uisave({'TimeVar', ’CounterVar', 'RawData', 'Accelerometer', ...
                'Gyroscope', 'Magnetometer', 'Velocity', 'Orientation'})
        end

        % Button pushed function: ExitButton
        function ExitButtonPushed(app, event)
            delete(app)
        end
    
    end
    % Component initialization
    methods (Access = private) ...
    
    % App creation and deletion
    methods (Access = public) ...
end