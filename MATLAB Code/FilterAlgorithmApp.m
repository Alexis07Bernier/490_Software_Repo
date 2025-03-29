classdef FilterAlgorithmApp < matlab.apps.AppBase
    %% App Properties
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        GridLayout          matlab.ui.container.GridLayout
        LoadButton          matlab.ui.control.Button
        ProcessButton       matlab.ui.control.Button
        ConnectButton       matlab.ui.control.Button
        DisconnectButton    matlab.ui.control.Button
        SearchButton        matlab.ui.control.Button
        StartButton         matlab.ui.control.Button
        StopButton          matlab.ui.control.Button
        SaveButton          matlab.ui.control.Button
        CloseButton         matlab.ui.control.Button
        Axes                matlab.ui.control.UIAxes
        Label               matlab.ui.control.Label        
    end

    properties (Access = private)
        Counter;
        Data;
        DeviceName;
        BLE;
        BLEChar;
        Time;
        AccelerometerValues;
        GyroscopeValues;
        MagnetometerValues;
        HighGyroscopeValues;
        VelocityArray;
        FinalAccel;
        FinalGyro;
        FinalHighGyro;
        FinalMag;
    end
    
    %% Main Algorithm Functions
    methods (Access = private)
        function loadData(app)
            [file, path] = uigetfile('*.mat', 'Select Data File');
            if isequal(file, 0)
                return;
            end
            app.Label.Text = ['Loaded: ', file];
            app.Data = load(fullfile(path, file));
        end
        
        function processAccel(app, data)
            a = swapbytes(typecast(reshape(uint8(data(:, 1:6)'), 1, []), "uint16"'));
            a = reshape(a, [3, 10])'; 
            a = double(a) * 4096 / 65535;
            a = a ./ [12.279 12.037 13.999];
            a = a * 9.81;
            app.AccelerometerValues = [app.AccelerometerValues; a];
        end
                
        function processGyro(app, data)
            g = swapbytes(typecast(reshape(uint8(data(:, 7:12)'), 1, []), "uint16"));
            g = reshape(g, [3, 10])';
            g = double(g) * 4000 / 32768;
            g = deg2rad(g);
            app.GyroscopeValues = [app.GyroscopeValues; g];
        end
        
        function processHighGyro(app, data)
            hg = swapbytes(typecast(reshape(uint8(data(:, 13:14)'), 1, []), "uint16"));
            hg = reshape(hg, [1, 10])';
            hg = double(hg) * 4096 / 65535;
            hg = hg / 0.0955;
            hg = deg2rad(hg);
            app.HighGyroscopeValues = [app.HighGyroscopeValues; hg];
        end
        
        function processMag(app, data) 
            m = swapbytes(typecast(reshape(uint8(data(:, 15:20)'), 1, []), "uint16"));
            m = reshape(m, [3, 10])';
            m = uint32(m);
            m(:, 1) = bitor(bitsll(m(:,1),2), uint32(bitsrl(bitand(uint8(data(:, 21)), 0xC0), 6)));
            m(:, 2) = bitor(bitsll(m(:,2),2), uint32(bitsrl(bitand(uint8(data(:, 21)), 0x30), 4)));
            m(:, 3) = bitor(bitsll(m(:,3),2), uint32(bitsrl(bitand(uint8(data(:, 21)), 0x0C), 2)));
            m = double(m) - 131072;
            m = m / 16384;
            m = m * 100;
            app.MagnetometerValues = [app.MagnetometerValues; m];
        end

        function velocityValue = estimateVelocity(app, madgwick_quat, accel, gyro, mag)
            persistent fuse;
            
            if isempty(fuse)
                fuse = insfilterMARG;
                fuse.IMUSampleRate = 400;
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

        function processData(app, src, ~)
            % Reshape new data from the sensors into 10x21 matrix
            % Preprocess the data for each sensor
            sensorData = read(src, "oldest");
            app.Data = [app.Data; sensorData];
            processCounter(app, sensorData);
            data = reshape(sensorData(1:210), [21, 10])';
            processAccel(app, data);
            processGyro(app, data);
            processHighGyro(app, data);
            processMag(app, data);
            app.Time = [app.Time; datetime("now", "Format", "dd-MMM-uuuu HH:mm:ss.SSS")];
        end
            
        
        function getResults(app)   
            % Data corrections
            app.FinalAccel = app.AccelerometerValues - mean(app.AccelerometerValues(1:100, :), "omitnan") + [0 0 9.81];
            app.FinalGyro = (app.GyroscopeValues - mean(app.GyroscopeValues(1:100, :), "omitnan")) .* [1 -1 -1];
            app.FinalMag = app.MagnetometerValues .* [1 -1 1];
            app.FinalHighGyro = (app.HighGyroscopeValues - mean(app.HighGyroscopeValues(1:100, :), "omitnan")) * -1;
            
            % Initialize Madgwick filter
            madgwick = MadgwickAHRS('SamplePeriod', 1/40, 'Beta', 0.1);
            numRowsProcessed = size(app.FinalAccel, 1);
            app.VelocityArray = zeros(numRowsProcessed, 1);
            
            highZCounter = 0;
            threshold = 50;
            for i = 1:numRowsProcessed
                gyro_sample = app.FinalGyro(i, :);
                accel_sample = app.FinalAccel(i, :);
                mag_sample = app.FinalMag(i, :);
                
                if abs(gyro_sample(3)) > threshold
                    highZCounter = highZCounter + 1;
                else
                    highZCounter = 0;
                end
                
                if highZCounter >= 5
                    gyro_sample(3) = app.FinalHighGyro(i);
                end
                
                madgwick.Update(gyro_sample, accel_sample, mag_sample);
                app.VelocityArray(i) = estimateVelocity(app, madgwick.Quaternion, accel_sample, gyro_sample, mag_sample);
            end
            peakVelocity = max(app.VelocityArray);
            fprintf('Peak Velocity: %.2f m/s\n', peakVelocity);
            time = 0:2.5e-3:(size(app.VelocityArray, 1)-1)/400;
            app.plotResults(time, app.VelocityArray);

            app.StartButton.Enable = "on";
            app.StopButton.Enable = "off";
            app.SaveButton.Enable = "on";
        end
        
        function plotResults(app, time, velocityArray)
            plot(app.Axes, time, velocityArray);
            xlabel(app.Axes, 'Time (s)');
            ylabel(app.Axes, 'Velocity (m/s)');
            title(app.Axes, 'Velocity over Time');
            grid(app.Axes, 'on');
        end
    
        %% Button Functions
        % Button pushed function: SearchButton
        function SearchButtonPushed(app, event)
            device = [];
            app.Label.Text = "Searching for" + newline + "the Puck Sensor..."; 
            while (isempty(device))
                device = blelist("Name", "Puck Sensor");
            end
            app.DeviceName = device.Name;
            app.Label.Text = app.DeviceName + newline + "detected";
            app.ConnectButton.Enable = "on";
        end
        
        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            app.BLE = ble(app.DeviceName);  
            app.BLEChar = characteristic(app.BLE, ...
                "0A361000-115B-4E3F-80E6-05D7F5F2730E", ...
                "0A361001-115B-4E3F-80E6-05D7F5F2730E");
            app.BLEChar.DataAvailableFcn = @(src, event)app.processData(src, event);           
            app.Label.Text = "Connected to" + newline + app.DeviceName;
            app.ConnectButton.Enable = "off";
            app.DisconnectButton.Enable = "on";
            app.StartButton.Enable = "on";
        end

        % Button pushed function: DisconnectButton
        function DisconnectButtonPushed(app, event)
            unsubscribe(app.BLEChar);
            disconnect(app.BLE);
            app.BLEChar = [];
            app.BLE = [];
            app.Time = [];
            app.Label.Text = "Disconnected from" + newline + app.DeviceName;
            app.ConnectButton.Enable = "on";
            app.DisconnectButton.Enable = "off";
            app.StartButton.Enable = "off";
        end

        % Button pushed function: StartButton
        function StartButtonPushed(app, event)
            app.Time = datetime("now", "Format", "dd-MMM-uuuu HH:mm:ss.SSS");
            app.Data = [];
            app.Counter = app.Counter(end);
            app.AccelerometerValues = [];
            app.GyroscopeValues = [];
            app.HighGyroscopeValues = [];
            app.MagnetometerValues = [];
            app.Label.Text = "Getting data from" + newline + app.DeviceName;
            app.StartButton.Enable = "off";
            app.StopButton.Enable = "on";
        end

        % Button pushed function: SaveButton
        function SaveButtonPushed(app, event)
            TimeVar = duration(app.FinalTime - app.FinalTime(1), "Format", "mm:ss.SSS"); %#ok
            CounterVar = app.FinalCounter - app.Counter(1); %#ok
            RawData = app.FinalData; %#ok
            Accelerometer = app.FinalAccel; %#ok
            Gyroscope = [app.FinalGyro app.FinalHighGyro]; %#ok
            Magnetometer = app.FinalMag; %#ok
            Velocity = app.Vel; %#ok
            Orientation = app.Ornt'; %#ok
            uisave({'TimeVar', 'CounterVar', 'RawData', 'Accelerometer', 'Gyroscope', 'Magnetometer', 'Velocity', 'Orientation'})
        end

        % Button pushed function: CloseButton
        function CloseButtonPushed(app, event)
            delete(app)
        end
    end

    %% App GUI Functions
    methods (Access = public)
        function app = FilterAlgorithmApp()
            % Button width and height
            Width = 120;
            Height = 40;
            X = 20; % X-position (aligned to the left)
            Spacing = 10; % Space between buttons
            % createComponents(app);

            app.UIFigure = uifigure('Name', 'Performance Tracking Hockey Puck App Results', ...
                 'Position', [100, 100, 900, 700]);
            app.SearchButton = uibutton(app.UIFigure, 'Text', 'Search', ...
                 'Position', [X, 380, Width, Height], ...
                 'ButtonPushedFcn', @(~,~) app.SearchButtonPushed());
            app.ConnectButton = uibutton(app.UIFigure, 'Text', 'Connect', ...
                 'Position', [X, 350 - (Height + Spacing), Width, Height], ...
                 'ButtonPushedFcn', @(~,~) app.ConnectButtonPushed());
            app.DisconnectButton = uibutton(app.UIFigure, 'Text', 'Disconnect', ...
                 'Position', [X, 350 - 2*(Height + Spacing), Width, Height], ...
                 'ButtonPushedFcn', @(~,~) app.DisconnectButtonPushed());
            app.StartButton = uibutton(app.UIFigure, 'Text', 'Start', ...
                 'Position', [X, 350 - 3*(Height + Spacing), Width, Height], ...
                 'ButtonPushedFcn', @(~,~) app.StartButton());
            app.StopButton = uibutton(app.UIFigure, 'Text', 'Stop', ...
                 'Position', [X, 350 - 4*(Height + Spacing), Width, Height], ...
                 'ButtonPushedFcn', @(~,~) app.getResults());
            app.SaveButton = uibutton(app.UIFigure, 'Text', 'Save', ...
                 'Position', [X, 350 - 5*(Height + Spacing), Width, Height], ...
                 'ButtonPushedFcn', @(~,~) app.SaveButtonPushed());
            app.Label = uilabel(app.UIFigure, 'Text', 'No file loaded', ...
                 'Position', [20, 340, 200, 40]);
            app.Axes = uiaxes(app.UIFigure, 'Position', [200, 50, 680, 600]);  % Larger graph area
        end

        % function createComponents(app)
        %     % Create PuckSensorIMUSensorSystemUIFigure and hide until all components are created
        %     app.UIFigure = uifigure('Visible', 'on');
        %     app.UIFigure.Position = [100 100 640 480];
        %     app.UIFigure.Name = 'Puck Sensor IMU Sensor System';
        %     app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @PuckSensorIMUSensorSystemUIFigureCloseRequest, true);
        %     app.UIFigure.WindowState = 'maximized';
        % 
        %     % Create GridLayout
        %     app.GridLayout = uigridlayout(app.UIFigure);
        %     app.GridLayout.ColumnWidth = {'9x', '1x'};
        %     app.GridLayout.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};
        % 
        %     % Create ConnectButton
        %     app.ConnectButton = uibutton(app.GridLayout, 'push');
        %     app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
        %     app.ConnectButton.FontSize = 14;
        %     app.ConnectButton.Enable = 'off';
        %     app.ConnectButton.Layout.Row = 4;
        %     app.ConnectButton.Layout.Column = 2;
        %     app.ConnectButton.Text = 'Connect';
        % 
        %     % Create DisconnectButton
        %     app.DisconnectButton = uibutton(app.GridLayout, 'push');
        %     app.DisconnectButton.ButtonPushedFcn = createCallbackFcn(app, @DisconnectButtonPushed, true);
        %     app.DisconnectButton.FontSize = 14;
        %     app.DisconnectButton.Enable = 'off';
        %     app.DisconnectButton.Layout.Row = 5;
        %     app.DisconnectButton.Layout.Column = 2;
        %     app.DisconnectButton.Text = 'Disconnect';
        % 
        %     % Create StartButton
        %     app.StartButton = uibutton(app.GridLayout, 'push');
        %     app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
        %     app.StartButton.FontSize = 14;
        %     app.StartButton.Enable = 'off';
        %     app.StartButton.Layout.Row = 11;
        %     app.StartButton.Layout.Column = 2;
        %     app.StartButton.Text = 'Start';
        % 
        %     % Create StopButton
        %     app.StopButton = uibutton(app.GridLayout, 'push');
        %     app.StopButton.ButtonPushedFcn = createCallbackFcn(app, @getResults, true);
        %     app.StopButton.FontSize = 14;
        %     app.StopButton.Enable = 'off';
        %     app.StopButton.Layout.Row = 12;
        %     app.StopButton.Layout.Column = 2;
        %     app.StopButton.Text = 'Finish';
        % end
    end
end
