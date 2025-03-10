classdef FilterAlgorithmApp < matlab.apps.AppBase
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        GridLayout          matlab.ui.container.GridLayout
        LoadButton          matlab.ui.control.Button
        ProcessButton       matlab.ui.control.Button
        ConnectButton       matlab.ui.control.Button
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
    
    methods (Access = private)
        function loadData(app)
            [file, path] = uigetfile('*.mat', 'Select Data File');
            if isequal(file, 0)
                return;
            end
            app.Label.Text = ['Loaded: ', file];
            app.Data = load(fullfile(path, file));
        end
        
        function a = processAccel(app, data)
            a = swapbytes(typecast(reshape(uint8(data(:, 1:6)'), 1, []), "uint16"'));
            a = reshape(a, [3, 10])'; 
            a = double(a) * 4096 / 65535;
            a = a ./ [12.279 12.037 13.999];
            a = a * 9.81;
        end
                
        function g = processGyro(app, data)
            g = swapbytes(typecast(reshape(uint8(data(:, 7:12)'), 1, []), "uint16"));
            g = reshape(g, [3, 10])';
            g = double(g) * 4000 / 32768;
            g = deg2rad(g);
        end
        
        function hg = processHighGyro(app, data)
            hg = swapbytes(typecast(reshape(uint8(data(:, 13:14)'), 1, []), "uint16"));
            hg = reshape(hg, [1, 10])';
            hg = double(hg) * 4096 / 65535;
            hg = hg / 0.0955;
            hg = deg2rad(hg);
        end
        
        function m = processMag(app, data) 
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

        function processData(app)
            if isempty(app.Data)
                uialert(app.UIFigure, 'No data loaded!', 'Error');
                return;
            end
            
            data = app.Data;
            rawData = data.RawData;
            
            % Process data (same as original script, but optimized)
            numRows = size(rawData, 1);
            app.AccelerometerValues = [];
            app.GyroscopeValues = [];
            app.MagnetometerValues = [];
            app.HighGyroscopeValues = [];
            
            for i = 1:numRows
                fullData = reshape(rawData(i, 1:210), [21, 10])';
                % Append processed data
                app.AccelerometerValues = [app.AccelerometerValues; processAccel(app, fullData)];
                app.GyroscopeValues = [app.GyroscopeValues; processGyro(app, fullData)];
                app.MagnetometerValues = [app.MagnetometerValues; processMag(app, fullData)];
                app.HighGyroscopeValues = [app.HighGyroscopeValues; processHighGyro(app, fullData)];
            end
            
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
        end
        
        function plotResults(app, time, velocityArray)
            plot(app.Axes, time, velocityArray);
            xlabel(app.Axes, 'Time (s)');
            ylabel(app.Axes, 'Velocity (m/s)');
            title(app.Axes, 'Velocity over Time');
            grid(app.Axes, 'on');
        end
    end
    
    methods (Access = public)
        function app = FilterAlgorithmApp()
            app.UIFigure = uifigure('Name', 'Performance Tracking Hockey Puck App Results');
            app.LoadButton = uibutton(app.UIFigure, 'Text', 'Load Data', 'Position', [20, 350, 100, 30], 'ButtonPushedFcn', @(~,~) app.loadData());
            %app.ProcessButton = uibutton(app.UIFigure, 'Text', 'Process Data', 'Position', [140, 350, 100, 30], 'ButtonPushedFcn', @(~,~) app.processData());
            app.StartButton = uibutton(app.UIFigure, 'Text', 'Start', 'Position', [140, 350, 100, 30], 'ButtonPushedFcn', @(~,~) app.processData());
            app.Label = uilabel(app.UIFigure, 'Text', 'No file loaded', 'Position', [20, 320, 200, 30]);
            app.Axes = uiaxes(app.UIFigure, 'Position', [20, 20, 400, 280]);
        end

        function createComponents(app)
            % Create ConnectButton
            app.ConnectButton = uibutton(app.GridLayout, 'push');
            app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectButton.FontSize = 14;
            app.ConnectButton.Enable = 'off';
            app.ConnectButton.Layout.Row = 4;
            app.ConnectButton.Layout.Column = 2;
            app.ConnectButton.Text = 'Connect';
        
            % Create StartButton
            app.StartButton = uibutton(app.GridLayout, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            app.StartButton.FontSize = 14;
            app.StartButton.Enable = 'off';
            app.StartButton.Layout.Row = 11;
            app.StartButton.Layout.Column = 2;
            app.StartButton.Text = 'Start';
        
            % Create FinishButton
            app.FinishButton = uibutton(app.GridLayout, 'push');
            app.FinishButton.ButtonPushedFcn = createCallbackFcn(app, @FinishButtonPushed, true);
            app.FinishButton.FontSize = 14;
            app.FinishButton.Enable = 'off';
            app.FinishButton.Layout.Row = 12;
            app.FinishButton.Layout.Column = 2;
            app.FinishButton.Text = 'Finish';
        end
    end
end
