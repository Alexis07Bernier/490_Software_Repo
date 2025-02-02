classdef MadgwickAHRS
    properties
        SamplePeriod = 1/256; % Default sample period
        Beta = 0.1; % Algorithm gain
        Quaternion = [1 0 0 0]; % Initial quaternion
    end

    methods
        function obj = MadgwickAHRS(varargin)
            for i = 1:2:length(varargin)
                obj.(varargin{i}) = varargin{i+1};
            end
        end

        function obj = UpdateIMU(obj, gyro, accel)
            q = obj.Quaternion;
            beta = obj.Beta;
            dt = obj.SamplePeriod;

            % Normalize accelerometer measurement
            if norm(accel) == 0, return; end
            accel = accel / norm(accel);

            % Gradient descent algorithm corrective step
            f = [2*(q(2)*q(4) - q(1)*q(3)) - accel(1);
                 2*(q(1)*q(2) + q(3)*q(4)) - accel(2);
                 2*(0.5 - q(2)^2 - q(3)^2) - accel(3)];

            J = [-2*q(3),  2*q(4), -2*q(1),  2*q(2);
                  2*q(2),  2*q(1),  2*q(4),  2*q(3);
                  0,      -4*q(2), -4*q(3),  0];

            step = J' * f;
            step = step / norm(step);

            % Compute rate of change of quaternion
            gyro = deg2rad(gyro); % Convert to radians
            
            %qDot = 0.5 * quatmultiply(q, [0 gyro]) - beta * step';
            %Need the quatmultiply function in MATLAB but can't use it with MATLAB online
            %So implement it manually
            qDot = 0.5 * [ ...
                -q(2)*gyro(1) - q(3)*gyro(2) - q(4)*gyro(3);
                 q(1)*gyro(1) + q(3)*gyro(3) - q(4)*gyro(2);
                 q(1)*gyro(2) - q(2)*gyro(3) + q(4)*gyro(1);
                 q(1)*gyro(3) + q(2)*gyro(2) - q(3)*gyro(1)
            ] - beta * step';
            

            % Integrate to update quaternion
            q = q + qDot * dt;
            obj.Quaternion = q / norm(q); % Normalize quaternion
        end
    end
end
