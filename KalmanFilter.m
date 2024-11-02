classdef KalmanFilter
    properties
        A  % State transition matrix
        B  % Control input matrix
        H  % Observation matrix
        Q  % Process noise covariance
        R  % Measurement noise covariance
        P  % Estimate error covariance
        x  % State estimate
    end
    
    methods
        function obj = KalmanFilter(A, B, H, Q, R, P, x)
            obj.A = A;
            obj.B = B;
            obj.H = H;
            obj.Q = Q;
            obj.R = R;
            obj.P = P;
            obj.x = x;
        end
        
        function obj = predict(obj, u)
            % Predict the state and estimate error covariance
            obj.x = obj.A * obj.x + obj.B * u;
            obj.P = obj.A * obj.P * obj.A' + obj.Q;
        end
        
        function obj = update(obj, z)
            % Compute the Kalman gain
            K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + obj.R);
            
            % Update the state estimate and estimate error covariance
            obj.x = obj.x + K * (z - obj.H * obj.x);
            obj.P = (eye(size(obj.P)) - K * obj.H) * obj.P;
        end
    end
end
