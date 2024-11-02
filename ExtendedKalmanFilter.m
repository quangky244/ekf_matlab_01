classdef ExtendedKalmanFilter
    properties
        A  % State transition matrix
        B  % Control input matrix
        Q  % Process noise covariance
        R  % Measurement noise covariance
        P  % Estimate error covariance
        x  % State estimate

        stateTransitionFunction
        measurementFunction
        jacobianStateFunction
        jacobianMeasurementFunction
    end
    
    methods
        function obj = ExtendedKalmanFilter(stateTransititonFunction, measurementFunction, jacobianStateFunction, jacobianMeasurementFunction, Q, R, P, x)
            obj.stateTransitionFunction = stateTransititonFunction;
            obj.measurementFunction = measurementFunction;
            obj.jacobianStateFunction = jacobianStateFunction;
            obj.jacobianMeasurementFunction = jacobianMeasurementFunction;
            obj.Q = Q;
            obj.R = R;
            obj.P = P;
            obj.x = x;
        end
        
        function obj = predict(obj, u)
            % Predict the state and estimate error covariance
            obj.x = obj.stateTransitionFunction(obj.x,u);
            F = obj.jacobianStateFunction(obj.x,u);
            obj.P = F * obj.P * F' + obj.Q;
        end
        
        function obj = update(obj, z)
            % Estimate error
            y_est = z - obj.measurementFunction(obj.x);
            
            H = obj.jacobianMeasurementFunction(obj.x);
            % Residual covariance
            S = H*obj.P*H' + obj.R;
            % Compute the Kalman gain
            K = obj.P * H' / S;
            
            % Update the state estimate and estimate error covariance
            obj.x = obj.x + K * y_est;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
        end
    end
end
