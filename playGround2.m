clear
init
% Define the system parameters
A = [0, 1; -.2, -.3];
B = [0.05; 1];
H = [1, 0];
Q = [0.001, 0; 0, 0.001];
R = 0.5;
P = 10*eye(2);
x = [0; 0];

D = 0;

syst = ss(A,B,H,D);

T = 8e-3;
tspan = 0:T:50;
n_steps = length(tspan);
reshape(tspan,length(tspan),1);
uspan = ones(length(tspan),1);
% uspan = 1*sin(tspan*2*pi*1);
[y,~,x_real] = lsim(syst,uspan,tspan);
y = y + sqrt(R)*randn(n_steps,1);

syst_d = c2d(syst,T);
figure(1);clf(1);
step(syst); hold on;
step(syst_d);
%%





%% Define extended kalman filter, instead of Kalman
ode = @ (x,u) A*x + B*u; % continuous state transition
% for the formulated ekf, the transition must be discrete
% therefore the rk4 integration scheme is used to calculate
% the discrete time state transition
f =@(x,u) misc.rk4(ode,x,u,T);

% Define measurement function 
h = @(x) x(1); 

% define jacobian of state transition function
% d(ode)/dx = A -> discretize A = Ad
F = @(x,u) syst_d.A;

% define jacobian of measurement funciton
% dh/dx
H = @(x) [1 0]; 


% Create the Kalman Filter object
ekf = ExtendedKalmanFilter(f, h, F, H,Q,R,P, x);
% kf = KalmanFilter(A, B, H, Q, R, P, x);
u_hist = zeros(length(n_steps),1);
y_est_hist = zeros(length(n_steps),1);
x_hist = zeros(length(n_steps),2);
x_hist(1,:) = x;
for i = 2:length(tspan)
    % Predict and update steps
    u = uspan(i);  % Control input
    z = y(i);  % Measurement

    ekf = ekf.predict(u);
    ekf = ekf.update(z);

    y_est =  h(ekf.x);

    y_est_hist(i) = y_est;
    x_hist(i,:) = reshape(ekf.x,1,2);
end

% Display the state estimate
figure(2);clf(2);
plot(tspan,y,tspan,y_est_hist);
xlabel("Time (s)");
ylabel("System output");
legend("Measured","Estimated");
title("Extended Kalman Filter")
figure(3);clf(3);

subplot(2,1,1)
plot(tspan,x_real(:,1),tspan,x_hist(:,1));
xlabel("Time (s)");
ylabel("State x1 ");
legend("True","Estimated");
title("Extended Kalman Filter","States Comparison")
subplot(2,1,2)
plot(tspan,x_real(:,2),tspan,x_hist(:,2));
xlabel("Time (s)");
ylabel("State x2 ");
legend("True","Estimated");


