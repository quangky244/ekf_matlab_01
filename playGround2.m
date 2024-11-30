clear
init
% Define the system parameters
A = [0, 1; -.2, -.3];
B = [0.05; 1];
H = [1, 0];
Q = [0.001, 0; 0, 0.00001];
R = 0.01;
P = 1*eye(2);
x = [2; 0];

D = 0;

syst = ss(A,B,H,D);

T = 32e-3;
tspan = 0:T:50;
n_steps = length(tspan);
reshape(tspan,length(tspan),1);
uspan = ones(length(tspan),1);
% uspan = 1*sin(tspan*2*pi*1);
[y,~,x_real] = lsim(syst,uspan,tspan,x);
y = y + sqrt(R)*randn(n_steps,1);

syst_d = c2d(syst,T);
figure(1);clf(1);
step(syst); hold on;
step(syst_d);


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

x = [0;0];
% Create the Kalman Filter object
ekf = ExtendedKalmanFilter(f, h, F, H,Q,R,P, x);
% kf = KalmanFilter(A, B, H, Q, R, P, x);
u_hist = zeros((n_steps),1);
y_est_hist = zeros((n_steps),1);
x_hist = zeros((n_steps),2);
P_hist = zeros((n_steps),numel(x),numel(x));
x_hist(1,:) = x;
P_hist(1,:,:) = P;
for i = 2:length(tspan)
    % Predict and update steps
    u = uspan(i);  % Control input
    z = y(i);  % Measurement

    ekf = ekf.predict(u);
    ekf = ekf.update(z);
    P = ekf.P;
    y_est =  h(ekf.x);

    P_hist(i,:,:)= P;
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

misc.figure(3);

ax1 = nexttile(1);
plot(tspan,x_real(:,1),tspan,x_hist(:,1));
xlabel("Time (s)");
ylabel("State x1 ");
legend("True","Estimated");
title("Extended Kalman Filter","States Comparison")
ax2 = nexttile(2);
plot(tspan,x_real(:,2),tspan,x_hist(:,2));
xlabel("Time (s)");
ylabel("State x2 ");
legend("True","Estimated");
linkaxes([ax1,ax2],'x');

misc.figure(4);
ax1 = nexttile(1);
plot(tspan,x_hist(:,1)-x_real(:,1)); hold on;
plot(tspan,sqrt(P_hist(:,1,1)),'r');
plot(tspan,-sqrt(P_hist(:,1,1)),'r');
xlabel("Time [s]");
ylabel("Error state x1");
ax2 = nexttile(2);
plot(tspan,x_hist(:,2)-x_real(:,2)); hold on;
plot(tspan,sqrt(P_hist(:,2,2)),'r');
plot(tspan,-sqrt(P_hist(:,2,2)),'r');
xlabel("Time [s]");
ylabel("Error state x2");
linkaxes([ax1,ax2],'x');