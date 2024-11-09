clear
% Define the system parameters
A = [0, 1; -.2, -.3];
B = [0.05; 1];
H = [1, 0];
Q = [0.1, 0; 0, 0.0001];
R = 0.5;
P = 10*eye(2);
x = [10; 0];

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

% Create the Kalman Filter object
kf = KalmanFilter(syst_d.A, syst_d.B, syst_d.C, Q, R, P, x);
% kf = KalmanFilter(A, B, H, Q, R, P, x);
u_hist = zeros(length(n_steps),1);
y_est_hist = zeros(length(n_steps),1);
x_hist = zeros(length(n_steps),2);
x_hist(1,:) = x;
for i = 2:length(tspan)
    % Predict and update steps
    u = uspan(i);  % Control input
    z = y(i);  % Measurement

    kf = kf.predict(u);
    kf = kf.update(z);

    y_est =  H*kf.x;

    y_est_hist(i) = y_est;
    x_hist(i,:) = reshape(kf.x,1,2);
end

% Display the state estimate
figure(2);clf(2);
plot(tspan,y,tspan,y_est_hist);
xlabel("Time (s)");
ylabel("System output");
legend("Measured","Estimated");

figure(3);clf(3);
subplot(2,1,1)
plot(tspan,x_real(:,1),tspan,x_hist(:,1));
xlabel("Time (s)");
ylabel("State x1 ");
legend("True","Estimated");
subplot(2,1,2)
plot(tspan,x_real(:,2),tspan,x_hist(:,2));
xlabel("Time (s)");
ylabel("State x2 ");
legend("True","Estimated");


