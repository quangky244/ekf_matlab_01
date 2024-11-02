clear
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

T = .5;
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

% define state trainsition function
f = @(x, u) A * x + B*u; % state transition of continuous linear function

x_hist = zeros(n_steps,2);
x_hist(1,:) = x;

for i = 2:numel(tspan)    
    u = uspan(i);
    x = misc.rk4(f, x, u, T);
    x_hist(i,:) = reshape(x,1,length(x));
end

% Plot result 
figure(5);clf(5);
subplot(2,1,1)
plot(tspan,x_real(:,1),tspan,x_hist(:,1));
xlabel("Time (s)");
ylabel("State x1 ");
legend("lsim","rk4");
subplot(2,1,2)
plot(tspan,x_real(:,2),tspan,x_hist(:,2));
xlabel("Time (s)");
ylabel("State x2 ");
legend("lsim","rk4");