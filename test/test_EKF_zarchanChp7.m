% test the extended kalman filter

% Zarchan chapter 7 example
% states: x, xdot
% input : none
% output:
%
% Noise densities
% w ~ N([0;0], 0.1*eye(2))   - process noise
% v ~ N(0, 1000^2)             - measurement noise

clear;

dt = 0.1;   % [sec] measurement dt. Constant sample time, as opposed to calculated
            % based on previous time step. Simplified this example.


g = 32.2;   % [ft/sec^2] gravity acceleration constant
drag_coeff = 500; %

% air density equation
rho = @(x) 0.0034 * exp(-x/22000);

% dynamic pressure
qdyn = @(x) 0.5 * rho(x(1)) * x(2)^2;

% sizes
nx = 2;
ny = 1;
nu = 0;

% state (process) models
f = @(x, u, w) [ x(2);
                qdyn(x) * g / drag_coeff - g] + w;

% measurement model
h = @(x, v) [1 0] * x + v;

% jacobians
f21 = @(x) -rho(x(1)) * g * x(2)^2 / (44000 * drag_coeff);
f22 = @(x) rho(x(1)) * x(2) / drag_coeff;
F_equ = @(x) [0 1;f21(x) f22(x)]; % there is no dt here either
L_equ = @(x)  eye(nx);
H_equ = @(x) [1 0];
M_equ = @(x) eye(ny);


%Process Noise - Tuning the filter. Need Q matrix to avoid smug filter
PhiS = 0; % Process spectral density. Initially zero. Adjusted during filter tuning

% Page 235
Qk = @(x,dt) PhiS * [ dt^3/3                    dt^2/2 + f22(x)*dt^3/3 ;
                      dt^2/2 + f22(x)*dt^3/3    dt + f22(x)*dt^2  + f22(x)^2*dt^3/3];

Q_equ = Qk;

% Measurment Model
yObsNoise = 1000;  % [ft] 1-sigma noise on measurement (the radar altitude)
R = yObsNoise^2;
R_equ = @(x,dt) R;

% TODO: eliminate the callback, just assign the value with setters.
callbacks = {
   f
   h
   F_equ
   L_equ
   H_equ
   M_equ
   Q_equ
   R_equ
   };

%% initial conditions
x0 = 200000;      %[ft] initial altitude;
%x0_sigma = 25;
x0_sigma = 1000; % Make initial uncertainty large. Will see it converge quickly
v0 = -6000;        %[ft/sec] initial velocity;
%v0_sigma = 150;
v0_sigma = 6000;


xEst0 = [x0;v0];
P0 = diag([x0_sigma^2,v0_sigma^2]); % state cov
% remember that variance is the standard deviation squared

% initialize filter class with the initial state and state cov matrix
kf = EKF(xEst0, P0, callbacks);

% Run a simulation to get true data and noise corrupted data
t0 = 0;
tf = 30;
tspan = [0:dt:tf];
odefun = @(t,x) f(x,0,zeros(nx,1));
[t,y] = ode45(odefun,tspan,[x0;v0]);

% add noise
%randn


m = length(tspan);         % number of observations
n = nx;                    % number of states
%y = ;                     % measurement (observation) (from simulation)
u = zeros(m,1);            % input u

xEst = zeros(m,n);
PEst = zeros(n,n,m);

% initialze output
xEst(1,:) = xEst0';
PEst(:,:,1) = P0;

% run filter from the 2nd data point through m
for i=2:m
   [x, p] = kf.step(dt,u(i),y(i,1)+yObsNoise*randn(1));
   xEst(i,:) = x';
   PEst(:,:,i) = p;
end

% basic simulation
close all;
figure(1)
plot(t,y(:,1)); grid on;
title('height');

figure(2);
plot(t,y(:,2)); grid on;
title('velocity');


%plot the error between the estimate and truth
figure(3);
err = xEst(:,1) - y(:,1);
err_1sigma = sqrt(squeeze(PEst(1,1,:)));
plot(t,err,t,err_1sigma,'r--',t,-err_1sigma,'r--'); grid on;
title('position error');
ylim(600*[-1 1]);


figure(4);
err = xEst(:,2) - y(:,2);
err_1sigma = sqrt(squeeze(PEst(2,2,:)));
plot(t,err,t,err_1sigma,'r--',t,-err_1sigma,'r--'); grid on;
title('velocity error');
ylim(150*[-1 1]);







